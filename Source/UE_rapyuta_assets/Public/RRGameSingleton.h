#pragma once
#include "CoreMinimal.h"

// UE
#include "Engine/AssetManager.h"
#include "Engine/StreamableManager.h"
#include "Materials/Material.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Templates/UniquePtr.h"

// RapyutaSim
#include "RRObjectCommon.h"
#include "Tools/RRAssetUtils.h"
#include "Tools/RRTypeUtils.h"
#include "UE_rapyuta_assets.h"

#include "RRGameSingleton.generated.h"

UCLASS(Config = RapyutaSimSettings)
class UE_RAPYUTA_ASSETS_API URRGameSingleton : public UObject
{
    GENERATED_BODY()
protected:
    URRGameSingleton();

public:
    static URRGameSingleton* Get();
    virtual ~URRGameSingleton();

    // SIM RESOURCES ==
    //
    UFUNCTION()
    bool InitializeResources();
    UFUNCTION()
    void FinalizeResources();

    template<typename T>
    bool CollateAssetsMetaData(const FString& InAssetFolderPath, FRRResourceInfo& OutResource)
    {
        TArray<FAssetData> assetDataList;
        URRAssetUtils::LoadAssetDataList<T>(InAssetFolderPath, assetDataList);
        for (const auto& asset : assetDataList)
        {
            UE_LOG(LogTemp,
                   VeryVerbose,
                   TEXT("[%s] ASSET [%s] [%s]"),
                   *asset.AssetName.ToString(),
                   *asset.PackagePath.ToString(),
                   *asset.GetFullName(),
                   *asset.ToSoftObjectPath().ToString());
            OutResource.AddResource(asset.AssetName.ToString(), asset.ToSoftObjectPath().ToString(), nullptr);
        }
        return (assetDataList.Num() > 0);
    }

    // ASSETS --
    // This list specifically host names of which module houses the UE assets based on their data type
    static TMap<ERRResourceDataType, const TCHAR*> SASSET_OWNING_MODULE_NAMES;

    // (snote) This only returns base path of assets residing in Plugin, not from Project level, which should starts with [/Game/]
    // And please note that the Sim does not store assets in Project, just to make them accessible among plugins.
    static constexpr const TCHAR* ASSETS_ROOT_PATH = TEXT("/");
    static FString GetAssetsBasePath(const TCHAR* InModuleName)
    {
        // For particular handling, please set the asset path ending with '/'
        // (snote) This concatenation operator ensure only a single '/' is put in between
        return FString(ASSETS_ROOT_PATH) / InModuleName;
    }

    static constexpr const TCHAR* DYNAMIC_CONTENTS_FOLDER_NAME = TEXT("DynamicContents");
    static FString GetDynamicAssetsPath(const ERRResourceDataType InDataType)
    {
        static FString runtimeAssetsPath = GetAssetsBasePath(SASSET_OWNING_MODULE_NAMES[InDataType]) / DYNAMIC_CONTENTS_FOLDER_NAME;
        return runtimeAssetsPath;
    }

    // All assets under [GetDynamicAssetsPath()]
    // UPROPERTY()
    // TArray<FAssetData> AssetDataList;
    // void FetchAllAssetsDataList();

    // template<typename T>
    // FORCEINLINE T* GetAssetObject(const FString& InAssetName) const
    //{
    //    for (const auto& asset : AssetDataList)
    //    {
    //        if (asset.AssetName.ToString() == InAssetName)
    //        {
    //            // This will load the asset if needed then return it
    //            return Cast<T>(asset.GetAsset());
    //        }
    //    }
    //    return nullptr;
    //}

    //  RESOURCE STORE --
    //
    UFUNCTION()
    bool HaveAllResourcesBeenLoaded(bool bIsLogged = false);

    UFUNCTION()
    bool RequestResourcesLoading(const ERRResourceDataType InDataType, const FRRResourceInfo& InResourceInfo);

    FORCEINLINE void OnResourceLoaded(ERRResourceDataType InDataType, FSoftObjectPath InResourcePath, FString InResourceUniqueName)
    {
        check(IsInGameThread());

        switch (InDataType)
        {
            case ERRResourceDataType::UE_STATIC_MESH:
                ProcessAsyncLoadedResource<UStaticMesh>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_MATERIAL:
                // Either [UMaterialInterface] or [UPhysicalMaterial] type that is resolved into a non-null resource will be
                // processed
                if (!ProcessAsyncLoadedResource<UMaterialInterface>(InDataType, InResourcePath, InResourceUniqueName))
                {
                    ProcessAsyncLoadedResource<UPhysicalMaterial>(InDataType, InResourcePath, InResourceUniqueName);
                }
                break;

            case ERRResourceDataType::UE_TEXTURE:
                ProcessAsyncLoadedResource<UTexture2D>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            default:
                break;
        }
    }

    // Callback to handle an synchronously loaded resource, which receives some having been loaded [UObject] handle
    // Then put it into both [ResourceMap] & [ResourceStore]
    template<typename TResource>
    FORCEINLINE bool ProcessSyncLoadedResource(const ERRResourceDataType InDataType,
                                               UObject* InResource,
                                               const FString& InResourceUniqueName)
    {
        verify(IsInGameThread());
        TResource* resource = Cast<TResource>(InResource);

        if (resource)
        {
            // Resource Info --
            FRRResourceInfo& simResourceInfo = GetSimResourceInfo(InDataType);
            simResourceInfo.AddResource(InResourceUniqueName, resource->GetPathName(), resource);
            UE_LOG(LogTemp,
                   VeryVerbose,
                   TEXT("[%s] [%s:%s] RESOURCE LOADED %d"),
                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                   *InResourceUniqueName,
                   *InResource->GetPathName(),
                   resource);

            // Resource Data --
            // (snote) Still need to store resource handle in a direct UPROPERTY() child TArray of this GameSingleton to bypass
            // early GC
            ResourceStore.AddUnique(InResource);
            return true;
        }
        return false;
    }

    // Callback to handle an asynchronously loaded resource, which receives a [FSoftObjectPath] that references a valid [UObject]
    // Then put it into both [ResourceMap] & [ResourceStore].
    template<typename TResource>
    FORCEINLINE bool ProcessAsyncLoadedResource(const ERRResourceDataType InDataType,
                                                const FSoftObjectPath& InResourcePath,
                                                const FString& InResourceUniqueName)
    {
        verify(IsInGameThread());
        TResource* resource = Cast<TResource>(InResourcePath.ResolveObject());

        if (resource)
        {
            // Resource Info --
            FRRResourceInfo& simResourceInfo = GetSimResourceInfo(InDataType);
            simResourceInfo.AddResource(InResourceUniqueName, InResourcePath, resource);
            simResourceInfo.ToBeAsyncLoadedResourceNum--;
            UE_LOG(LogTemp,
                   VeryVerbose,
                   TEXT("%d [%s] [%s:%s] RESOURCE LOADED %d"),
                   simResourceInfo.ToBeAsyncLoadedResourceNum,
                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                   *InResourceUniqueName,
                   *InResourcePath.ToString(),
                   resource)
            if (simResourceInfo.ToBeAsyncLoadedResourceNum == 0)
            {
                simResourceInfo.HasBeenAllLoaded = true;
            }

            // Resource Data --
            // (snote) Still need to store resource handle in a direct UPROPERTY() child TArray of this GameSingleton to bypass
            // early GC
            UObject* objectResource = Cast<UObject>(resource);
            if (objectResource)
            {
                ResourceStore.AddUnique(objectResource);
            }
            return true;
        }
        return false;
    }

    template<typename TResource>
    TResource* GetSimResource(const ERRResourceDataType InDataType, const FString& InResourceUniqueName)
    {
        TResource* resourceAsset = Cast<TResource>(GetSimResourceInfo(InDataType).Data.FindRef(InResourceUniqueName).AssetData);

        if (!resourceAsset || !resourceAsset->IsValidLowLevelFast())
        {
            UE_LOG(LogTemp,
                   Fatal,
                   TEXT("[%s] [Unique Name: %s] INVALID RESOURCE %d!"),
                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                   *InResourceUniqueName,
                   resourceAsset)
            return nullptr;
        }
        return resourceAsset;
    }

    TMap<FString, FRRResource>& GetSimResourceList(const ERRResourceDataType DataType)
    {
        verifyf(ResourceMap.Contains(DataType),
                TEXT("It seems [ResourceMap][%s] not yet fully initialized!"),
                *URRTypeUtils::GetERRResourceDataTypeAsString(DataType)) return GetSimResourceInfo(DataType)
            .Data;
    }

    FRRResourceInfo& GetSimResourceInfo(const ERRResourceDataType DataType)
    {
        verifyf(ResourceMap.Contains(DataType),
                TEXT("It seems [ResourceMap][%s] not yet fully initialized!"),
                *URRTypeUtils::GetERRResourceDataTypeAsString(DataType));
        return ResourceMap[DataType];
    }

    // STATIC MESHES --
    // Mesh Entities Info folder path
    UPROPERTY(config)
    FString CFOLDER_PATH_SIM_ASSET_STATIC_MESHES = TEXT("StaticMeshes");

    // (snote) These names must match ones defined in [StaticMeshShapesInfoFileName] file
    // Here we only define specially used shapes for some specific purpose!
    static constexpr const TCHAR* CSHAPE_NAME_PLANE = TEXT("Shape_Plane");
    static constexpr const TCHAR* CSHAPE_NAME_CUBE = TEXT("Shape_Cube");
    static constexpr const TCHAR* CSHAPE_NAME_CYLINDER = TEXT("Shape_Cylinder");
    static constexpr const TCHAR* CSHAPE_NAME_SPHERE = TEXT("Shape_Sphere");

    UFUNCTION()
    FORCEINLINE UStaticMesh* GetStaticMesh(const FString& MeshName)
    {
        return GetSimResource<UStaticMesh>(ERRResourceDataType::UE_STATIC_MESH, MeshName);
    }

private:
    // Async loaded, thus must be thread safe. A map just helps referencing an item faster, though costs some overheads.
    TMap<ERRResourceDataType, FRRResourceInfo> ResourceMap;

    // We need this to bypass UObject-based resources' Garbage Collection
    UPROPERTY()
    TArray<UObject*> ResourceStore;
};
