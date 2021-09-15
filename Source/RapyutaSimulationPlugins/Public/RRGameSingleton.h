// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
#include "CoreMinimal.h"

// UE
#include "Engine/StaticMesh.h"
#include "Engine/StreamableManager.h"
#include "Materials/Material.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Templates/UniquePtr.h"

// RapyutaSim
#include "RRObjectCommon.h"
#include "RapyutaSimulationPlugins.h"
#include "Tools/RRAssetUtils.h"
#include "Tools/RRTypeUtils.h"

#include "RRGameSingleton.generated.h"

UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API URRGameSingleton : public UObject
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
    bool CollateAssetsInfo(const ERRResourceDataType InDataType, const FString& InAssetRelativeFolderPath)
    {
        FRRResourceInfo& outResourceInfo = GetSimResourceInfo(InDataType);

        TArray<FAssetData> assetDataList;
        URRAssetUtils::LoadAssetDataList<T>(GetDynamicAssetsPath(InDataType) / InAssetRelativeFolderPath, assetDataList);
        for (const auto& asset : assetDataList)
        {
            UE_LOG(LogTemp,
                   VeryVerbose,
                   TEXT("[%s] ASSET [%s] [%s]"),
                   *asset.AssetName.ToString(),
                   *asset.PackagePath.ToString(),
                   *asset.GetFullName(),
                   *asset.ToSoftObjectPath().ToString());
            outResourceInfo.AddResource(asset.AssetName.ToString(), asset.ToSoftObjectPath().ToString(), nullptr);
        }
        return (assetDataList.Num() > 0);
    }

    // ASSETS --
    // This list specifically hosts names of which module houses the UE assets based on their data type
    static TMap<ERRResourceDataType, const TCHAR*> SASSET_OWNING_MODULE_NAMES;

    // This only returns base path of assets residing in Plugin, not from Project level, which should starts with [/Game/]
    // And please note that the Sim does not store assets in Project, just to make them accessible among plugins.
    static constexpr const TCHAR* ASSETS_ROOT_PATH = TEXT("/");
    static FString GetAssetsBasePath(const TCHAR* InModuleName)
    {
        // For particular handling, please set the asset path ending with '/'
        // This concatenation operator ensure only a single '/' is put in between
        return FString(ASSETS_ROOT_PATH) / InModuleName;
    }

    static constexpr const TCHAR* DYNAMIC_CONTENTS_FOLDER_NAME = TEXT("DynamicContents");
    static FString GetDynamicAssetsPath(const ERRResourceDataType InDataType)
    {
        static FString runtimeAssetsPath = GetAssetsBasePath(SASSET_OWNING_MODULE_NAMES[InDataType]) / DYNAMIC_CONTENTS_FOLDER_NAME;
        return runtimeAssetsPath;
    }

    //  RESOURCE STORE --
    //
    UFUNCTION()
    bool HaveAllResourcesBeenLoaded(bool bIsLogged = false);

    UFUNCTION()
    bool RequestResourcesLoading(const ERRResourceDataType InDataType);

    // This is used as param to [FStreamableDelegate::CreateUObject()] thus its params could not be constref-ized
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

            default:
                break;
        }
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
            // Update [ResourceMap] with the newly loaded resource --
            FRRResourceInfo& resourceInfo = GetSimResourceInfo(InDataType);
            resourceInfo.AddResource(InResourceUniqueName, InResourcePath, resource);
            resourceInfo.ToBeAsyncLoadedResourceNum--;
            UE_LOG(LogTemp,
                   VeryVerbose,
                   TEXT("%d [%s] [%s:%s] RESOURCE LOADED %d"),
                   resourceInfo.ToBeAsyncLoadedResourceNum,
                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                   *InResourceUniqueName,
                   *InResourcePath.ToString(),
                   resource);
            if (resourceInfo.ToBeAsyncLoadedResourceNum == 0)
            {
                resourceInfo.HasBeenAllLoaded = true;
            }

            // Resource Data --
            // Still need to store resource handle in a direct UPROPERTY() child TArray of this GameSingleton to bypass
            // early GC
            ResourceStore.AddUnique(Cast<UObject>(resource));
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
            // For some reason, [LogRapyutaCore] could not be used here due to a linking error as being invoked from project
            // sources.
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

    TMap<FString, FRRResource>& GetSimResourceList(const ERRResourceDataType InDataType)
    {
        verifyf(ResourceMap.Contains(InDataType),
                TEXT("It seems [ResourceMap][%s] not yet fully initialized!"),
                *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType));
        return GetSimResourceInfo(InDataType).Data;
    }

    FRRResourceInfo& GetSimResourceInfo(const ERRResourceDataType InDataType)
    {
        verifyf(ResourceMap.Contains(InDataType),
                TEXT("It seems [ResourceMap][%s] has not yet been fully initialized!"),
                *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType));
        return ResourceMap[InDataType];
    }

    // STATIC MESHES --
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_STATIC_MESHES = TEXT("StaticMeshes");

    // These names must match ones defined in [StaticMeshShapesInfoFileName] file
    // Here we only define specially used shapes for some specific purpose!
    static constexpr const TCHAR* SHAPE_NAME_PLANE = TEXT("Plane");
    static constexpr const TCHAR* SHAPE_NAME_CUBE = TEXT("Cube");
    static constexpr const TCHAR* SHAPE_NAME_CYLINDER = TEXT("Cylinder");
    static constexpr const TCHAR* SHAPE_NAME_SPHERE = TEXT("Sphere");

    UFUNCTION()
    FORCEINLINE UStaticMesh* GetStaticMesh(const FString& InStaticMeshName)
    {
        return GetSimResource<UStaticMesh>(ERRResourceDataType::UE_STATIC_MESH, InStaticMeshName);
    }

    // MATERIALS --
    // Material Entities Info folder path
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_MATERIALS = TEXT("Materials");

    // Here we only define specially used materials for some specific purpose!
    static constexpr const TCHAR* MATERIAL_NAME_FLOOR = TEXT("M_FloorMat");

    UFUNCTION()
    FORCEINLINE UMaterialInterface* GetMaterial(const FString& InMaterialName)
    {
        return GetSimResource<UMaterialInterface>(ERRResourceDataType::UE_MATERIAL, InMaterialName);
    }

private:
    // Async loaded, thus must be thread safe. A map just helps referencing an item faster, though costs some overheads.
    // Besides, UE does not support UPROPERTY() on a map yet.
    TMap<ERRResourceDataType, FRRResourceInfo> ResourceMap;

    // We need this to escape UObject-based resource Garbage Collection
    UPROPERTY()
    TArray<UObject*> ResourceStore;
};
