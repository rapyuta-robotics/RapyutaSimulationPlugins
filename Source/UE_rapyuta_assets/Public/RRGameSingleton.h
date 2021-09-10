#pragma once
#include "CoreMinimal.h"

// UE
#include "Engine/AssetManager.h"
#include "Engine/StreamableManager.h"
#include "Templates/UniquePtr.h"

// RapyutaSim
#include "Tools/RRAssetUtils.h"
#include "Tools/RRTypeUtils.h"
#include "UE_rapyuta_assets.h"

#include "RRGameSingleton.generated.h"

// SIM RESOURCE DATA --
//
UENUM(BlueprintType)
enum class ERRResourceDataType : uint8
{
    NONE,
    // UASSET --
    STATIC_MESH,
    MATERIAL,
    TEXTURE,

    // URDF/SDF--
    URDF_SDF,

    TOTAL
};

// The atomic Sim resource
USTRUCT()
struct UE_RAPYUTA_ASSETS_API FRRResource
{
    GENERATED_BODY()
    FRRResource()
    {
    }
    FRRResource(const FString& InUniqueName, const FSoftObjectPath& InAssetPath, UObject* InAssetData)
        : UniqueName(InUniqueName), AssetPath(InAssetPath), AssetData(InAssetData)
    {
    }

    UPROPERTY()
    FString UniqueName;

    UPROPERTY()
    FSoftObjectPath AssetPath;
    FString GetAssetPath() const
    {
        return AssetPath.ToString();
    }

    UPROPERTY()
    UObject* AssetData = nullptr;
};

USTRUCT()
struct UE_RAPYUTA_ASSETS_API FRRResourceInfo
{
    GENERATED_BODY()
    FRRResourceInfo()
    {
    }
    FRRResourceInfo(ERRResourceDataType InDataType) : DataType(InDataType)
    {
    }

    UPROPERTY()
    ERRResourceDataType DataType = ERRResourceDataType::NONE;
    UPROPERTY()
    int32 ToBeAsyncLoadedResourceNum = 0;
    UPROPERTY()
    bool HasBeenAllLoaded = false;

    UPROPERTY()
    TMap<FString, FRRResource> Data;

    void AddResource(const FString& InUniqueName, const FSoftObjectPath& InAssetPath, UObject* InAssetData)
    {
        Data.Add(InUniqueName, FRRResource(InUniqueName, InAssetPath, InAssetData));
    }

    void Finalize()
    {
        DataType = ERRResourceDataType::NONE;
        ToBeAsyncLoadedResourceNum = 0;
        HasBeenAllLoaded = false;
        Data.Empty();
    }
};

UCLASS()
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
    void InitializeResources();
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

    virtual bool InitializeRuntimeResources();
    UFUNCTION()
    void SetResourcesLoadingResult(const ERRResourceDataType DataType, bool Result);

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

    static constexpr const TCHAR* STARTUP_CONTENT_FOLDER_NAME = TEXT("StartUp");
    static FString GetStartUpContentPath()
    {
        static FString startUpContentPath = GetAssetsBasePath(UE_RAPYUTA_ASSETS_MODULE_NAME) / STARTUP_CONTENT_FOLDER_NAME;
        return startUpContentPath;
    }

    // All assets under [GetStartUpContentPath()]
    UPROPERTY()
    TArray<FAssetData> AssetDataList;
    void FetchAllAssetsDataList();

    template<typename T>
    FORCEINLINE T* GetAssetObject(const FString& InAssetName) const
    {
        for (const auto& asset : AssetDataList)
        {
            if (asset.AssetName.ToString() == InAssetName)
            {
                // This will load the asset if needed then return it
                return Cast<T>(asset.GetAsset());
            }
        }
        return nullptr;
    }

    //  RESOURCE STORE --
    //
    UFUNCTION()
    bool HaveAllResourcesBeenLoaded(bool bIsLogged = false);

    bool RequestResourcesLoading(const ERRResourceDataType DataType, const FRRResourceInfo& InResourceInfo)
    {
        if (GetSimResourceInfo(DataType).HasBeenAllLoaded || (0 == resourceMetaData.Num()))
        {
            return false;
        }
        // REQUEST FOR LOADING THE RESOURCES ASYNCHRONOUSLY
        //
        GetSimResourceInfo(DataType).ToBeAsyncLoadedResourceNum = InResourceInfo.Data.Num();
        UE_LOG(LogTemp,
               Display,
               TEXT("[%s] TO BE LOADED NUM: %d"),
               *URRTypeUtils::GetERRResourceDataTypeAsString(DataType),
               ToBeAsyncLoadedResourceNum);
        UAssetManager* assetManager = UAssetManager::GetIfValid();
        if (assetManager)
        {
            for (const auto& resourceMeta : InResourceInfo.Data)
            {
                // https://docs.unrealengine.com/en-US/Resources/SampleGames/ARPG/BalancingBlueprintAndCPP/index.html
                // "Avoid Referencing Assets by String"
                FSoftObjectPath resourceSoftObjPath(resourceMeta.Value.GetAssetDataPath());
                assetManager->GetStreamableManager().RequestAsyncLoad(
                    resourceSoftObjPath,
                    FStreamableDelegate::CreateUObject(
                        this, &URRGameSingleton::OnResourceLoaded, DataType, resourceSoftObjPath, resourceMeta.Value.UniqueName));
            }
            return true;
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[InitializeResources] UNABLE TO GET ASSET MANAGER!"))
            return false;
        }
    }

    FORCEINLINE void OnResourceLoaded(ERRResourceDataType DataType, FSoftObjectPath ResourcePath, FString ResourceUniqueName)
    {
        verify(IsInGameThread());

        switch (DataType)
        {
            case ERRResourceDataType::STATIC_MESH:
                ProcessAsyncLoadedResource<UStaticMesh>(DataType, ResourcePath, ResourceUniqueName);
                break;

            case ERRResourceDataType::MATERIAL:
                // Either [UMaterialInterface] or [UPhysicalMaterial] type that is resolved into a non-null resource will be
                // processed
                if (!ProcessAsyncLoadedResource<UMaterialInterface>(DataType, ResourcePath, ResourceUniqueName))
                {
                    ProcessAsyncLoadedResource<UPhysicalMaterial>(DataType, ResourcePath, ResourceUniqueName);
                }
                break;

            case ERRResourceDataType::TEXTURE:
                ProcessAsyncLoadedResource<UTexture2D>(DataType, ResourcePath, ResourceUniqueName);
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
    TResource* GetSimResource(const ERRResourceDataType DataType, const FString& ResourceUniqueName)
    {
        TResource* resourceAsset = Cast<TResource>(GetSimResourceInfo(DataType).Data.FindRef(ResourceUniqueName).AssetData);

        if (!resourceAsset || !resourceAsset->IsValidLowLevelFast())
        {
            UE_LOG(LogTemp,
                   Fatal,
                   TEXT("[%s] [Unique Name: %s] INVALID RESOURCE %d!"),
                   *URRTypeUtils::GetERRResourceDataTypeAsString(DataType),
                   *ResourceUniqueName,
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
    FString CFOLDER_PATH_SIM_ASSET_STATIC_MESHES = TEXT("StartUp/StaticMeshes");

    // (snote) These names must match ones defined in [StaticMeshShapesInfoFileName] file
    // Here we only define specially used shapes for some specific purpose!
    static constexpr const TCHAR* CSHAPE_NAME_PLANE = TEXT("Shape_Plane");
    static constexpr const TCHAR* CSHAPE_NAME_CUBE = TEXT("Shape_Cube");
    static constexpr const TCHAR* CSHAPE_NAME_CYLINDER = TEXT("Shape_Cylinder");
    static constexpr const TCHAR* CSHAPE_NAME_SPHERE = TEXT("Shape_Sphere");

    UFUNCTION()
    FORCEINLINE UStaticMesh* GetStaticMesh(const FString& MeshName)
    {
        return GetSimResource<UStaticMesh>(ERRResourceDataType::STATIC_MESH, MeshName);
    }

private:
    // Async loaded, thus must be thread safe. A map just helps referencing an item faster, though costs some overheads.
    TMap<ERRResourceDataType, FRRResourceInfo> ResourceMap;

    // We need this to bypass UObject-based resources' Garbage Collection
    UPROPERTY()
    TArray<UObject*> ResourceStore;
};
