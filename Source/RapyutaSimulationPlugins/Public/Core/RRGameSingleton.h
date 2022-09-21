/**
 * @file RRGameSingleton.h
 * @brief GameSingleton
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
#include "CoreMinimal.h"

// UE
#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "Engine/StreamableManager.h"
#include "Materials/Material.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "Templates/UniquePtr.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRAssetUtils.h"
#include "Core/RRObjectCommon.h"
#include "Core/RRTypeUtils.h"
#include "RapyutaSimulationPlugins.h"

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
    bool InitializeResources();
    void FinalizeResources();

    template<typename T>
    bool CollateAssetsInfo(const ERRResourceDataType InDataType, const FString& InAssetRelativeFolderPath)
    {
        FRRResourceInfo& outResourceInfo = GetSimResourceInfo(InDataType);

        TArray<FAssetData> totalAssetDataList;
        for (const auto& assetsPath : GetDynamicAssetsPathList(InDataType))
        {
            TArray<FAssetData> assetDataList;
            URRAssetUtils::LoadAssetDataList<T>(assetsPath / InAssetRelativeFolderPath, assetDataList);
            totalAssetDataList.Append(assetDataList);
        }

        for (const auto& asset : totalAssetDataList)
        {
#if RAPYUTA_SIM_DEBUG
            UE_LOG(LogTemp,
                   Warning,
                   TEXT("[%s] ASSET [%s] [%s]"),
                   *asset.AssetName.ToString(),
                   *asset.PackagePath.ToString(),
                   *asset.GetFullName(),
                   *asset.ToSoftObjectPath().ToString());
#endif
            outResourceInfo.AddResource(asset.AssetName.ToString(), asset.ToSoftObjectPath().ToString(), nullptr);
        }
        return (totalAssetDataList.Num() > 0);
    }

    // ASSETS --
    // This list specifically hosts names of which module houses the UE assets based on their data type
    static TMap<ERRResourceDataType, TArray<const TCHAR*>> SASSET_OWNING_MODULE_NAMES;

    // This only returns base path of assets residing in Plugin, not from Project level, which should starts with [/Game/]
    // And please note that the Sim does not store assets in Project, just to make them accessible among plugins.
    static constexpr const TCHAR* ASSETS_ROOT_PATH = TEXT("/");
    static constexpr const TCHAR* ASSETS_PROJECT_MODULE_NAME = TEXT("Game/RapyutaContents");
    static FString GetAssetsBasePath(const TCHAR* InModuleName)
    {
        // For particular handling, please set the asset path ending with '/'
        // This concatenation operator ensure only a single '/' is put in between
        return FString(ASSETS_ROOT_PATH) / InModuleName;
    }

    static constexpr const TCHAR* DYNAMIC_CONTENTS_FOLDER_NAME = TEXT("DynamicContents");
    static TArray<FString> GetDynamicAssetsPathList(const ERRResourceDataType InDataType)
    {
        static TArray<FString> runtimeAssetsPathList;
        for (const auto& moduleName : SASSET_OWNING_MODULE_NAMES[InDataType])
        {
            runtimeAssetsPathList.Emplace(GetAssetsBasePath(moduleName) / DYNAMIC_CONTENTS_FOLDER_NAME);
        }
        return runtimeAssetsPathList;
    }

    //  RESOURCE STORE --
    //
    bool HaveAllResourcesBeenLoaded(bool bIsLogged = false) const;
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

    /**
     * @brief
     * Callback to handle an asynchronously loaded resource, which receives a [FSoftObjectPath] that references a valid [UObject]
     * Then put it into both [ResourceMap] & [ResourceStore].
     *
     * @tparam TResource
     * @param InDataType
     * @param InResourcePath
     * @param InResourceUniqueName
     * @return bool
     */
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
#if RAPYUTA_SIM_DEBUG
            UE_LOG(LogTemp,
                   Warning,
                   TEXT("%d [%s] [%s:%s] RESOURCE LOADED %d"),
                   resourceInfo.ToBeAsyncLoadedResourceNum,
                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                   *InResourceUniqueName,
                   *InResourcePath.ToString(),
                   resource);
#endif
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
    FORCEINLINE void AddDynamicResource(const ERRResourceDataType InDataType,
                                        TResource* InResourceObject,
                                        const FString& InResourceUniqueName)
    {
        // Update [ResourceMap] with dynamically runtime-generated [InResourceObject]
        // of which soft object path is also created on the fly.
        FRRResourceInfo& resourceInfo = GetSimResourceInfo(InDataType);
        // (Note) FSoftObjectPath only accepts legit package names, not [InResourceUniqueName] like an arbitrary one
        resourceInfo.AddResource(InResourceUniqueName, FSoftObjectPath(InResourceObject), InResourceObject);
        resourceInfo.HasBeenAllLoaded = true;

#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogTemp,
               Warning,
               TEXT("[%s] [%s] DYNAMIC RUNTIME RESOURCE ADDED %s"),
               *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
               *InResourceUniqueName,
               *InResourceObject->GetName());
#endif

        // Resource Data
        // Still need to store resource handle in a direct UPROPERTY() child TArray of this GameSingleton to bypass
        // early GC
        if (IsValid(InResourceObject))
        {
            ResourceStore.AddUnique(Cast<UObject>(InResourceObject));
        }
    }

    template<typename TResource>
    TResource* GetSimResource(const ERRResourceDataType InDataType,
                              const FString& InResourceUniqueName,
                              bool bIsStaticResource = true) const
    {
        TResource* resourceAsset = Cast<TResource>(GetSimResourceInfo(InDataType).Data.FindRef(InResourceUniqueName).AssetData);

        if (bIsStaticResource && (!resourceAsset))
        {
            // For some reason, [LogRapyutaCore] could not be used here due to a linking error as being invoked from project
            // sources.
            UE_LOG(LogTemp,
                   Fatal,
                   TEXT("[%s] [Unique Name: %s] INVALID STATIC RESOURCE %d!"),
                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                   *InResourceUniqueName,
                   resourceAsset)
            return nullptr;
        }
        else if (resourceAsset && !resourceAsset->IsValidLowLevelFast())
        {
            UE_LOG(LogTemp,
                   Error,
                   TEXT("[%s] [Unique Name: %s] INVALID-AT-LOW-LEVEL RESOURCE %d!"),
                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                   *InResourceUniqueName,
                   resourceAsset)
            return nullptr;
        }
        return resourceAsset;
    }

    bool HasSimResource(const ERRResourceDataType InDataType, const FString& InResourceUniqueName) const
    {
        return GetSimResourceInfo(InDataType).Data.Contains(InResourceUniqueName);
    }

    const TMap<FString, FRRResource>& GetSimResourceList(const ERRResourceDataType InDataType) const
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

    const FRRResourceInfo& GetSimResourceInfo(const ERRResourceDataType InDataType) const
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
    static constexpr const TCHAR* SHAPE_NAME_CAPSULE = TEXT("Capsule");

    static const FString GetMeshNameFromShapeType(const ERRShapeType InShapeType)
    {
        switch (InShapeType)
        {
            case ERRShapeType::PLANE:
                return SHAPE_NAME_PLANE;
            case ERRShapeType::BOX:
                return SHAPE_NAME_CUBE;
            case ERRShapeType::CYLINDER:
                return SHAPE_NAME_CYLINDER;
            case ERRShapeType::SPHERE:
                return SHAPE_NAME_SPHERE;
            case ERRShapeType::CAPSULE:
                return SHAPE_NAME_CAPSULE;
            default:
                return EMPTY_STR;
        }
    }

    static ERRShapeType GetShapeTypeFromMeshName(const FString& InMeshName)
    {
        return InMeshName.Equals(SHAPE_NAME_PLANE)    ? ERRShapeType::PLANE
             : InMeshName.Equals(SHAPE_NAME_CUBE)     ? ERRShapeType::BOX
             : InMeshName.Equals(SHAPE_NAME_CYLINDER) ? ERRShapeType::CYLINDER
             : InMeshName.Equals(SHAPE_NAME_SPHERE)   ? ERRShapeType::SPHERE
             : InMeshName.Equals(SHAPE_NAME_CAPSULE)  ? ERRShapeType::CAPSULE
                                                      : ERRShapeType::MESH;
    }

    // (NOTE) StaticMesh could be dynamically created
    FORCEINLINE UStaticMesh* GetStaticMesh(const FString& InStaticMeshName, bool bIsStaticResource = true) const
    {
        return GetSimResource<UStaticMesh>(ERRResourceDataType::UE_STATIC_MESH, InStaticMeshName, bIsStaticResource);
    }

    // SKELETAL ASSETS --
    FORCEINLINE USkeletalMesh* GetSkeletalMesh(const FString& InSkeletalMeshName, bool bIsStaticResource = true)
    {
        return GetSimResource<USkeletalMesh>(ERRResourceDataType::UE_SKELETAL_MESH, InSkeletalMeshName, bIsStaticResource);
    }

    FORCEINLINE USkeleton* GetSkeleton(const FString& InSkeletonName, bool bIsStaticResource = true)
    {
        return GetSimResource<USkeleton>(ERRResourceDataType::UE_SKELETON, InSkeletonName, bIsStaticResource);
    }

    FORCEINLINE UPhysicsAsset* GetPhysicsAsset(const FString& InPhysicsAssetName, bool bIsStaticResource = true)
    {
        return GetSimResource<UPhysicsAsset>(ERRResourceDataType::UE_PHYSICS_ASSET, InPhysicsAssetName, bIsStaticResource);
    }

    // MATERIALS --
    // Material Entities Info folder path
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_MATERIALS = TEXT("Materials");

    // Here we only define specially used materials for some specific purpose!
    static constexpr const TCHAR* MATERIAL_NAME_ASSET_MASTER = TEXT("M_RapyutaAssetMaster");
    static constexpr const TCHAR* MATERIAL_NAME_PROP_MASTER = TEXT("M_RapyutaPropMaster");
    static constexpr const TCHAR* MATERIAL_NAME_PHYSICS_WHEEL = TEXT("PM_Wheel");

    FORCEINLINE UMaterialInterface* GetMaterial(const FString& InMaterialName) const
    {
        return GetSimResource<UMaterialInterface>(ERRResourceDataType::UE_MATERIAL, InMaterialName);
    }
    FORCEINLINE UPhysicalMaterial* GetPhysicalMaterial(const FString& InPhysicalMaterialName)
    {
        return GetSimResource<UPhysicalMaterial>(ERRResourceDataType::UE_MATERIAL, InPhysicalMaterialName);
    }

    // TEXTURES --
    FORCEINLINE UTexture* GetTexture(const FString& InTextureName) const
    {
        return GetSimResource<UTexture>(ERRResourceDataType::UE_TEXTURE, InTextureName);
    }

    // BODY SETUPS --
    FORCEINLINE UBodySetup* GetBodySetup(const FString& InBodySetupName) const
    {
        // Body setups are dynamically created, thus not static resources
        return GetSimResource<UBodySetup>(ERRResourceDataType::UE_BODY_SETUP, InBodySetupName, false);
    }

private:
    // Async loaded, thus must be thread safe. A map just helps referencing an item faster, though costs some overheads.
    // Besides, UE does not support UPROPERTY() on a map yet.
    TMap<ERRResourceDataType, FRRResourceInfo> ResourceMap;

    // We need this to escape UObject-based resource Garbage Collection
    UPROPERTY()
    TArray<UObject*> ResourceStore;
};
