/**
 * @file RRGameSingleton.h
 * @brief GameSingleton
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// Native
#include <type_traits>

// UE
#include "CoreMinimal.h"
#include "Engine/AssetManager.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "Engine/StreamableManager.h"
#include "Materials/Material.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Templates/UniquePtr.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRAssetUtils.h"
#include "Core/RRGeneralUtils.h"
#include "Core/RRObjectCommon.h"
#include "Core/RRTypeUtils.h"
#include "RapyutaSimulationPlugins.h"

#include "RRGameSingleton.generated.h"

class URRPakLoader;

template<const ERRResourceDataType InDataType>
using URRAssetObject = typename std::conditional<
    (ERRResourceDataType::UE_STATIC_MESH == InDataType),
    UStaticMesh,
    typename std::conditional<(ERRResourceDataType::UE_SKELETAL_MESH == InDataType),
        USkeletalMesh,
        typename std::conditional<(ERRResourceDataType::UE_SKELETON == InDataType),
            USkeleton,
            typename std::conditional<(ERRResourceDataType::UE_PHYSICS_ASSET == InDataType),
                UPhysicsAsset,
                typename std::conditional<(ERRResourceDataType::UE_MATERIAL == InDataType),
                    UMaterialInterface,
                    typename std::conditional<(ERRResourceDataType::UE_PHYSICAL_MATERIAL == InDataType),
                        UPhysicalMaterial,
                        typename std::conditional<(ERRResourceDataType::UE_TEXTURE == InDataType),
                            UTexture,
                            typename std::conditional<(ERRResourceDataType::UE_DATA_TABLE == InDataType),
                                UDataTable,
                                typename std::conditional<(ERRResourceDataType::UE_BODY_SETUP == InDataType),
                                    UBodySetup,
                                    UObject>::type>::type>::type>::type>::type>::type>::type>::type>::type;

/**
 * @brief GameSingleton class which handles asset loading.
 * GameSingleton class can exist during editor usage.
 * - #InitializeResources will async load data into #ResourceMap for each #ERRResourceDataType
 * - Get Asset meta data with URRAssetUtils.
 * - Load data with [UAssetManager](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UAssetManager/)
 * @sa [GameSingleton](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UEngine/GameSingleton/)
 * @sa [AsyncLoading](https://docs.unrealengine.com/5.1/en-US/asynchronous-asset-loading-in-unreal-engine/)
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API URRGameSingleton : public UObject
{
    GENERATED_BODY()
protected:
    URRGameSingleton();

public:
    static URRGameSingleton* Get();
    virtual ~URRGameSingleton();

    virtual void PrintSimConfig() const;

    // SIM GLOBAL PROPERTIES --
    //
    UPROPERTY(config)
    bool BSIM_PROFILING = false;

    // SIM RESOURCES ==
    //
    /**
     * @brief Read all sim dynamic resouces(Uassets) info from designated folders
     * @param bInRequestResourceLoading
     * - True: Load all dynamic-contents-designated resources
     * - False: Only collate resources metadata, LoadObject<>() will be invoked on-the-fly during Sim run in [GetSimResource()]
     * @return true if inited successfully
     */
    bool InitializeResources(bool bInRequestResourceLoading = false);

    /**
     * @brief Finalize #ResourceMap by calling #FRRResourceInfo::Finalize
     *
     */
    void FinalizeResources();

    // PAKS --
    UPROPERTY(Config)
    FString FOLDER_PATH_ASSET_PAKS = TEXT("Paks");
    UPROPERTY()
    TObjectPtr<URRPakLoader> PakLoader = nullptr;
    //! Whether PakLoader has been initialized, which is only true as in packaged game
    UPROPERTY()
    bool bPakLoaderInitialized = false;
    /**
     * @brief Load PAK files of a specific set of entity models, mounting then then collate their content assets info
     * @param InEntityModelName
     * @param bInForceReload
     * @return true/false
     */
    bool CollateEntityAssetsInfoFromPAK(const TArray<FString>& InEntityModelNameList, bool bInForceReload = false);

    // ASSETS --
    //! This list specifically hosts names of which module houses the UE assets based on their data type
    static TMap<ERRResourceDataType, TArray<const TCHAR*>> SASSET_OWNING_MODULE_NAMES;

    //! [ASSETS_ROOT_PATH] only returns base path of assets residing in Plugin, not from Project level, which should starts with [/Game/]
    //! And ideally, Sim should not store common runtime-created assets in Project, since they should be accessible among plugins.
    static constexpr const TCHAR* ASSETS_ROOT_PATH = TEXT("/");
    static constexpr const TCHAR* ASSETS_PROJECT_BASE_MODULE_NAME = TEXT("Game");
    static constexpr const TCHAR* ASSETS_PROJECT_MODULE_NAME = TEXT("Game/RapyutaContents");

    //! Base path whereby runtime-created blueprint classes are saved, ideally in Project, so it could reference all plugins' assets.
    UPROPERTY(Config)
    FString ASSETS_RUNTIME_BP_SAVE_BASE_PATH = TEXT("/Game/RapyutaContents/Blueprints");

    /**
     * @brief Get UE asset path of a dynamically created BP asset
     * @param InBPAssetName
     * @return as #ASSETS_RUNTIME_BP_SAVE_BASE_PATH / InBPAssetName
     * @note Dynamic BP assets are loaded by UE itself, not by Sim resource early loading as other dynamic resources (static meshes, textures, etc.).
     * Also, in order for them to reference any other arbitrary asset, it's advisable to save them into project's Content (not into plugins)
     */
    FString GetDynamicBPAssetPath(const FString& InBPAssetName)
    {
        return FString(ASSETS_RUNTIME_BP_SAVE_BASE_PATH) / InBPAssetName;
    }

    /**
     * @brief Get name prefix of asset
     * @param InDataType
     * @sa [Asset naming convention](https://docs.unrealengine.com/5.2/en-US/recommended-asset-naming-conventions-in-unreal-engine-projects)
     */
    FORCEINLINE static constexpr const TCHAR* GetAssetNamePrefix(const ERRResourceDataType InDataType)
    {
        switch (InDataType)
        {
            case ERRResourceDataType::UE_STATIC_MESH:
                return TEXT("SM_");

            case ERRResourceDataType::UE_SKELETAL_MESH:
                return TEXT("SK_");

            case ERRResourceDataType::UE_SKELETON:
                return TEXT("SKEL_");

            case ERRResourceDataType::UE_PHYSICS_ASSET:
                return TEXT("PHYS_");

            case ERRResourceDataType::UE_MATERIAL:
                return TEXT("M_");
            case ERRResourceDataType::UE_PHYSICAL_MATERIAL:
                return TEXT("PM_");

            case ERRResourceDataType::UE_TEXTURE:
                return TEXT("T_");

            case ERRResourceDataType::UE_DATA_TABLE:
                return TEXT("DT_");

            case ERRResourceDataType::UE_BODY_SETUP:
                return TEXT("BS_");

            default:
                return EMPTY_STR;
        }
    }

    /**
     * @brief Get the Assets Base Path object.
     * For particular handling, please set the asset path ending with '/'
     * This concatenation operator ensure only a single '/' is put in between
     * @param InModuleName
     * @return FString #ASSETS_ROOT_PATH / InModuleName
     */
    FORCEINLINE static FString GetAssetsBasePath(const TCHAR* InModuleName)
    {
        return FString(ASSETS_ROOT_PATH) / InModuleName;
    }

    /**
     * @brief Get the leaf relative UE path of a folder housing assets of a particular resource data type.
     * NOTE: This function returns a user-configurable folder path value, thus cannot be made static or constexpr
     * @param InDataType
     * @return FString
     */
    FORCEINLINE FString GetAssetsFolderName(const ERRResourceDataType InDataType)
    {
        switch (InDataType)
        {
            case ERRResourceDataType::UE_PAK:
                return FOLDER_PATH_ASSET_PAKS;

            case ERRResourceDataType::UE_STATIC_MESH:
                return FOLDER_PATH_ASSET_STATIC_MESHES;

            case ERRResourceDataType::UE_SKELETAL_MESH:
                return FOLDER_PATH_ASSET_SKELETAL_MESHES;

            case ERRResourceDataType::UE_SKELETON:
                return FOLDER_PATH_ASSET_SKELETONS;

            case ERRResourceDataType::UE_PHYSICS_ASSET:
                return FOLDER_PATH_PHYSICS_ASSETS;

            case ERRResourceDataType::UE_MATERIAL:
            case ERRResourceDataType::UE_PHYSICAL_MATERIAL:
                return FOLDER_PATH_ASSET_MATERIALS;

            case ERRResourceDataType::UE_TEXTURE:
                return FOLDER_PATH_ASSET_TEXTURES;

            case ERRResourceDataType::UE_DATA_TABLE:
                return FOLDER_PATH_ASSET_DATA_TABLES;

            default:
                return EMPTY_STR;
        }
    }

    static constexpr const TCHAR* DYNAMIC_CONTENTS_FOLDER_NAME = TEXT("DynamicContents");

    /**
     * @brief Get the dynamic Assets Base Path of a module
     * @param InModuleName
     * @return FString #GetAssetsBasePath(InModuleName) / #DYNAMIC_CONTENTS_FOLDER_NAME
     */
    FORCEINLINE static FString GetDynamicAssetsBasePath(const TCHAR* InModuleName)
    {
        return GetAssetsBasePath(InModuleName) / DYNAMIC_CONTENTS_FOLDER_NAME;
    }

    /**
     * @brief Get the full UE path of a dynamic asset that is either early loaded at Sim initilization or generated dynamically at runtime
     * @param InDataType
     * @param InAssetName
     * @param InModuleName
     * @return FString
     */
    FORCEINLINE FString GetDynamicAssetPath(const ERRResourceDataType InDataType,
                                            const FString& InAssetName,
                                            const TCHAR* InModuleName)
    {
        return GetDynamicAssetsBasePath(InModuleName) / GetAssetsFolderName(InDataType) / InAssetName;
    }

    /**
     * @brief Get the Dynamic Assets Path List object
     * Return the TArray of #SASSET_OWNING_MODULE_NAMES[i] / #DYNAMIC_CONTENTS_FOLDER_NAME
     * @note #SASSET_OWNING_MODULE_NAMES must have been initialized earlier & once only at Sim start by [ConfigureSimInPlay()]
     * @param InDataType
     * @return TArray<FString>
     */
    FORCEINLINE static TArray<FString> GetDynamicAssetsBasePathList(const ERRResourceDataType InDataType)
    {
        static TArray<FString> dynamicAssetsBasePathList = [InDataType]()
        {
            TArray<FString> basePaths;
            for (const auto& moduleName : SASSET_OWNING_MODULE_NAMES[InDataType])
            {
                basePaths.Emplace(GetDynamicAssetsBasePath(moduleName));
            }
            return basePaths;
        }();
        return dynamicAssetsBasePathList;
    }

    /**
     * @brief Collate assets info for #ResourceMap
     * @tparam InDataType
     * @param InAssetRelativeFolderPath
     * @return Num of assets
     */
    template<ERRResourceDataType InDataType>
    int32 CollateAssetsInfo()
    {
        return CollateAssetsInfo<URRAssetObject<InDataType>>(InDataType, GetAssetsFolderName(InDataType));
    }

    /**
     * @brief Collate assets info for #ResourceMap
     * @tparam TResource
     * @param InDataType
     * @param InAssetRelativeFolderPath
     * @return Num of assets
     */
    template<typename T>
    int32 CollateAssetsInfo(const ERRResourceDataType InDataType, const FString& InAssetRelativeFolderPath)
    {
        FRRResourceInfo& outResourceInfo = GetSimResourceInfo(InDataType);

        TArray<FAssetData> totalAssetDataList;
        for (const auto& assetsBasePath : GetDynamicAssetsBasePathList(InDataType))
        {
            TArray<FAssetData> assetDataList;
            URRAssetUtils::LoadAssetDataList<T>(assetsBasePath / InAssetRelativeFolderPath, assetDataList);
            totalAssetDataList.Append(assetDataList);
        }

        for (const auto& asset : totalAssetDataList)
        {
#if RAPYUTA_SIM_DEBUG
            UE_LOG_WITH_INFO_SHORT(LogTemp,
                                   Warning,
                                   TEXT("[%s] ASSET [%s] [%s] [%s]"),
                                   *asset.AssetName.ToString(),
                                   *asset.PackagePath.ToString(),
                                   *asset.GetFullName(),
                                   *asset.ToSoftObjectPath().ToString());
#endif
            outResourceInfo.AddResource(asset.AssetName.ToString(), asset.ToSoftObjectPath().ToString(), nullptr);
        }
        return totalAssetDataList.Num();
    }

    /**
     * @brief Collate assets info
     */
    void CollateAssetsInfo()
    {
        CollateAssetsInfo<ERRResourceDataType::UE_STATIC_MESH>();
        CollateAssetsInfo<ERRResourceDataType::UE_SKELETAL_MESH>();
        CollateAssetsInfo<ERRResourceDataType::UE_SKELETON>();
        CollateAssetsInfo<ERRResourceDataType::UE_PHYSICS_ASSET>();
        CollateAssetsInfo<ERRResourceDataType::UE_MATERIAL>();
        CollateAssetsInfo<ERRResourceDataType::UE_PHYSICAL_MATERIAL>();
        CollateAssetsInfo<ERRResourceDataType::UE_TEXTURE>();
    }

    //  RESOURCE STORE --
    //
    /**
     * @brief Check all resources have been loaded or not.
     *
     * @param bIsLogged
     * @return true
     * @return false
     */
    bool HaveAllResourcesBeenLoaded(bool bIsLogged = false) const;

    /**
     * @brief Collate asset resources info & Async load them by UAssetManager
     * @tparam InDataType
     * @return true
     * @return false
     * @sa [UAssetManager](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UAssetManager/)
     */
    template<ERRResourceDataType InDataType>
    bool RequestResourcesLoading()
    {
        FRRResourceInfo& resourceInfo = GetSimResourceInfo(InDataType);
        if (resourceInfo.bHasBeenAllLoaded)
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("All resources have been loaded. No need to request their loading again."));
            return true;
        }

        // 1- COLLATE ALL ASSETS INFO
        if (0 == CollateAssetsInfo<URRAssetObject<InDataType>>(InDataType, GetAssetsFolderName(InDataType)))
        {
            resourceInfo.bHasBeenAllLoaded = true;
            UE_LOG_WITH_INFO(
                LogTemp, Warning, TEXT("THERE IS NO [%s] TO BE LOADED"), *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType));
            return true;
        }

        // 2- REQUEST FOR LOADING THE RESOURCES ASYNCHRONOUSLY
        resourceInfo.ToBeAsyncLoadedResourceNum = resourceInfo.Data.Num();
        resourceInfo.bHasBeenAllLoaded = false;
#if RAPYUTA_SIM_VERBOSE
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Warning,
                         TEXT("[%s] TO BE LOADED NUM: %d"),
                         *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                         resourceInfo.ToBeAsyncLoadedResourceNum);
#endif

        UAssetManager* assetManager = UAssetManager::GetIfInitialized();
        if (assetManager)
        {
            for (const auto& resourceMetaData : resourceInfo.Data)
            {
                // https://docs.unrealengine.com/en-US/Resources/SampleGames/ARPG/BalancingBlueprintAndCPP/index.html
                // "Avoid Referencing Assets by String"
                FSoftObjectPath resourceSoftObjPath(resourceMetaData.Value.GetAssetPath());
                assetManager->GetStreamableManager().RequestAsyncLoad(
                    resourceSoftObjPath,
                    FStreamableDelegate::CreateUObject(this,
                                                       &URRGameSingleton::OnResourceLoaded,
                                                       InDataType,
                                                       resourceSoftObjPath,
                                                       resourceMetaData.Value.UniqueName));
            }
            return true;
        }
        else
        {
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("UNABLE TO GET ASSET MANAGER!"))
            return false;
        }
    }

    //! This is used as param to [FStreamableDelegate::CreateUObject()] thus its params could not be constref-ized
    FORCEINLINE void OnResourceLoaded(ERRResourceDataType InDataType, FSoftObjectPath InResourcePath, FString InResourceUniqueName)
    {
        check(IsInGameThread());

        switch (InDataType)
        {
            case ERRResourceDataType::UE_STATIC_MESH:
                ProcessAsyncLoadedResource<UStaticMesh>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_SKELETAL_MESH:
                ProcessAsyncLoadedResource<USkeletalMesh>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_SKELETON:
                ProcessAsyncLoadedResource<USkeleton>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_PHYSICS_ASSET:
                ProcessAsyncLoadedResource<UPhysicsAsset>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_MATERIAL:
                ProcessAsyncLoadedResource<UMaterialInterface>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_PHYSICAL_MATERIAL:
                ProcessAsyncLoadedResource<UPhysicalMaterial>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_TEXTURE:
                ProcessAsyncLoadedResource<UTexture>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            case ERRResourceDataType::UE_DATA_TABLE:
                ProcessAsyncLoadedResource<UDataTable>(InDataType, InResourcePath, InResourceUniqueName);
                break;

            default:
                break;
        }
    }

    /**
     * @brief
     * Callback to handle an asynchronously loaded resource, which receives a [FSoftObjectPath] that references a valid [UObject]
     * Then put it into both #ResourceMap & #ResourceStore.
     *
     * @tparam TResource
     * @param InDataType
     * @param InResourcePath
     * @param InResourceUniqueName
     * @return bool
     * @sa [FSoftObjectPath](https://docs.unrealengine.com/4.27/en-US/API/Runtime/CoreUObject/UObject/FSoftObjectPath/)
     * @sa [Asynchronous Asset Loading](https://docs.unrealengine.com/5.1/en-US/asynchronous-asset-loading-in-unreal-engine/)
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
            UE_LOG_WITH_INFO(LogTemp,
                             Warning,
                             TEXT("%d [%s] [%s:%s] RESOURCE LOADED %u"),
                             resourceInfo.ToBeAsyncLoadedResourceNum,
                             *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                             *InResourceUniqueName,
                             *InResourcePath.ToString(),
                             resource);
#endif
            if (resourceInfo.ToBeAsyncLoadedResourceNum == 0)
            {
                resourceInfo.bHasBeenAllLoaded = true;
            }

            // Resource Data --
            // Still need to store resource handle in a direct UPROPERTY() child TArray of this GameSingleton to bypass
            // early GC
            ResourceStore.AddUnique(Cast<UObject>(resource));
            return true;
        }
        return false;
    }

    /**
     * @brief Update #ResourceMap and #ResourceStore with dynamically runtime-generated InResourceObject of which soft object path is also created on the fly.
     *
     * @tparam TResource
     * @param InDataType
     * @param InResourceObject
     * @param InResourceUniqueName
     * @return
     */
    template<typename TResource>
    FORCEINLINE void AddDynamicResource(const ERRResourceDataType InDataType,
                                        TResource* InResourceObject,
                                        const FString& InResourceUniqueName,
                                        const FString& InResourceAssetPath = EMPTY_STR)
    {
        // Update [ResourceMap] with dynamically runtime-generated [InResourceObject]
        // of which soft object path is also created on the fly.
        FRRResourceInfo& resourceInfo = GetSimResourceInfo(InDataType);
        // (Note) FSoftObjectPath only accepts legit package names, not [InResourceUniqueName] like an arbitrary one
        resourceInfo.AddResource(InResourceUniqueName,
                                 InResourceAssetPath.IsEmpty() ? FSoftObjectPath(InResourceObject) : InResourceAssetPath,
                                 InResourceObject);
        resourceInfo.bHasBeenAllLoaded = true;

#if RAPYUTA_SIM_DEBUG
        UE_LOG_WITH_INFO(LogTemp,
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

    /**
     * @brief Get the Sim Resource object, loading if required + updating resource store
     *
     * @tparam TResource
     * @param InDataType
     * @param InResourceUniqueName
     * @param bIsStaticResource
     * @return TResource*
     */
    template<typename TResource>
    TResource* GetSimResource(const ERRResourceDataType InDataType,
                              const FString& InResourceUniqueName,
                              bool bIsStaticResource = true)
    {
        if (false == ResourceMap.Contains(InDataType))
        {
            UE_LOG_WITH_INFO_SHORT(LogTemp,
                                   Error,
                                   TEXT("It seems [ResourceMap][%s] has not yet been fully initialized! Make sure GameMode class "
                                        "is child of RRGameMode."),
                                   *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType));
            return nullptr;
        }
        FRRResource resourceInfo = GetSimResourceInfo(InDataType).Data.FindRef(InResourceUniqueName);
        const FString resourceAssetPath = resourceInfo.GetAssetPath();
        auto& resourceAssetData = resourceInfo.AssetData;

        bool bNewlyLoaded = false;
        // NOTE: Null [resourceAssetData] either means it is a not-yet-created dynamic resource Or a not-yet-loaded-from-disk static resource (uasset)
        if (false == IsValid(resourceAssetData))
        {
            if (IsInGameThread())
            {
                // NOTE: Empty [resourceAssetPath] should only mean a dynamic resource. Otherwise, the static resource asset path must not have been properly collated!
                resourceAssetData = resourceAssetPath.IsEmpty()
                                        ? nullptr
                                        : URRAssetUtils::LoadObjFromAssetPath<TResource>((UObject*)this, resourceAssetPath);
                if (resourceAssetData)
                {
                    bNewlyLoaded = true;
                }
                else
                {
                    // NOTE: For dynamically-created resources, 1st time querying (due to empty [resourceAssetPath]) always yields null [resourceAssetData] due to no [resourceAssetPath] determined yet,
                    // in which case no error log should be printed
                    if (bIsStaticResource)
                    {
#if WITH_EDITOR
                        UE_LOG_WITH_INFO_SHORT(LogTemp,
                                               Error,
                                               TEXT("[%s] [Unique Name: %s] INVALID STATIC RESOURCE PATH [%s]!"),
                                               *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                                               *InResourceUniqueName,
                                               *resourceAssetPath);
#else
                        UE_LOG_WITH_INFO_SHORT(LogTemp,
                                               Error,
                                               TEXT("[%s] [Unique Name: %s] INVALID STATIC RESOURCE PATH [%s]!\n"
                                                    "Please consider adding its containing folder asset path (Eg: "
                                                    "'/RapyutaSimulationPlugins/DynamicContents') to "
                                                    "[DirectoriesToAlwaysCook] in DefaultGame.ini"),
                                               *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                                               *InResourceUniqueName,
                                               *resourceAssetPath);
#endif
                    }
                    return nullptr;
                }
            }
            else
            {
                UE_LOG_WITH_INFO_SHORT(LogTemp,
                                       Error,
                                       TEXT("[%s] [Unique Name: %s] RESOURCE SHOULD HAVE BEEN LOADED EARLIER IN GAMETHREAD [%s]!"),
                                       *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                                       *InResourceUniqueName,
                                       *resourceAssetPath);
                return nullptr;
            }
        }

        // Verify resource asset's in-memory validity
        TResource* resourceAsset = Cast<TResource>(resourceAssetData);
        if (resourceAsset)
        {
            if (resourceAsset->IsValidLowLevelFast())
            {
                // Add into resource store if newly loaded above
                if (bNewlyLoaded)
                {
                    AddDynamicResource<TResource>(InDataType, resourceAsset, InResourceUniqueName, resourceAssetPath);
                }
            }
            else
            {
                UE_LOG_WITH_INFO_SHORT(LogTemp,
                                       Error,
                                       TEXT("[%s] [Unique Name: %s] INVALID-AT-LOW-LEVEL RESOURCE %u!"),
                                       *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
                                       *InResourceUniqueName,
                                       resourceAsset)
                return nullptr;
            }
        }
        return resourceAsset;
    }

    /**
     * @brief Check resource exist with #GetSimResourceInfo
     *
     * @param InDataType
     * @param InResourceUniqueName
     * @return true
     * @return false
     */
    bool HasSimResource(const ERRResourceDataType InDataType, const FString& InResourceUniqueName) const
    {
        return GetSimResourceInfo(InDataType).Data.Contains(InResourceUniqueName);
    }

    /**
     * @brief Get the Sim Resource List object with #GetSimResourceInfo
     *
     * @param InDataType
     * @return const TMap<FString, FRRResource>&
     */
    const TMap<FString, FRRResource>& GetSimResourceList(const ERRResourceDataType InDataType) const
    {
        verifyf(ResourceMap.Contains(InDataType),
                TEXT("It seems [ResourceMap][%s] not yet fully initialized!"),
                *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType));
        return GetSimResourceInfo(InDataType).Data;
    }

    /**
     * @brief Get the Sim Resource Info object from #ResourceMap
     *
     * @param InDataType
     * @return FRRResourceInfo&
     */
    FRRResourceInfo& GetSimResourceInfo(const ERRResourceDataType InDataType)
    {
        verifyf(ResourceMap.Contains(InDataType),
                TEXT("It seems [ResourceMap][%s] has not yet been fully initialized!"),
                *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType));
        return ResourceMap[InDataType];
    }

    /**
     * @brief Get the Sim Resource Info object from #ResourceMap
     *
     * @param InDataType
     * @return const FRRResourceInfo&
     */
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
        return InMeshName.Equals(SHAPE_NAME_PLANE)      ? ERRShapeType::PLANE
               : InMeshName.Equals(SHAPE_NAME_CUBE)     ? ERRShapeType::BOX
               : InMeshName.Equals(SHAPE_NAME_CYLINDER) ? ERRShapeType::CYLINDER
               : InMeshName.Equals(SHAPE_NAME_SPHERE)   ? ERRShapeType::SPHERE
               : InMeshName.Equals(SHAPE_NAME_CAPSULE)  ? ERRShapeType::CAPSULE
                                                        : ERRShapeType::MESH;
    }

    /**
     * @brief Get or load static mesh, #GetSimResource with #ERRResourceDataType::UE_STATIC_MESH
     * (NOTE) StaticMesh could be dynamically created
     *
     * @param InStaticMeshName
     * @param bIsStaticResource
     * @return FORCEINLINE*
     */
    FORCEINLINE UStaticMesh* GetStaticMesh(const FString& InStaticMeshName, bool bIsStaticResource = true)
    {
        return GetSimResource<UStaticMesh>(ERRResourceDataType::UE_STATIC_MESH, InStaticMeshName, bIsStaticResource);
    }

    // SKELETAL ASSETS --
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_SKELETAL_MESHES = TEXT("SkeletalMeshes");
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_SKELETONS = TEXT("Skeletons");
    UPROPERTY(config)
    FString FOLDER_PATH_PHYSICS_ASSETS = TEXT("PhysicsAssets");
    /**
     * @brief Get or load skeletal mesh, #GetSimResource with #ERRResourceDataType::UE_SKELETAL_MESH
     *
     * @param InSkeletalMeshName
     * @param bIsStaticResource
     * @return FORCEINLINE*
     */
    FORCEINLINE USkeletalMesh* GetSkeletalMesh(const FString& InSkeletalMeshName, bool bIsStaticResource = true)
    {
        return GetSimResource<USkeletalMesh>(ERRResourceDataType::UE_SKELETAL_MESH, InSkeletalMeshName, bIsStaticResource);
    }

    /**
     * @brief Get or load physics skeleton, #GetSimResource with #ERRResourceDataType::UE_SKELETON
     *
     * @param InSkeletonName
     * @param bIsStaticResource
     * @return FORCEINLINE*
     */
    FORCEINLINE USkeleton* GetSkeleton(const FString& InSkeletonName, bool bIsStaticResource = true)
    {
        return GetSimResource<USkeleton>(ERRResourceDataType::UE_SKELETON, InSkeletonName, bIsStaticResource);
    }

    /**
     * @brief Get or load physics asset, #GetSimResource with #ERRResourceDataType::UE_PHYSICS_ASSET
     *
     * @param InPhysicsAssetName
     * @param bIsStaticResource
     * @return FORCEINLINE*
     */
    FORCEINLINE UPhysicsAsset* GetPhysicsAsset(const FString& InPhysicsAssetName, bool bIsStaticResource = true)
    {
        return GetSimResource<UPhysicsAsset>(ERRResourceDataType::UE_PHYSICS_ASSET, InPhysicsAssetName, bIsStaticResource);
    }

    // MATERIALS --
    //! Dynamic Material assets folder path
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_MATERIALS = TEXT("Materials");

    // Here we only define specially used materials for some specific purpose!
    static constexpr const TCHAR* MATERIAL_NAME_ASSET_MASTER = TEXT("M_RapyutaAssetMaster");
    static constexpr const TCHAR* MATERIAL_NAME_PROP_MASTER = TEXT("M_RapyutaPropMaster");
    static constexpr const TCHAR* MATERIAL_NAME_PHYSICS_WHEEL = TEXT("PM_Wheel");
    static constexpr const TCHAR* MATERIAL_NAME_TRANSLUCENCE_MASTER = TEXT("M_RapyutaTranslucenceMaster");

    /**
     * @brief Get or load material, calling #GetSimResource with #ERRResourceDataType::UE_MATERIAL
     *
     * @param InMaterialName
     * @return FORCEINLINE*
     */
    FORCEINLINE UMaterialInterface* GetMaterial(const FString& InMaterialName)
    {
        return GetSimResource<UMaterialInterface>(ERRResourceDataType::UE_MATERIAL, InMaterialName);
    }

    /**
     * @brief Get or load physical material, calling #GetSimResource with #ERRResourceDataType::UE_PHYSICAL_MATERIAL
     *
     * @param InPhysicalMaterialName
     * @return FORCEINLINE*
     */
    FORCEINLINE UPhysicalMaterial* GetPhysicalMaterial(const FString& InPhysicalMaterialName)
    {
        return GetSimResource<UPhysicalMaterial>(ERRResourceDataType::UE_PHYSICAL_MATERIAL, InPhysicalMaterialName);
    }

    // TEXTURES --
    //! Dynamic Texture assets folder path
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_TEXTURES = TEXT("Textures");

    static constexpr const TCHAR* TEXTURE_NAME_WHITE_MASK = TEXT("T_WhiteMask");
    static constexpr const TCHAR* TEXTURE_NAME_BLACK_MASK = TEXT("T_BlackMask");

    /**
     * @brief Get or load texture, calling #GetSimResource with #ERRResourceDataType::UE_TEXTURE
     * @param InTextureName
     * @param bIsStaticResource
     * @return FORCEINLINE*
     */
    FORCEINLINE UTexture* GetTexture(const FString& InTextureName, bool bIsStaticResource = true)
    {
        return GetSimResource<UTexture>(ERRResourceDataType::UE_TEXTURE, InTextureName, bIsStaticResource);
    }

    // DATA TABLES --
    //! Dynamic Data Table assets folder path
    UPROPERTY(config)
    FString FOLDER_PATH_ASSET_DATA_TABLES = TEXT("DataTables");

    /**
     * @brief Get or load Data table, calling #GetSimResource with #ERRResourceDataType::UE_DATA_TABLE
     * @param InDataTableName
     * @param bIsStaticResource
     * @return FORCEINLINE*
     */
    FORCEINLINE UDataTable* GetDataTable(const FString& InDataTableName, bool bIsStaticResource = true)
    {
        return GetSimResource<UDataTable>(ERRResourceDataType::UE_DATA_TABLE, InDataTableName, bIsStaticResource);
    }

    // BODY SETUPS --
    /**
     * @brief Get or load BodySetup, calling #GetSimResource with #ERRResourceDataType::UE_BODY_SETUP
     * @param InBodySetupName
     * @return FORCEINLINE*
     */
    FORCEINLINE UBodySetup* GetBodySetup(const FString& InBodySetupName)
    {
        // Body setups are dynamically created, thus not static resources
        return GetSimResource<UBodySetup>(ERRResourceDataType::UE_BODY_SETUP, InBodySetupName, false);
    }

private:
    //! Async loaded, thus must be thread safe. A map just helps referencing an item faster, though costs some overheads.
    //! Besides, UE does not support UPROPERTY() on a map yet.
    TMap<ERRResourceDataType, FRRResourceInfo> ResourceMap;

    //! We need this to escape UObject-based resource Garbage Collection
    UPROPERTY()
    TArray<TObjectPtr<UObject>> ResourceStore;
};
