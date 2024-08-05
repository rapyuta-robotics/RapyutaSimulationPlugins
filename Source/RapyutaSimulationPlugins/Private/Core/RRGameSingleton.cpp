// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Core/RRGameSingleton.h"

// RapyutaSim
#include "Core/RRPakLoader.h"
#include "Core/RRTypeUtils.h"

TMap<ERRResourceDataType, TArray<const TCHAR*>> URRGameSingleton::SASSET_OWNING_MODULE_NAMES = {
    // Only required for statically loaded UASSET
    {ERRResourceDataType::UE_PAK, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_STATIC_MESH, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_SKELETAL_MESH, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_SKELETON, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_PHYSICS_ASSET, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_MATERIAL, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_PHYSICAL_MATERIAL,
     {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_TEXTURE, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_DATA_TABLE, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
};

URRGameSingleton::URRGameSingleton()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("INSTANTIATED! ======================"));
#endif
    // Prepare an empty [ResourceMap]
    for (uint8 i = (static_cast<uint8>(ERRResourceDataType::NONE) + 1); i < static_cast<uint8>(ERRResourceDataType::TOTAL); ++i)
    {
        const ERRResourceDataType dataType = static_cast<ERRResourceDataType>(i);
        ResourceMap.Add(dataType, FRRResourceInfo(dataType));
    }
}

URRGameSingleton::~URRGameSingleton()
{
    // AssetDataList.Empty();
}

void URRGameSingleton::PrintSimConfig() const
{
    UE_LOG(LogRapyutaCore, Log, TEXT("RRGameSingleton Configs:"));
    UE_LOG(LogRapyutaCore, Log, TEXT("- SIM PROFILING: %d"), BSIM_PROFILING);
    UE_LOG(LogRapyutaCore, Log, TEXT("- ASSETS_RUNTIME_BP_SAVE_BASE_PATH: %s"), *ASSETS_RUNTIME_BP_SAVE_BASE_PATH);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_ASSET_PAKS: %s"), *FOLDER_PATH_ASSET_PAKS);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_ASSET_STATIC_MESHES: %s"), *FOLDER_PATH_ASSET_STATIC_MESHES);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_ASSET_SKELETAL_MESHES: %s"), *FOLDER_PATH_ASSET_SKELETAL_MESHES);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_ASSET_SKELETONS: %s"), *FOLDER_PATH_ASSET_SKELETONS);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_PHYSICS_ASSETS: %s"), *FOLDER_PATH_PHYSICS_ASSETS);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_ASSET_MATERIALS: %s"), *FOLDER_PATH_ASSET_MATERIALS);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_ASSET_TEXTURES: %s"), *FOLDER_PATH_ASSET_TEXTURES);
    UE_LOG(LogRapyutaCore, Log, TEXT("- FOLDER_PATH_ASSET_DATA_TABLES: %s"), *FOLDER_PATH_ASSET_DATA_TABLES);
#if RAPYUTA_SIM_VERBOSE
    for (auto i = static_cast<int8>(ERRResourceDataType::NONE) + 1; i < static_cast<int8>(ERRResourceDataType::TOTAL); ++i)
    {
        const ERRResourceDataType dataType = static_cast<ERRResourceDataType>(i);
        UE_LOG(LogRapyutaCore,
               Log,
               TEXT("[%s]'s dynamic-assets base paths:"),
               *URRTypeUtils::GetERRResourceDataTypeAsString(dataType));
        for (const auto& basePath : GetDynamicAssetsBasePathList(dataType))
        {
            UE_LOG(LogRapyutaCore, Log, TEXT("%s"), *basePath);
        }
    }
#endif
}

URRGameSingleton* URRGameSingleton::Get()
{
    URRGameSingleton* singleton = Cast<URRGameSingleton>(GEngine->GameSingleton);
    if (!singleton)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("[GEngine->GameSingletonClassName] IS NOT SET AS [URRGameSingleton]"));
        return nullptr;
    }

    if (IsInGameThread())
    {
        if (IsValid(singleton))
        {
            if (!singleton->HasAnyFlags(EObjectFlags::RF_Standalone))
            {
                UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("RF_Standalone!!"));
                singleton->SetFlags(EObjectFlags::RF_Standalone);
                // If needed, also hook it to Root set
            }
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Not valid or pending kill!!"));
        }
    }

    return singleton;
}

bool URRGameSingleton::InitializeResources(bool bInRequestResourceLoading)
{
    // Initialize PakLoader (only available in packaged Sim)
#if WITH_EDITOR
    bPakLoaderInitialized = false;
#else
    PakLoader = URRUObjectUtils::CreateSelfSubobject<URRPakLoader>(this, TEXT("RRPakLoader"));
    // Already logged here-in
    bPakLoaderInitialized = PakLoader->Initialize();
#endif

    // Start request resource loading if required
    bool bResult = true;
    if (bInRequestResourceLoading)
    {
        // READ ALL SIM DYNAMIC RESOURCES (UASSETS) INFO FROM DESGINATED [~CONTENT] FOLDERS
        // & REGISTER THEM TO BE ASYNC LOADED INTO [ResourceMap]
        // [PAK] --
        if (bPakLoaderInitialized)
        {
            // Load pak files
            for (const auto& paksBasePath : GetDynamicAssetsBasePathList(ERRResourceDataType::UE_PAK))
            {
                FString paksBaseFolderPath;
                if (FPackageName::TryConvertLongPackageNameToFilename(paksBasePath, paksBaseFolderPath))
                {
                    const FString paksFolderPath = paksBaseFolderPath / GetAssetsFolderName(ERRResourceDataType::UE_PAK);
                    if (FPaths::DirectoryExists(paksFolderPath))
                    {
                        PakLoader->LoadPAKFiles(paksFolderPath);
                    }
                }
                else
                {
                    UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("Failed converting [%s] to local disk path"), *paksBasePath);
                }
            }
        }
        GetSimResourceInfo(ERRResourceDataType::UE_PAK).bHasBeenAllLoaded = true;

        // [STATIC MESH] --
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_STATIC_MESH>();

        // [SKELETAL MESH] --
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_SKELETAL_MESH>();

        // [SKELETON] --
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_SKELETON>();

        // [PHYSICS ASSET] --
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_PHYSICS_ASSET>();

        // [MATERIAL] --
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_MATERIAL>();
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_PHYSICAL_MATERIAL>();

        // [TEXTURE] --
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_TEXTURE>();

        // [DATATABLE] --
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_DATA_TABLE>();

        // [BODY SETUP] --
        // Body setups are dynamically created in runtime only
        GetSimResourceInfo(ERRResourceDataType::UE_BODY_SETUP).bHasBeenAllLoaded = true;

#if RAPYUTA_SIM_VERBOSE
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("RESOURCES REGISTERED TO BE LOADED!"));
#endif
    }
    else
    {
        // NOTE: NO initial resources loading required
        // -> Considered to have been all loaded
        for (auto& [dataType, resourceInfo] : ResourceMap)
        {
            resourceInfo.bHasBeenAllLoaded = (dataType != ERRResourceDataType::UE_DATA_TABLE);
            // -> LoadObject<>() is expected to be called on [resourceInfo.GetAssetPath()] on-the-fly during Sim run
            // -> Refer to [GetSimResource()]
        }
        // Except [UE_DATA_TABLE], which should be always loaded initially
        bResult &= RequestResourcesLoading<ERRResourceDataType::UE_DATA_TABLE>();

        // -> Only collate assets' metadata without loading
        CollateAssetsInfo();

        // NOTE: Due to on-the-fly resource loading is only possible if running in Game-thread
        // -> Preload global statically defined resources, in case they are referenced in non-game thread
        // -> Could only load after collating assets info above
        // StaticMeshes
        GetStaticMesh(SHAPE_NAME_PLANE);
        GetStaticMesh(SHAPE_NAME_CUBE);
        GetStaticMesh(SHAPE_NAME_CYLINDER);
        GetStaticMesh(SHAPE_NAME_SPHERE);
        GetStaticMesh(SHAPE_NAME_CAPSULE);

        // Materials
        GetMaterial(MATERIAL_NAME_PROP_MASTER);
        GetPhysicalMaterial(MATERIAL_NAME_PHYSICS_WHEEL);

        // Textures
        GetTexture(TEXTURE_NAME_WHITE_MASK);
        GetTexture(TEXTURE_NAME_BLACK_MASK);
    }

    return bResult;
}

bool URRGameSingleton::CollateEntityAssetsInfoFromPAK(const TArray<FString>& InEntityModelNameList, bool bInForceReload)
{
    if (bPakLoaderInitialized)
    {
        // Load pak files of [InEntityModelNameList]
        for (const auto& paksBasePath : GetDynamicAssetsBasePathList(ERRResourceDataType::UE_PAK))
        {
            FString paksBaseFolderPath;
            if (FPackageName::TryConvertLongPackageNameToFilename(paksBasePath, paksBaseFolderPath))
            {
                const FString paksFolderPath = paksBaseFolderPath / GetAssetsFolderName(ERRResourceDataType::UE_PAK);
                if (FPaths::DirectoryExists(paksFolderPath) &&
                    PakLoader->LoadEntitiesPAKFiles(paksFolderPath, InEntityModelNameList, bInForceReload))
                {
                    // Collate assets info after mounting PAK files, so they can be loaded on-the-fly
                    CollateAssetsInfo();
                    return true;
                }
            }
            else
            {
                UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("Failed converting [%s] to local disk path"), *paksBasePath);
            }
        }
        return true;
    }
    else
    {
        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("PakLoader has not been initialized"));
        return false;
    }
}

void URRGameSingleton::FinalizeResources()
{
#if !WITH_EDITOR
    for (uint8 i = (static_cast<uint8>(ERRResourceDataType::NONE) + 1); i < static_cast<uint8>(ERRResourceDataType::TOTAL); ++i)
    {
        ResourceMap[static_cast<ERRResourceDataType>(i)].Finalize();
    }

    ResourceStore.Empty();
#endif
    // ELSE: All resources must be kept otherwise the next PIE could not load them by LoadObject<>
}

bool URRGameSingleton::HaveAllResourcesBeenLoaded(bool bIsLogged) const
{
    for (const auto& resourceInfo : ResourceMap)
    {
        if (false == resourceInfo.Value.bHasBeenAllLoaded)
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogRapyutaCore,
                                 Warning,
                                 TEXT("[%s] Resources have not yet been fully loaded!"),
                                 *URRTypeUtils::GetERRResourceDataTypeAsString(resourceInfo.Key));
            }
            return false;
        }
    }
    return true;
}
