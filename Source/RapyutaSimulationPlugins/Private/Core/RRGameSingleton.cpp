// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRGameSingleton.h"

// RapyutaSim
#include "Core/RRTypeUtils.h"

TMap<ERRResourceDataType, TArray<const TCHAR*>> URRGameSingleton::SASSET_OWNING_MODULE_NAMES = {
    // Only required for statically loaded UASSET
    {ERRResourceDataType::UE_STATIC_MESH, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_SKELETAL_MESH, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_SKELETON, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_PHYSICS_ASSET, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_MATERIAL, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_TEXTURE, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_DATA_TABLE, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
};

URRGameSingleton::URRGameSingleton()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("INSTANTIATED! ======================"));
#endif
}

URRGameSingleton::~URRGameSingleton()
{
    // AssetDataList.Empty();
}

void URRGameSingleton::PrintSimConfig() const
{
    UE_LOG(LogRapyutaCore, Display, TEXT("RRGameSingleton Configs:"));
    UE_LOG(LogRapyutaCore, Display, TEXT("- SIM PROFILING: %d"), BSIM_PROFILING);
    UE_LOG(LogRapyutaCore, Display, TEXT("- ASSETS_RUNTIME_BP_SAVE_BASE_PATH: %s"), *ASSETS_RUNTIME_BP_SAVE_BASE_PATH);
    UE_LOG(LogRapyutaCore, Display, TEXT("- FOLDER_PATH_ASSET_STATIC_MESHES: %s"), *FOLDER_PATH_ASSET_STATIC_MESHES);
    UE_LOG(LogRapyutaCore, Display, TEXT("- FOLDER_PATH_ASSET_SKELETAL_MESHES: %s"), *FOLDER_PATH_ASSET_SKELETAL_MESHES);
    UE_LOG(LogRapyutaCore, Display, TEXT("- FOLDER_PATH_ASSET_SKELETONS: %s"), *FOLDER_PATH_ASSET_SKELETONS);
    UE_LOG(LogRapyutaCore, Display, TEXT("- FOLDER_PATH_PHYSICS_ASSETS: %s"), *FOLDER_PATH_PHYSICS_ASSETS);
    UE_LOG(LogRapyutaCore, Display, TEXT("- FOLDER_PATH_ASSET_MATERIALS: %s"), *FOLDER_PATH_ASSET_MATERIALS);
    UE_LOG(LogRapyutaCore, Display, TEXT("- FOLDER_PATH_ASSET_TEXTURES: %s"), *FOLDER_PATH_ASSET_TEXTURES);
    UE_LOG(LogRapyutaCore, Display, TEXT("- FOLDER_PATH_ASSET_DATA_TABLES: %s"), *FOLDER_PATH_ASSET_DATA_TABLES);
    for (auto i = static_cast<int8>(ERRResourceDataType::NONE); i < static_cast<int8>(ERRResourceDataType::TOTAL); ++i)
    {
        const ERRResourceDataType dataType = static_cast<ERRResourceDataType>(i);
        UE_LOG(LogRapyutaCore,
               Display,
               TEXT("[%s]'s dynamic-assets base paths:"),
               *URRTypeUtils::GetERRResourceDataTypeAsString(dataType));
        for (const auto& basePath : GetDynamicAssetsBasePathList(dataType))
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("%s"), *basePath);
        }
    }
}

URRGameSingleton* URRGameSingleton::Get()
{
    URRGameSingleton* singleton = Cast<URRGameSingleton>(GEngine->GameSingleton);
    if (!singleton)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("NOT YET SET AS GAME SINGLETON!!"));
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
    // Prepare an empty [ResourceMap]
    for (uint8 i = (static_cast<uint8>(ERRResourceDataType::NONE) + 1); i < static_cast<uint8>(ERRResourceDataType::TOTAL); ++i)
    {
        const ERRResourceDataType dataType = static_cast<ERRResourceDataType>(i);
        ResourceMap.Add(dataType, FRRResourceInfo(dataType));
    }

    bool bResult = true;
    if (bInRequestResourceLoading)
    {
        // READ ALL SIM DYNAMIC RESOURCES (UASSETS) INFO FROM DESGINATED [~CONTENT] FOLDERS
        // & REGISTER THEM TO BE ASYNC LOADED INTO [ResourceMap]
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
        // NOTE: NO dynamic resources required -> considered to have been all loaded
        for (auto& [_, resourceInfo] : ResourceMap)
        {
            resourceInfo.bHasBeenAllLoaded = true;
        }
    }

    return bResult;
}

void URRGameSingleton::FinalizeResources()
{
    for (uint8 i = (static_cast<uint8>(ERRResourceDataType::NONE) + 1); i < static_cast<uint8>(ERRResourceDataType::TOTAL); ++i)
    {
        ResourceMap[static_cast<ERRResourceDataType>(i)].Finalize();
    }

    ResourceStore.Empty();
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
