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
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("- SIM PROFILING: %d"), BSIM_PROFILING);
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

bool URRGameSingleton::InitializeResources()
{
    // Prepare an empty [ResourceMap]
    for (uint8 i = (static_cast<uint8>(ERRResourceDataType::NONE) + 1); i < static_cast<uint8>(ERRResourceDataType::TOTAL); ++i)
    {
        const ERRResourceDataType dataType = static_cast<ERRResourceDataType>(i);
        ResourceMap.Add(dataType, FRRResourceInfo(dataType));
    }

    // READ ALL SIM DYNAMIC RESOURCES (UASSETS) INFO FROM DESGINATED [~CONTENT] FOLDERS
    // & REGISTER THEM TO BE ASYNC LOADED INTO [ResourceMap]
    // [STATIC MESH] --
    RequestResourcesLoading<ERRResourceDataType::UE_STATIC_MESH>();

    // [SKELETAL MESH] --
    RequestResourcesLoading<ERRResourceDataType::UE_SKELETAL_MESH>();

    // [SKELETON] --
    RequestResourcesLoading<ERRResourceDataType::UE_SKELETON>();

    // [PHYSICS ASSET] --
    RequestResourcesLoading<ERRResourceDataType::UE_PHYSICS_ASSET>();

    // [MATERIAL] --
    RequestResourcesLoading<ERRResourceDataType::UE_MATERIAL>();

    // [TEXTURE] --
    RequestResourcesLoading<ERRResourceDataType::UE_TEXTURE>();

    // [BODY SETUP] --
    // Body setups are dynamically created in runtime only
    GetSimResourceInfo(ERRResourceDataType::UE_BODY_SETUP).bHasBeenAllLoaded = true;

#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("RESOURCES REGISTERED TO BE LOADED!"));
#endif
    return true;
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
    bool bResult = true;
    for (const auto& resourceInfo : ResourceMap)
    {
        bResult &= resourceInfo.Value.bHasBeenAllLoaded;
        if (!bResult && bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Warning,
                             TEXT("[%s] Resources have not yet been fully loaded!"),
                             *URRTypeUtils::GetERRResourceDataTypeAsString(resourceInfo.Key));
        }
    }

    return bResult;
}
