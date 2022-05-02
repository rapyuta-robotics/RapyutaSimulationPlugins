// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRGameSingleton.h"

// UE
#include "Engine/AssetManager.h"
#include "Engine/Engine.h"

// RapyutaSim
#include "Core/RRTypeUtils.h"

TMap<ERRResourceDataType, TArray<const TCHAR*>> URRGameSingleton::SASSET_OWNING_MODULE_NAMES = {
    // Only required for statically loaded UASSET
    {ERRResourceDataType::UE_STATIC_MESH, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_SKELETAL_MESH, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_SKELETON, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_PHYSICS_ASSET, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
    {ERRResourceDataType::UE_MATERIAL, {URRGameSingleton::ASSETS_PROJECT_MODULE_NAME, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME}},
};

URRGameSingleton::URRGameSingleton(){
    UE_LOG(LogRapyutaCore, Display, TEXT("[RR GAME SINGLETON] INSTANTIATED! ======================"))}

URRGameSingleton::~URRGameSingleton()
{
    // AssetDataList.Empty();
}

URRGameSingleton* URRGameSingleton::Get()
{
    URRGameSingleton* singleton = Cast<URRGameSingleton>(GEngine->GameSingleton);
    if (!singleton)
    {
        UE_LOG(LogRapyutaCore, Fatal, TEXT("[URRGameSingleton] NOT YET SET AS GAME SINGLETON!!"));
        return nullptr;
    }

    if (IsInGameThread())
    {
        if (IsValid(singleton))
        {
            if (!singleton->HasAnyFlags(EObjectFlags::RF_Standalone))
            {
                UE_LOG(LogRapyutaCore, Warning, TEXT("[URRGameSingleton] RF_Standalone!!"));
                singleton->SetFlags(EObjectFlags::RF_Standalone);
                // If needed, also hook it to Root set
            }
        }
        else
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("[URRGameSingleton] Not valid or pending kill!!"));
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
    ERRResourceDataType dataType = ERRResourceDataType::UE_STATIC_MESH;
    CollateAssetsInfo<UStaticMesh>(dataType, FOLDER_PATH_ASSET_STATIC_MESHES);

    // Request StaticMesh resource async loading
    GetSimResourceInfo(dataType).HasBeenAllLoaded = false;
    verify(RequestResourcesLoading(dataType));

    // (NOTE) RuntimeMesh & SkeletalMesh-related resources are only created in runtime for now
    // (though they could be also loaded from Content)

    // [SKELETAL MESH] --
    GetSimResourceInfo(ERRResourceDataType::UE_SKELETAL_MESH).HasBeenAllLoaded = true;

    // [SKELETON] --
    GetSimResourceInfo(ERRResourceDataType::UE_SKELETON).HasBeenAllLoaded = true;

    // [PHYSICS ASSET] --
    GetSimResourceInfo(ERRResourceDataType::UE_PHYSICS_ASSET).HasBeenAllLoaded = true;
    // [MATERIAL] --
    dataType = ERRResourceDataType::UE_MATERIAL;
    CollateAssetsInfo<UMaterialInterface>(dataType, FOLDER_PATH_ASSET_MATERIALS);

    // Request Material resource async loading
    GetSimResourceInfo(dataType).HasBeenAllLoaded = false;
    verify(RequestResourcesLoading(dataType));

    // [TEXTURE] --
    // Textures are only externally loaded for now
    GetSimResourceInfo(ERRResourceDataType::UE_TEXTURE).HasBeenAllLoaded = true;

    // [BODY SETUP] --
    // Body setups are dynamically created in runtime
    GetSimResourceInfo(ERRResourceDataType::UE_BODY_SETUP).HasBeenAllLoaded = true;

    UE_LOG(LogRapyutaCore, Display, TEXT("[InitializeResources] => RESOURCES REGISTERED TO BE LOADED!"));
    return true;
}

bool URRGameSingleton::RequestResourcesLoading(const ERRResourceDataType InDataType)
{
    const FRRResourceInfo& resourceInfo = GetSimResourceInfo(InDataType);
    const int32 resourceNum = resourceInfo.Data.Num();
    if (resourceInfo.HasBeenAllLoaded)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("All resources have been loaded. No need to request the resources loading again."));
        return true;
    }
    else if (0 == resourceNum)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("THERE ARE NO [%] TO BE LOADED."),
               *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType));
        return true;
    }

    // REQUEST FOR LOADING THE RESOURCES ASYNCHRONOUSLY
    GetSimResourceInfo(InDataType).ToBeAsyncLoadedResourceNum = resourceNum;
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("[%s] TO BE LOADED NUM: %d"),
           *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
           resourceNum);

    UAssetManager* assetManager = UAssetManager::GetIfValid();
    if (assetManager)
    {
        for (const auto& resourceMetaData : resourceInfo.Data)
        {
            // https://docs.unrealengine.com/en-US/Resources/SampleGames/ARPG/BalancingBlueprintAndCPP/index.html
            // "Avoid Referencing Assets by String"
            FSoftObjectPath resourceSoftObjPath(resourceMetaData.Value.GetAssetPath());
            assetManager->GetStreamableManager().RequestAsyncLoad(
                resourceSoftObjPath,
                FStreamableDelegate::CreateUObject(
                    this, &URRGameSingleton::OnResourceLoaded, InDataType, resourceSoftObjPath, resourceMetaData.Value.UniqueName));
        }
        return true;
    }
    else
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("[InitializeResources] UNABLE TO GET ASSET MANAGER!"))
        return false;
    }
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
        bResult &= resourceInfo.Value.HasBeenAllLoaded;
        if (!bResult && bIsLogged)
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s] Resources have not yet been fully loaded!"),
                   *URRTypeUtils::GetERRResourceDataTypeAsString(resourceInfo.Key));
        }
    }

    return bResult;
}
