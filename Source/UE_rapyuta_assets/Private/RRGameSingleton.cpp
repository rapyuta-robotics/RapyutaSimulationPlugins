#include "RRGameSingleton.h"

#include "Tools/RRTypeUtils.h"

TMap<ERRResourceDataType, const TCHAR*> URRGameSingleton::SASSET_OWNING_MODULE_NAMES = {
    {ERRResourceDataType::UE_STATIC_MESH, UE_RAPYUTA_ASSETS_MODULE_NAME},
    {ERRResourceDataType::UE_MATERIAL, UE_RAPYUTA_ASSETS_MODULE_NAME},
    {ERRResourceDataType::UE_TEXTURE, UE_RAPYUTA_ASSETS_MODULE_NAME}};

URRGameSingleton::URRGameSingleton(){UE_LOG(LogTemp, Display, TEXT("[RR GAME SINGLETON] INSTANTIATED! ======================"))}

URRGameSingleton::~URRGameSingleton()
{
    // AssetDataList.Empty();
}

URRGameSingleton* URRGameSingleton::Get()
{
    URRGameSingleton* singleton = Cast<URRGameSingleton>(GEngine->GameSingleton);
    if (!singleton)
    {
        UE_LOG(LogTemp, Fatal, TEXT("[URRGameSingleton] NOT YET SET AS GAME SINGLETON!!"))
    }

    if (IsInGameThread())
    {
        bool isValidInMemory = IsValid(singleton);
        if (isValidInMemory)
        {
            if (!singleton->HasAnyFlags(EObjectFlags::RF_Standalone))
            {
                UE_LOG(LogTemp, Warning, TEXT("[URRGameSingleton] RF_Standalone!!"));
                singleton->SetFlags(EObjectFlags::RF_Standalone);
                // If needed, also hook it to Root set
            }
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[URRGameSingleton] Not valid in memory!!"))
        }
    }

    return singleton;
}

bool URRGameSingleton::InitializeResources()
{
    // Initialize resource meta info
    for (int8 i = (static_cast<int8>(ERRResourceDataType::NONE) + 1); i < static_cast<int8>(ERRResourceDataType::TOTAL); ++i)
    {
        const ERRResourceDataType dataType = static_cast<ERRResourceDataType>(i);
        ResourceMap.Add(dataType, FRRResourceInfo(dataType));
    }

    // READ ALL SIM DYNAMIC RESOURCES INFO FROM DESGINATED [~CONTENT] FOLDERS
    // [STATIC MESH]
    ERRResourceDataType dataType = ERRResourceDataType::UE_STATIC_MESH;
    FRRResourceInfo resourceInfo(dataType);

    // Fetch built-in assets meta data to [resourceInfo]
    bool result = CollateAssetsMetaData<UStaticMesh>(
        GetDynamicAssetsPath(ERRResourceDataType::UE_STATIC_MESH) / CFOLDER_PATH_SIM_ASSET_STATIC_MESHES, resourceInfo);
    if (!result)
    {
        return false;
    }

    // Request Runtime resource async loading
    GetSimResourceInfo(dataType).HasBeenAllLoaded = false;
    if (false == RequestResourcesLoading(dataType, resourceInfo))
    {
        UE_LOG(LogTemp, Error, TEXT("[%s] READ MESH META DATA FAILED"), *URRTypeUtils::GetERRResourceDataTypeAsString(dataType));
        return false;
    }

    UE_LOG(LogTemp, Display, TEXT("[InitializeResources] => RESOURCES REGISTERED TO BE LOADED!"));
    return true;
}

bool URRGameSingleton::RequestResourcesLoading(const ERRResourceDataType InDataType, const FRRResourceInfo& InResourceInfo)
{
    if (GetSimResourceInfo(InDataType).HasBeenAllLoaded || (0 == InResourceInfo.Data.Num()))
    {
        return false;
    }

    // REQUEST FOR LOADING THE RESOURCES ASYNCHRONOUSLY
    GetSimResourceInfo(InDataType).ToBeAsyncLoadedResourceNum = InResourceInfo.Data.Num();
    UE_LOG(LogTemp,
           Display,
           TEXT("[%s] TO BE LOADED NUM: %d"),
           *URRTypeUtils::GetERRResourceDataTypeAsString(InDataType),
           InResourceInfo.Data.Num());
    UAssetManager* assetManager = UAssetManager::GetIfValid();
    if (assetManager)
    {
        for (const auto& resourceMeta : InResourceInfo.Data)
        {
            // https://docs.unrealengine.com/en-US/Resources/SampleGames/ARPG/BalancingBlueprintAndCPP/index.html
            // "Avoid Referencing Assets by String"
            FSoftObjectPath resourceSoftObjPath(resourceMeta.Value.GetAssetPath());
            assetManager->GetStreamableManager().RequestAsyncLoad(
                resourceSoftObjPath,
                FStreamableDelegate::CreateUObject(
                    this, &URRGameSingleton::OnResourceLoaded, InDataType, resourceSoftObjPath, resourceMeta.Value.UniqueName));
        }
        return true;
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[InitializeResources] UNABLE TO GET ASSET MANAGER!"))
        return false;
    }
}

void URRGameSingleton::FinalizeResources()
{
    // Sim Resources (Mesh shapes, materials, textures, curves, and their metadata, etc.)
    SASSET_OWNING_MODULE_NAMES.Empty();

    for (int8 i = static_cast<int8>(ERRResourceDataType::NONE) + 1; i < static_cast<int8>(ERRResourceDataType::TOTAL); ++i)
    {
        ResourceMap[static_cast<ERRResourceDataType>(i)].Finalize();
    }

    ResourceStore.Empty();
}

bool URRGameSingleton::HaveAllResourcesBeenLoaded(bool bIsLogged)
{
    bool bResult = true;
    for (const auto& resourceInfo : ResourceMap)
    {
        bResult &= resourceInfo.Value.HasBeenAllLoaded;
    }

    if (!bResult && bIsLogged)
    {
        UE_LOG(LogTemp, Error, TEXT("Resources are not yet fully loaded!"));
    }
    return bResult;
}

// void URRGameSingleton::FetchAllAssetsDataList()
//{
//    URRAssetUtils::FetchAssetDataListFromRegistry<UObject>(
//        GetDynamicAssetsPath(static_cast<ERRResourceDataType>(dataType)), AssetDataList, true, false);
//}
