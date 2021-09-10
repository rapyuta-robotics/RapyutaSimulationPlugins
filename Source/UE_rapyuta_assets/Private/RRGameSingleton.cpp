#include "RRGameSingleton.h"

#include "Tools/RRUObjectUtils.h"

TMap<ERRResourceDataType, const TCHAR*> URRGameSingleton::SASSET_OWNING_MODULE_NAMES;

URRGameSingleton::URRGameSingleton(){UE_LOG(LogTemp, Display, TEXT("[RR GAME SINGLETON] INSTANTIATED! ======================"))}

URRGameSingleton::~URRGameSingleton()
{
    AssetDataList.Empty();
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
        bool isValidInMemory = URRUObjectUtils::IsEntityValidInMemory(singleton);
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

bool URRGameSingleton::InitializeRuntimeResources()
{
    // Initialize resource meta info
    for (int8 i = (static_cast<int8>(ERRResourceDataType::NONE) + 1); i < static_cast<int8>(ERRResourceDataType::TOTAL); ++i)
    {
        ResourceMap.Add(static_cast<ERRResourceDataType>(i), FRRResource(dataType));
    }

    // READ ALL SIM RESOURCE INFO FROM DESGINATED [~CONTENT] FOLDERS
    // [STATIC MESH]
    ERRResourceDataType dataType = ERRResourceDataType::STATIC_MESH;
    FRRResourceInfo resourceInfo(dataType);

    // Fetch built-in assets meta data to [resourceInfo]
    bool result = CollateAssetsMetaData<UStaticMesh>(SASSET_HOUSING_MODULE_NAMES[dataType] / CFOLDER_PATH_SIM_ASSET_STATIC_MESHES,
                                                     resourceInfo);
    if (!result)
    {
        return false;
    }

    // Request Runtime resource async loading
    GetSimResourceInfo(dataType).HasBeenAllLoaded = false;
    if (!RequestResourcesLoading(dataType, resourceInfo))
    {
        UE_LOG(LogTemp,
               Error,
               TEXT("[%s] READ MESH META DATA FAILED"),
               *URRTypeUtils::GetERRResourceDataTypeAsString(dataType, resourceInfo));
        return false;
    }

    UE_LOG(LogTemp, Display, TEXT("[InitializeRuntimeResources] => RESOURCES REGISTERED TO BE LOADED!"));
    return true;
}

void URRGameSingleton::FinalizeRuntimeResources()
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

void URRGameSingleton::FetchAllAssetsDataList()
{
    URRAssetUtils::FetchAssetDataListFromRegistry<UObject>(GetSimStartupContentPath(), AssetDataList, true, false);
}
