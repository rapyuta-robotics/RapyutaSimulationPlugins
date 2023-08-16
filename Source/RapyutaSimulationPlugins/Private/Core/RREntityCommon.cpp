// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RREntityCommon.h"

// UE
#include "AssetRegistry/AssetRegistryModule.h"
#include "DrawDebugHelpers.h"

#if WITH_EDITOR
#include "UnrealEd.h"
#endif

// RapyutaSimulationPlugins
#include "Core/RRAssetUtils.h"
#include "Core/RRCoreUtils.h"
#include "Core/RREntityStructs.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshActor.h"
#include "Core/RRSDFParser.h"
#include "Core/RRTypeUtils.h"
#include "Core/RRUObjectUtils.h"
#include "Core/RRURDFParser.h"
#include "Robots/RRBaseRobot.h"

// [URREntityCommon] --
//
TMap<FString, TSharedPtr<FRREntityModelInfo, ESPMode::ThreadSafe>> URREntityCommon::SEntityModelsInfo;
TArray<FString> URREntityCommon::EntityModelsFolderPathList;

void URREntityCommon::PrintSimConfig() const
{
    UE_LOG(LogRapyutaCore, Log, TEXT("ENTITY COMMON CONFIG -----------------------------"));
    UE_LOG(LogRapyutaCore, Log, TEXT("EXTERNAL_ENTITY_MODELS_FOLDER_PATHS:"));
    for (const auto& modelsPath : EXTERNAL_ENTITY_MODELS_FOLDER_PATHS)
    {
        UE_LOG(LogRapyutaCore, Log, TEXT("- %s"), *modelsPath);
    }
}

bool URREntityCommon::IsEntityModelRegistered(const FString& InEntityModelName)
{
    return SEntityModelsInfo.Contains(InEntityModelName);
}

void URREntityCommon::RegisterEntityModel(FRREntityModelInfo&& InModelInfo, bool bInOverwriteExisting)
{
    // [InModelInfo] will be [MoveTemp] thus its [ModelName] must be saved beforehand
    FString modelName = InModelInfo.Data.GetModelName();
    if (IsEntityModelRegistered(modelName))
    {
        if (bInOverwriteExisting)
        {
            UE_LOG_WITH_INFO_SHORT(
                LogRapyutaCore,
                Warning,
                TEXT("!NOTE - OVERWRITING IN-MEMORY ENTITY MODEL [%s] - DescType [%s]"),
                *modelName,
                *URRTypeUtils::GetEnumValueAsString(
                    TEXT("ERREntityDescriptionType"), InModelInfo.Data.ModelDescType, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME));
        }
        else
        {
            return;
        }
    }

    // Add new one to [SEntityModelsInfo]
    SEntityModelsInfo.Add(MoveTemp(modelName), MakeShared<FRREntityModelInfo, ESPMode::ThreadSafe>(MoveTemp(InModelInfo)));
}

void URREntityCommon::UnregisterEntityModel(const FString& InEntityModelName)
{
    SEntityModelsInfo.Remove(InEntityModelName);
}

TSharedRef<FRREntityModelInfo, ESPMode::ThreadSafe> URREntityCommon::GetEntityModelInfo(const FString& InEntityModelName)
{
    return SEntityModelsInfo.FindRef(InEntityModelName).ToSharedRef();
}

bool URREntityCommon::HasEntityDescriptionPathBeenLoaded(const FString& InDescriptionFilePath)
{
    for (const auto& entityModelInfo : SEntityModelsInfo)
    {
        if (entityModelInfo.Value->GetDescriptionFilePath() == InDescriptionFilePath)
        {
            return true;
        }
    }
    return false;
}

void URREntityCommon::RegisterEntityModelsFromCADPaths(const TArray<FString>& InCADPathList,
                                                       TArray<FString>* OutEntityModelNameList,
                                                       bool bInOverwriteExisting)
{
    for (const auto& cadPath : InCADPathList)
    {
        // Only FBX is supported for now
        if ((false == cadPath.EndsWith(URRCoreUtils::GetSimFileExt(ERRFileType::CAD_FBX))) ||
            (false == FPaths::FileExists(cadPath)))
        {
            continue;
        }

        // NOTE: [newModelInfo]'s detailed data will be populated later after [CAD -> StaticMesh/SkeletalMesh] process
        const FString newModelName = FPaths::GetBaseFilename(cadPath);
        FRREntityModelInfo newModelInfo = FRREntityModelInfo(FRREntityModelData({newModelName}));
        newModelInfo.Data.ModelDescType = ERREntityDescriptionType::SINGLE_CAD;
        newModelInfo.Data.DescriptionFilePath = FPaths::ConvertRelativePathToFull(cadPath);

        if (ensure(newModelInfo.IsValid(true)))
        {
            // Always fill in [OutEntityModelNameList]
            if (OutEntityModelNameList)
            {
                OutEntityModelNameList->AddUnique(newModelName);
            }

            // Already account for anti-overwriting over existing model data here-in (logged here-in)
            RegisterEntityModel(MoveTemp(newModelInfo), bInOverwriteExisting);
        }
    }
}

void URREntityCommon::ConfigureEntityModelsFolderPathList(URREntityCommon* InEntityCommon, const TCHAR* InPluginModuleFolderName)
{
    const FString modelsBaseFolderPath = FPaths::ConvertRelativePathToFull(
        InPluginModuleFolderName ? (FPaths::ProjectPluginsDir() / InPluginModuleFolderName) : FPaths::ProjectDir());

    for (const auto& userModelsPath : InEntityCommon->EXTERNAL_ENTITY_MODELS_FOLDER_PATHS)
    {
        const bool bUserPathAbsolute = !FPaths::IsRelative(userModelsPath);
        FString modelsAbsolutePath = bUserPathAbsolute ? userModelsPath : (modelsBaseFolderPath / userModelsPath);

        // Only append if the folder exists on disk
        if (FPaths::DirectoryExists(modelsAbsolutePath))
        {
            URREntityCommon::EntityModelsFolderPathList.AddUnique(MoveTemp(modelsAbsolutePath));
        }
        else if (bUserPathAbsolute)
        {
            UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("[%s] does not exist"), *userModelsPath);
        }
        // NOTE: For relative users-provided model paths, by default Project + all plugins are added as the base paths, which do not always have to be pre-existent, thus no error logs here,
        // which will be printed during models loading later on.
    }
}

bool URREntityCommon::ModelsDirHasEntityModel(const FString& InEntityModelsDirPath, const FString& InEntityModelName)
{
    if (FPaths::DirectoryExists(InEntityModelsDirPath))
    {
        TArray<FString> modelDescFilePathList;
        // Already logged here-in
        if (URRCoreUtils::LoadFullFilePaths(InEntityModelsDirPath, modelDescFilePathList, {ERRFileType::URDF, ERRFileType::SDF}))
        {
            return modelDescFilePathList.ContainsByPredicate(
                [InEntityModelName](const FString& InDescFilePath)
                { return (FPaths::GetBaseFilename(InDescFilePath) == InEntityModelName); });
        }
    }
    return false;
}

FVector URREntityCommon::GetEntityModelSize(ARRMeshActor* InEntity)
{
    return GetEntityModelSize(InEntity->EntityModelName);
}

FRRMaterialProperty URREntityCommon::GetEntityModelMaterialInfo(ARRMeshActor* InEntity)
{
    return GetEntityModelMaterialInfo(InEntity->EntityModelName);
}

bool URREntityCommon::LoadAllEntityModelsInfo(const TArray<ERRFileType>& InDescFileTypes,
                                              bool bInForceReload,
                                              TArray<FString>* OutEntityModelNameList)
{
    if (!ensure(EntityModelsFolderPathList.Num() > 0))
    {
        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("[EntityModelsFolderPathList] is empty"));
        return false;
    }
    bool bValidModels = true;

    // Model Parser -> newModelInfo
    TUniquePtr<FRREntityDescriptionParser> sdfParser = MakeUnique<FRRSDFParser>();
    TUniquePtr<FRREntityDescriptionParser> urdfParser = MakeUnique<FRRURDFParser>();
    for (const auto& modelsFolderPath : EntityModelsFolderPathList)
    {
        // 1- [URDF/SDF or CAD MODELS] INFO --
        if ((bValidModels = FPaths::DirectoryExists(modelsFolderPath)))
        {
            // 1.1- Load all urdf/sdf files recursively
            TArray<FString> modelDescFilePathList;
            // Already logged here-in
            if ((bValidModels = URRCoreUtils::LoadFullFilePaths(modelsFolderPath, modelDescFilePathList, InDescFileTypes)))
            {
                // Read [URDF/SDF Models] info -> [SEntityModelsInfo]
                // EACH [URDF/SDF Model Info] -> represents ONE [URDF/SDF Model]
#if RAPYUTA_SIM_VERBOSE
                UE_LOG_WITH_INFO_SHORT(
                    LogRapyutaCore, Warning, TEXT("[URDF/SDF] TO BE LOADED NUM: %d!"), modelDescFilePathList.Num());
#endif
                for (const auto& modelDescFilePath : modelDescFilePathList)
                {
                    // NOTE: Entity Model Name could only be read after reading the whole model description file
                    // (The model file name itself could not be used as the model name due to being unreliable)
                    // -> Thus cannot use [IsEntityModelRegistered(modelName)] here
                    // Skip ones (urdf/sdf paths) that have been already loaded
                    if ((!bInForceReload) && HasEntityDescriptionPathBeenLoaded(modelDescFilePath))
                    {
                        continue;
                    }

                    // Load & register new model from [modelDescFilePath]
                    auto& modelParser =
                        modelDescFilePath.EndsWith(URRCoreUtils::GetSimFileExt(ERRFileType::URDF)) ? urdfParser : sdfParser;
                    FRREntityModelInfo newEntityModelInfo = modelParser->LoadModelInfoFromFile(modelDescFilePath);
#if RAPYUTA_SIM_VERBOSE
                    newEntityModelInfo.PrintSelf();
#endif
                    if (ensure(newEntityModelInfo.IsValid(true)))
                    {
                        // Always fill in [OutEntityModelNameList]
                        if (OutEntityModelNameList)
                        {
                            OutEntityModelNameList->AddUnique(newEntityModelInfo.Data.GetModelName());
                        }

                        // Check for anti-overwriting over existing model data here-in
                        RegisterEntityModel(MoveTemp(newEntityModelInfo), bInForceReload);
                    }
                }
            }
            else
            {
                // 1.2- Load all CAD files recursively (ONLY IF THERE IS NO URDF/SDF)
                // NOTE: Only [FBX] is supported for now
                TArray<FString> cadPathList;
                // Already logged here-in
                if ((bValidModels = URRCoreUtils::LoadFullFilePaths(modelsFolderPath, cadPathList, {ERRFileType::CAD_FBX})))
                {
                    RegisterEntityModelsFromCADPaths(cadPathList, OutEntityModelNameList, bInForceReload);
                }
            }
        }
        else
        {
            UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("[%s] Entity models Info Folder does NOT exist"), *modelsFolderPath);
        }
    }

#if WITH_EDITOR
    // 2- [DATA-TABLE BP-ENTITY MODELS] INFO --
    // 2.1- Update/Create [BPModelsDataTable] from [SEntityModelsInfo] (with possibly some newly registered above)
    UpdateBlueprintEntityModelsDataTable(bInForceReload, OutEntityModelNameList);
#endif
    return OutEntityModelNameList ? (OutEntityModelNameList->Num() > 0) : bValidModels;
}

bool URREntityCommon::LoadEntityModelsInfo(const TArray<FString>& InEntityModelsNameList,
                                           const TArray<ERRFileType>& InDescFileTypes,
                                           bool bInForceReload)
{
    // [TODO]: ONLY RELOAD ENTITY MODEL IF EITHER NOT YET REIGISTERED || UPDATE-RESPONSE FROM QUERY TO WEB-SERVER
    if (!ensure(EntityModelsFolderPathList.Num() > 0))
    {
        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("[EntityModelsFolderPathList] is empty"));
        return false;
    }

    bool bResult = true;
    TUniquePtr<FRREntityDescriptionParser> sdfParser = MakeUnique<FRRSDFParser>();
    TUniquePtr<FRREntityDescriptionParser> urdfParser = MakeUnique<FRRURDFParser>();

    // 1- SEARCH IN NATIVE [URDF/SDF/WORLD/YAML or CAD] INFO FOR [InEntityModelsNameList] --
    for (const auto& entityModelName : InEntityModelsNameList)
    {
        // 1.0.1- Skip if [entityModelName] has been already registered, except [bInForceReload]
        if ((!bInForceReload) && URREntityCommon::IsEntityModelRegistered(entityModelName))
        {
            continue;
        }

        // 1.0.2- Skip BP-based model
        if (IsBlueprintEntityModelName(entityModelName))
        {
            continue;
        }

        // 1.1- Find the corresponding native desc file (URDF/SDF/WORLD/YAML) of [entityModelName]
        bool bEntityModelLoaded = false;
        for (const auto& modelsFolderPath : EntityModelsFolderPathList)
        {
            bool bEntityNativeDescFileFound = false;
            if (FPaths::DirectoryExists(modelsFolderPath))
            {
                TArray<FString> modelDescFilePathList;
                // Already logged here-in
                if (URRCoreUtils::LoadFullFilePaths(modelsFolderPath, modelDescFilePathList, InDescFileTypes))
                {
                    FString modelDescFilePath;
                    bEntityNativeDescFileFound = modelDescFilePathList.ContainsByPredicate(
                        [&entityModelName, &modelDescFilePath](const FString& InDescFilePath)
                        {
                            if (FPaths::GetBaseFilename(InDescFilePath) == entityModelName)
                            {
                                modelDescFilePath = InDescFilePath;
                                return true;
                            }
                            else
                            {
                                return false;
                            }
                        });

                    if (bEntityNativeDescFileFound)
                    {
                        // 1.2- Load & register new model from [modelDescFilePath]
                        const auto& modelParser =
                            modelDescFilePath.EndsWith(URRCoreUtils::GetSimFileExt(ERRFileType::URDF)) ? urdfParser : sdfParser;

                        // Entity Model Name could only be read after reading the whole model description file
                        // (The model file name itself could not be used as the model name due to being unreliable)
                        // Check for anti-overwriting over existing model data here-in
                        FRREntityModelInfo entityModelInfo = modelParser->LoadModelInfoFromFile(modelDescFilePath);
                        bEntityModelLoaded = entityModelInfo.IsValid(true);    // Logged here-in already
                        if (bEntityModelLoaded)
                        {
                            RegisterEntityModel(MoveTemp(entityModelInfo), bInForceReload);
                        }

                        // Model found (regardless of validity) -> Break out of [EntityModelsFolderPathList] loop
                        // -> Proceed to next [entityModelName]
                        break;
                    }
                }
                // Else: continue to next [modelsFolderPath]
            }
            else
            {
                UE_LOG_WITH_INFO_SHORT(
                    LogRapyutaCore, Error, TEXT("[%s] Entity models Info Folder does NOT exist"), *modelsFolderPath);
                // NOTE: Not return here to proceed to keep searching in [BP-ENTITY MODELS DATA TABLE] below
            }

            // 1.3- Find the corresponding CAD file of [entityModelName] (ONLY IF there is NO URDF/SDF found above)
            if (false == bEntityNativeDescFileFound)
            {
                // NOTE: Only [FBX] is supported for now
                TArray<FString> cadPathList;
                if (URRCoreUtils::LoadFullFilePaths(
                        modelsFolderPath, cadPathList, {ERRFileType::CAD_FBX}))    // already logged here-in
                {
                    FString cadPath;
                    bEntityNativeDescFileFound = cadPathList.ContainsByPredicate(
                        [&entityModelName, &cadPath](const FString& InCADPath)
                        {
                            if (FPaths::GetBaseFilename(InCADPath) == entityModelName)
                            {
                                cadPath = InCADPath;
                                return true;
                            }
                            else
                            {
                                return false;
                            }
                        });

                    // Only register CAD-based models upon [bEntityNativeDescFileFound] for at least an entity model
                    if (bEntityNativeDescFileFound)
                    {
                        bEntityModelLoaded = true;
                        RegisterEntityModelsFromCADPaths({cadPath}, nullptr, bInForceReload);
                    }
                }
                // Else: continue to next [modelsFolderPath]
            }

#if WITH_EDITOR
            // 1.4- Update/Create [BPModelsDataTable] from [SEntityModelsInfo] (with possibly some newly registered above)
            UpdateBlueprintEntityModelsDataTable(bInForceReload);
#endif
        }    // End loop of [EntityModelsFolderPathList]

        if (false == bEntityModelLoaded)
        {
            bResult = false;
            UE_LOG_WITH_INFO_SHORT(
                LogRapyutaCore,
                Error,
                TEXT("[%s] Entity model either invalid or having no corresponding description files "
                     "(%s or CAD) to be loaded"),
                *entityModelName,
                *FString::JoinBy(InDescFileTypes,
                                 TEXT(","),
                                 [](const ERRFileType& InFileType) { return URRCoreUtils::GetSimFileExt(InFileType); }));
        }
    }    // End loop of [URDF/SDF/WORLD/YAML or CAD] search for [InEntityModelsNameList]

    // 2- SEARCH IN [BP-ENTITY MODELS DATA TABLE] FOR [InEntityModelsNameList] --
#if WITH_EDITOR
    // 2.1- NOTE: First, always update/create [BPModelsDataTable] from current [SEntityModelsInfo] if non-existing
    UpdateBlueprintEntityModelsDataTable(bInForceReload);
#endif

    // 2.2- Find Or Add new entry of [bpModelsDataTable] based on [InEntityModelsNameList] (ONLY IF IT IS BP-based)
    UDataTable* bpModelsDataTable = URRGameSingleton::Get()->GetDataTable(GetBPEntityModelsDataTableName(), false);
    if (bpModelsDataTable)
    {
        UE_LOG_WITH_INFO_SHORT(
            LogRapyutaCore, Log, TEXT("Found BP-Entity models data table at [%s]"), *GetBPEntityModelsDataTableFilePath());
    }

    for (const auto& entityModelName : InEntityModelsNameList)
    {
        if (IsBlueprintEntityModelName(entityModelName))
        {
            TUniquePtr<FRREntityModelInfo> bpEntityModelInfoPtr = nullptr;
            if (bpModelsDataTable)
            {
                const FName bpEntityModelName = FName(*entityModelName);
                if (auto* modelRow = bpModelsDataTable->FindRow<FRREntityModelTableRow>(bpEntityModelName, EMPTY_STR, false))
                {
                    // 2.3- Save existing BP-[modelRow->Data] to [bpEntityModelInfoPtr]
                    bpEntityModelInfoPtr = MakeUnique<FRREntityModelInfo>(modelRow->Data);
                }
                else
                {
                    // 2.4- Create [newBPEntityModelData] from [entityModelName]
                    // -> Add new entry to [bpModelsDataTable]
                    FRREntityModelData newBPEntityModelData =
                        ComposeBlueprintEntityModelData(FRREntityModelData({entityModelName}));
                    bpModelsDataTable->AddRow(bpEntityModelName, FRREntityModelTableRow(newBPEntityModelData));

                    // 2.5- Save [newBPEntityModelData] to [bpEntityModelInfoPtr]
                    bpEntityModelInfoPtr = MakeUnique<FRREntityModelInfo>(MoveTemp(newBPEntityModelData));
                    UE_LOG_WITH_INFO_SHORT(
                        LogRapyutaCore, Warning, TEXT("[DATA TABLE] NEW BP-ENTITY MODEL REGISTERED: %s!"), *entityModelName);
                }
            }
            else
            {
                // 2.6- Save [newBPEntityModelData] to [bpEntityModelInfoPtr]
                bpEntityModelInfoPtr =
                    MakeUnique<FRREntityModelInfo>(ComposeBlueprintEntityModelData(FRREntityModelData({entityModelName})));
            }

            // 2.7- Register [bpEntityModelInfoPtr] to [SEntityModelsInfo]
            if (bpEntityModelInfoPtr)
            {
                RegisterEntityModel(MoveTemp(*bpEntityModelInfoPtr), bInForceReload);
            }
        }
    }
    return bResult;
}

bool URREntityCommon::LoadEntityModelsAllData(const TArray<FString>& InEntityModelsNameList,
                                              const TArray<ERRFileType>& InDescFileTypes,
                                              bool bInSaveToAsset,
                                              bool bInForceReload)
{
#if !WITH_EDITOR
    // PAKLoader is accessible in Engine packaged build only
    if (false == URRGameSingleton::Get()->CollateEntityAssetsInfoFromPAK(InEntityModelsNameList))
    {
        return false;
    }
#endif

    return
        // 1- Entity models info first
        LoadEntityModelsInfo(InEntityModelsNameList, InDescFileTypes, bInForceReload) &&
        //2- Entity manufacturing textures later due to querying [SEntityModelsInfo]
        LoadEntityModelsManufacturingTextures(InEntityModelsNameList, bInSaveToAsset, bInForceReload);
}

FRREntityModelData URREntityCommon::ComposeBlueprintEntityModelData(const FRREntityModelData& InEntityModelData)
{
    // Create [newBPEntityModelData] from [InEntityModelData]
    FRREntityModelData newBPEntityModelData = InEntityModelData;
    newBPEntityModelData.ModelNameList = {GetBlueprintEntityModelName(InEntityModelData.GetModelName())};
    newBPEntityModelData.ModelDescType = ERREntityDescriptionType::UE_DATA_TABLE;
    newBPEntityModelData.DescriptionFilePath = GetBPEntityModelsDataTableFilePath();
    return newBPEntityModelData;
}

UDataTable* URREntityCommon::UpdateBlueprintEntityModelsDataTable(bool bInOverwriteExisting,
                                                                  TArray<FString>* OutTotalEntityModelNameList)
{
#if WITH_EDITOR
    // 1- Always create [bpModelsDataTable] if not yet existing, so any ensuing BP-entity first creation could register its model info used by its instance initializing
    auto* gameSingleton = URRGameSingleton::Get();
    const FString bpModelsDataTableName = GetBPEntityModelsDataTableName();
    UDataTable* bpModelsDataTable = gameSingleton->GetDataTable(bpModelsDataTableName, false);
    const bool bNewCreation = (nullptr == bpModelsDataTable);
    if (bNewCreation)
    {
        // 1.1 - Compose [bpModelsDataTable] of [FRREntityModelTableRow] format
        bpModelsDataTable = NewObject<UDataTable>(gameSingleton, FName(*bpModelsDataTableName));
        bpModelsDataTable->RowStruct = FRREntityModelTableRow::StaticStruct();

        // 1.2- Add [bpModelsDataTable] to dynamic resource store
        gameSingleton->AddDynamicResource<UDataTable>(ERRResourceDataType::UE_DATA_TABLE, bpModelsDataTable, bpModelsDataTableName);

        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Log, TEXT("[%s] New BP-Entity models data table created!"), *bpModelsDataTableName);
    }

    // 2- [SEntityModelsInfo] -> [bpModelsDataTable]
    const int32 prevBPModelsNum = bpModelsDataTable->GetRowMap().Num();
    if (SEntityModelsInfo.Num() > 0)
    {
        // NOTE: Possibly new one will be added to [SEntityModelsInfo] thus cannot loop directly over it
        const auto currentEntityModelsInfo = SEntityModelsInfo;
        for (const auto& [entityModelName, entityModelInfo] : currentEntityModelsInfo)
        {
            // NOTE:
            // - Exclude already existing BP-DataTable-based ones
            // - For SingleCAD-based models, due to being not loaded from URDF/SDF, the created bp one has no initial link/joint info.
            // [entityModelInfo]'s link/joint prop list will be auto-filled from SingleCAD-based skeleton's bones but may happen much later as the robot is loaded up!
            // - SingleCAD-based models are generally expected to be manually-configured to make their robot instances functional.
            if (false == entityModelInfo->Data.IsUEDataTable())
            {
                const FString bpEntityModelName = GetBlueprintEntityModelName(entityModelName);

                // NOTE: Only create [bInOverwriteExisting] entry upon [] or not existing
                if (bInOverwriteExisting ||
                    (nullptr == bpModelsDataTable->FindRow<FRREntityModelTableRow>(FName(*bpEntityModelName), EMPTY_STR, false)))
                {
                    // Create [newBPEntityModelData] from [entityModelInfo]
                    // -> Add new entry to [bpModelsDataTable]
                    FRREntityModelData newBPEntityModelData = ComposeBlueprintEntityModelData(entityModelInfo->Data);
                    bpModelsDataTable->AddRow(FName(*bpEntityModelName), FRREntityModelTableRow(newBPEntityModelData));

                    // 2.1- Register [newBPEntityModelData] to [SEntityModelsInfo]
#if RAPYUTA_SIM_VERBOSE
                    UE_LOG_WITH_INFO_SHORT(LogRapyutaCore,
                                           Warning,
                                           TEXT("[DATA TABLE] NEW BP-ENTITY MODEL TO BE REGISTERED: %s!"),
                                           *bpEntityModelName);
#endif
                    RegisterEntityModel(FRREntityModelInfo(MoveTemp(newBPEntityModelData)), bInOverwriteExisting);
                }

                if (OutTotalEntityModelNameList)
                {
                    OutTotalEntityModelNameList->AddUnique(bpEntityModelName);
                }
            }
        }
    }

    // 3- Save [bpModelsDataTable] (only upon new creation or entries added) -> [UASSET] file on disk
    if (bNewCreation || (bpModelsDataTable->GetRowMap().Num() > prevBPModelsNum))
    {
        URRAssetUtils::SaveObjectToAssetInModule(bpModelsDataTable,
                                                 ERRResourceDataType::UE_DATA_TABLE,
                                                 bpModelsDataTableName,
                                                 RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME,
                                                 false,
                                                 false,
                                                 true,
                                                 true /*bOverwrite*/);
    }
    return bpModelsDataTable;
#else
    UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("BP entity models data table creation/updating is Editor-only!"));
    return nullptr;
#endif
}

bool URREntityCommon::LoadAllEntityModelsManufacturingTextures(bool bInSaveToAsset, bool bInForceReload)
{
    URRGameSingleton* gameSingleton = URRGameSingleton::Get();

    bool bResult = true;
    for (const auto& modelsFolderPath : EntityModelsFolderPathList)
    {
        TArray<FString> texturePathList;
        if (false == URRCoreUtils::LoadFullFilePaths(
                         modelsFolderPath,
                         texturePathList,
                         {ERRFileType::IMAGE_PNG, ERRFileType::IMAGE_TGA, ERRFileType::IMAGE_EXR, ERRFileType::IMAGE_HDR}))
        {
            UE_LOG_WITH_INFO_SHORT(
                LogRapyutaCore, Warning, TEXT("[%s] folder seems to be EMPTY of texture files!"), *modelsFolderPath);
            continue;
        }

        for (const auto& texturePath : texturePathList)
        {
            const FString textureName = FPaths::GetBaseFilename(texturePath);

            // Check texture in dynamic store first, only if NOT [bInForceReload]
            UTexture* texture = bInForceReload ? nullptr : gameSingleton->GetTexture(textureName, false);
            if (texture)
            {
                continue;
            }

            // ELSE: Only load texture of which the entity model is requested by user in INI
            // Search in [SEntityModelsInfo]
            bool bToBeLoaded = false;
            for (const auto& [entityModelName, entityModelInfo] : URREntityCommon::SEntityModelsInfo)
            {
                if (entityModelInfo->GetVisualMaterialInfo().HasTexture(textureName))
                {
                    bToBeLoaded = true;
                    break;
                }
            }

            if (!bToBeLoaded)
            {
                continue;
            }

            // Load texture from [texturePath.HDR, .EXR, etc.] on disk
            texture = URRCoreUtils::LoadImageToTexture(texturePath, textureName, bInSaveToAsset);
            if (texture)
            {
                // Add to GameSingleton's UE_TEXTURE resource store
                gameSingleton->AddDynamicResource<UTexture>(
                    ERRResourceDataType::UE_TEXTURE,
                    texture,
                    textureName,
                    bInSaveToAsset ? texture->GetPackage()->GetPathName() : texture->GetPathName());
            }
            else
            {
                UE_LOG_WITH_INFO_SHORT(
                    LogRapyutaCore, Error, TEXT("LoadImageToTexture [%s] failed %s"), *textureName, *texturePath);
                // NOTE: We want to try import as many textures as possible, thus not return here!
                bResult = false;
            }
        }
    }
    return bResult;
}

bool URREntityCommon::LoadEntityModelsManufacturingTextures(const TArray<FString>& InEntityModelsNameList,
                                                            bool bInSaveToAsset,
                                                            bool bInForceReload)
{
    URRGameSingleton* gameSingleton = URRGameSingleton::Get();

    bool bResult = true;
    for (const auto& modelsFolderPath : EntityModelsFolderPathList)
    {
        TArray<FString> texturePathList;
        if (false == URRCoreUtils::LoadFullFilePaths(
                         modelsFolderPath,
                         texturePathList,
                         {ERRFileType::IMAGE_PNG, ERRFileType::IMAGE_TGA, ERRFileType::IMAGE_EXR, ERRFileType::IMAGE_HDR}))
        {
            UE_LOG_WITH_INFO_SHORT(
                LogRapyutaCore, Warning, TEXT("[%s] folder seems to be EMPTY of texture files!"), *modelsFolderPath);
            continue;
        }

        for (const auto& entityModelName : InEntityModelsNameList)
        {
            const auto& entityModelInfo = SEntityModelsInfo.FindRef(entityModelName);

            // Verify [entityModelInfo]
            if (false == entityModelInfo.IsValid())
            {
                UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("[%s] is not yet registered!"), *entityModelName);
                bResult = false;
                continue;
            }

            // Load [entityModelInfo's textures]
            for (const auto& texturePath : texturePathList)
            {
                const FString textureName = FPaths::GetBaseFilename(texturePath);

                // ELSE: Only load texture specified in [entityModelInfo]
                if (entityModelInfo->GetVisualMaterialInfo().HasTexture(textureName))
                {
                    // Check texture in dynamic store first, only if NOT [bInForceReload]
                    UTexture* texture = bInForceReload ? nullptr : gameSingleton->GetTexture(textureName, false);
                    if (texture)
                    {
                        continue;
                    }

                    // Load texture from [texturePath.HDR, .EXR, etc.] on disk
                    texture = URRCoreUtils::LoadImageToTexture(texturePath, textureName, bInSaveToAsset);
                    if (texture)
                    {
                        // Add to GameSingleton's UE_TEXTURE resource store
                        gameSingleton->AddDynamicResource<UTexture>(
                            ERRResourceDataType::UE_TEXTURE,
                            texture,
                            textureName,
                            bInSaveToAsset ? texture->GetPackage()->GetPathName() : texture->GetPathName());
                    }
                    else
                    {
                        UE_LOG_WITH_INFO_SHORT(
                            LogRapyutaCore, Error, TEXT("LoadImageToTexture [%s] failed %s"), *textureName, *texturePath);
                        bResult = false;
                    }
                }
            }
        }
    }
    return bResult;
}
