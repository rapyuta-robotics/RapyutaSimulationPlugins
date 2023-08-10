/**
 * @file RREntityCommon.h
 * @brief Common Entity (object, robot) Service Object which is shared among all #ARRBaseRobot
 * @copyright Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Misc/Paths.h"
#include "Templates/SharedPointer.h"

// RapyutaSimulationPlugins
#include "Core/RREntityStructs.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRObjectCommon.h"

#include "RREntityCommon.generated.h"
// (NOTE) To avoid cyclic inclusion, except for utils, please DO NOT include any other component header files here!

class ARRBaseRobot;
class ARRMeshActor;

// COMMON ENTITY SERVICE OBJECT
/**
 * @brief Common Entity (object, robot) Service Object which is shared among all #ARRBaseRobot
 * - Store collective entity models info, each of which is accessible by multi-threads
 * - Load entity models(urdf/sdf/meshes/data table) and stores to #SEntityModelsInfo and create/update entity UDataTable
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API URREntityCommon : public UObject
{
    GENERATED_BODY()
public:
    //! Absolute or relative (to either Project or its plugin dir) path list of parent folders housing Entity model description packages
    UPROPERTY(config)
    TArray<FString> EXTERNAL_ENTITY_MODELS_FOLDER_PATHS = {TEXT("ExternalData/GazeboModels"), TEXT("ExternalData/RobotModels")};

    /**
     * @brief Print sim INI config
     */
    virtual void PrintSimConfig() const;

    //! Store of all Entity models info
    static TMap<FString, TSharedPtr<FRREntityModelInfo, ESPMode::ThreadSafe>> SEntityModelsInfo;

    //! All absoluate paths to folders from which entity models are loaded, optionally including ones based on users-configured #EXTERNAL_ENTITY_MODELS_FOLDER_PATHS
    static TArray<FString> EntityModelsFolderPathList;

    /**
     * @brief Append entity model folder paths to #EntityModelsFolderPathList, based on InEntityCommon's user relative model path configs
     * @param InEntityCommon
     * @param InPluginModuleFolderName (optional) If null, project dir is used as the base path
     */
    static void ConfigureEntityModelsFolderPathList(URREntityCommon* InEntityCommon,
                                                    const TCHAR* InPluginModuleFolderName = nullptr);

    /**
     * @brief Check whether a directory has URDF/SDF of an entity model
     * @note This only checks URDF/SDF file name without reading its model property
     * @param InEntityModelsDirPath
     * @param InEntityModelName
     * @return true/false
     */
    static bool ModelsDirHasEntityModel(const FString& InEntityModelsDirPath, const FString& InEntityModelName);

    /**
     * @brief Load all entity models from #EntityModelsFolderPathList to #SEntityModelsInfo
     * 1. Use FRREntityDescriptionParser to parse urdf/sdf to FRREntityModelInfo for each model.
     * 2. Try loading entity models info from [DataTable]
     * @param OutEntityModelNameList Loaded entity models name list
     * @param bInForceReload Whether or not overwriting existing info
     * @return true
     * @return false
     */
    static bool LoadAllEntityModelsInfo(bool bInForceReload = false, TArray<FString>* OutEntityModelNameList = nullptr);

    /**
     * @brief Load a specific set of entity models from #EntityModelsFolderPathList to #SEntityModelsInfo
     * @param InEntityModelsNameList
     * @param InDescFileTypes Model description file types
     * @param bInForceReload Whether or not overwriting existing info
     * @return true/false
     */
    static bool LoadEntityModelsInfo(const TArray<FString>& InEntityModelsNameList,
                                     const TArray<ERRFileType>& InDescFileTypes = {ERRFileType::URDF, ERRFileType::SDF},
                                     bool bInForceReload = false);

    /**
     * @brief Load all data (info, textures, etc.) of a specific set of entity models
     * @param InEntityModelsNameList
     * @param InDescFileTypes Model description file types
     * @param bInSaveToAsset Whether or not saving data (eg textures) to uasset on disk
     * @param bInForceReload Whether or not overwriting existing data
     * @return true/false
     */
    static bool LoadEntityModelsAllData(const TArray<FString>& InEntityModelsNameList,
                                        const TArray<ERRFileType>& InDescFileTypes = {ERRFileType::URDF, ERRFileType::SDF},
                                        bool bInSaveToAsset = false,
                                        bool bInForceReload = false);

    /**
     * @brief Load all entity models' manufacturing textures
     * @param bInSaveToAsset Whether or not saving to uasset on disk
     * @param bInForceReload Whether or not overwriting existent one
     * @return true/false
     */
    static bool LoadAllEntityModelsManufacturingTextures(bool bInSaveToAsset = false, bool bInForceReload = false);

    /**
     * @brief Load manufacturing textures of a specific set of entity models
     * @param InEntityModelsNameList
     * @param bInSaveToAsset Whether or not saving to uasset on disk
     * @param bInForceReload Whether or not overwriting existent one
     * @return true/false
     */
    static bool LoadEntityModelsManufacturingTextures(const TArray<FString>& InEntityModelsNameList,
                                                      bool bInSaveToAsset = false,
                                                      bool bInForceReload = false);

    /**
     * @brief Whether an entity model is already loaded/registered
     * @param InEntityModelName
     * @return true/false
     */
    static bool IsEntityModelRegistered(const FString& InEntityModelName);
    /**
     * @brief Register an entity model
     * @param InModelInfo
     * @param bInOverwriteExisting Whether or not overwriting existing one
     */
    static void RegisterEntityModel(FRREntityModelInfo&& InModelInfo, bool bInOverwriteExisting = false);

    /**
     * @brief Unregister an entity model
     * @param InCADPathList
     * @param OutEntityModelNameList (optional)
     * @param bInOverwriteExisting Whether or not overwriting existing one
     */
    static void RegisterEntityModelsFromCADPaths(const TArray<FString>& InCADPathList,
                                                 TArray<FString>* OutEntityModelNameList = nullptr,
                                                 bool bInOverwriteExisting = false);

    /**
     * @brief Unregister an entity model
     * @param InEntityModelName
     */
    static void UnregisterEntityModel(const FString& InEntityModelName);

    /**
     * @brief Fetch entity model info as TSharedRef
     * @param InEntityModelName
     * @return TSharedRef<FRREntityModelInfo, ESPMode::ThreadSafe>
     */
    static TSharedRef<FRREntityModelInfo, ESPMode::ThreadSafe> GetEntityModelInfo(const FString& InEntityModelName);

    /**
     * @brief Check whether an entity model has been loaded from InDescriptionFilePath
     * @note This is used in [LoadAllEntityModelsInfo()] for the check before reading URDF/SDF thus not yet obtain the model name
     * @param InDescriptionFilePath
     * @return true/fa;se
     */
    static bool HasEntityDescriptionPathBeenLoaded(const FString& InDescriptionFilePath);

    /**
     * @brief Check whether an entity model has been loaded from InDescriptionFilePath
     * @param InDescriptionFilePath
     * @return true/fa;se
     */
    static TArray<FString> GetAllEntityModelsNameList()
    {
        TArray<FString> entityModelsNameList;
        SEntityModelsInfo.GenerateKeyArray(entityModelsNameList);
        return entityModelsNameList;
    }

    /**
     * @brief Get entity model size in case of a mesh-based entity
     * @param InEntityModelName
     * @return FVector
     */
    static FVector GetEntityModelSize(const FString& InEntityModelName)
    {
        return GetEntityModelInfo(InEntityModelName)->Data.TotalSize;
    }

    /**
     * @brief Get entity model size in case of a mesh-based entity
     * @param InEntity
     * @return FVector
     */
    static FVector GetEntityModelSize(ARRMeshActor* InEntity);
    /**
     * @brief Set entity model size in case of a mesh-based entity
     * @param InEntity
     * @param InEntityModelSize
     */

    static void SetEntityModelSize(const FString& InEntityModelName, const FVector& InEntityModelSize)
    {
        if (ensure(false == InEntityModelSize.IsZero()))
        {
            GetEntityModelInfo(InEntityModelName)->Data.TotalSize = InEntityModelSize;
        }
    }

    /**
     * @brief Get entity model's material info
     * @param InEntityModelName
     * @return FRRMaterialProperty
     */
    static FRRMaterialProperty GetEntityModelMaterialInfo(const FString& InEntityModelName)
    {
        return GetEntityModelInfo(InEntityModelName)->GetVisualMaterialInfo();
    }

    /**
     * @brief Get entity model's material info
     * @param InEntity
     * @return FRRMaterialProperty
     */
    static FRRMaterialProperty GetEntityModelMaterialInfo(ARRMeshActor* InEntity);

    /**
     * @brief Check whether an entity model name is of a blueprint-based entity
     * @param InEntityModelName
     * @return true/false
     */
    static bool IsBlueprintEntityModelName(const FString& InEntityModelName)
    {
        return InEntityModelName.StartsWith(TEXT("BP_"));
    }

    /**
     * @brief Get name of a blueprint-based entity model
     * @note Dynamic runtime cpp-only entities load their model info from URDF/SDF, while static BP-based entities load from UDataTable,
     * thus for clarity their model names need to be prefixed by "BP_"
     * @param InEntityModelName
     * @return FString
     */
    static FString GetBlueprintEntityModelName(const FString& InEntityModelName)
    {
        return IsBlueprintEntityModelName(InEntityModelName) ? InEntityModelName
                                                             : FString::Printf(TEXT("BP_%s"), *InEntityModelName);
    }

    /**
     * @brief Get name of the BP-entity models data table
     * @return FString
     */
    static FString GetBPEntityModelsDataTableName()
    {
        static FString sBPModelsDataTableName =
            FString::Printf(TEXT("%sBPEntityModels"), URRGameSingleton::GetAssetNamePrefix(ERRResourceDataType::UE_DATA_TABLE));
        return sBPModelsDataTableName;
    }

    /**
     * @brief Get local disk path to the BP-entity models data table file
     * @return FString
     */
    static FString GetBPEntityModelsDataTableFilePath()
    {
        static FString sBPModelsDataTableFilePath = FPackageName::LongPackageNameToFilename(
            URRGameSingleton::Get()->GetDynamicAssetPath(
                ERRResourceDataType::UE_DATA_TABLE, GetBPEntityModelsDataTableName(), RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME),
            FPackageName::GetAssetPackageExtension());
        return sBPModelsDataTableFilePath;
    }

    /**
     * @brief Compose new BP-Entity model data from an existing non-BP [FRREntityModelData]
     * @param InEntityModelData
     * @return FRREntityModelData
     */
    static FRREntityModelData ComposeBlueprintEntityModelData(const FRREntityModelData& InEntityModelData);

    /**
     * @brief Create/Update & Save UdataTable storing models info of Blueprint-based entitys from #SEntityModelsInfo
     * @param bInOverwriteExisting Whether or not overwriting existing entries
     * @param OutTotalEntityModelNameList Both native & BP entity models after update
     */
    static UDataTable* UpdateBlueprintEntityModelsDataTable(bool bInOverwriteExisting = false,
                                                            TArray<FString>* OutTotalEntityModelNameList = nullptr);
};
