// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "GameFramework/GameState.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"

#include "RRGameState.generated.h"

class ARRGameMode;
class URRGameInstance;
class ARRPlayerController;
class ARRMeshActor;

UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API ARRGameState : public AGameState
{
    GENERATED_BODY()

public:
    ARRGameState();
    virtual void StartSim();

    static constexpr const TCHAR* RAPYUTA_SIM_VERSION_FILE_NAME = TEXT("RapyutaSimVersion.ini");
    UFUNCTION()
    FString GetSimVersionFilePath() const
    {
        return GetSimOutputsBaseFolderPath() / RAPYUTA_SIM_VERSION_FILE_NAME;
    }

    UPROPERTY(config)
    int8 SCENE_INSTANCES_NUM = 1;

    UPROPERTY(config)
    int32 RGB_RANDOMIZATION_NUM = 1;

    UPROPERTY(config)
    int32 OPERATION_BATCH_NUM = 10;

    UFUNCTION()
    uint64 GetTotalPlannedCaptureCount()
    {
        static const uint64 sPlannedNum = SCENE_INSTANCES_NUM * RGB_RANDOMIZATION_NUM * OPERATION_BATCH_NUM;
        return sPlannedNum;
    }

    UPROPERTY()
    ARRGameMode* GameMode = nullptr;

    UPROPERTY()
    URRGameInstance* GameInstance = nullptr;

    static constexpr const float SCENE_INSTANCES_DISTANCE_INTERVAL = 2000.f;
    UPROPERTY()
    TArray<URRSceneInstance*> SceneInstanceList;

    template<typename TScenceInstanceClass>
    TScenceInstanceClass* GetSceneInstance(int8 InSceneInstanceId) const
    {
        TScenceInstanceClass* instance = SceneInstanceList.IsValidIndex(InSceneInstanceId)
                                           ? Cast<TScenceInstanceClass>(SceneInstanceList[InSceneInstanceId])
                                           : nullptr;
        verify(instance);
        return instance;
    }

    UFUNCTION()
    bool HasSceneInstance(int8 InSceneInstanceId)
    {
        return SceneInstanceList.IsValidIndex(InSceneInstanceId) && SceneInstanceList[InSceneInstanceId];
    }

    UFUNCTION()
    virtual bool HasSceneInstanceListBeenCreated(bool bIsLogged = false) const;

    UFUNCTION()
    virtual bool HasInitialized(bool bIsLogged = false) const;

    // SIM MESH ACTORS
    UPROPERTY()
    TArray<ARRMeshActor*> TotalMeshActors;

    // SIM OUTPUTS ==
    static constexpr const TCHAR* SIM_LOG_DATA_FILE_NAME = TEXT("RRSimLogData.db");
    static constexpr const TCHAR* SIM_LOG_DATA_FOLDER_NAME = TEXT("RRSimLog");
    UPROPERTY(config)
    FString FOLDER_NAME_OUTPUTS_BASE = TEXT("OutputData");

    // To faciliate testing on CI, Outputs base folder need to be cleared duing the test.
    // Thus, it would be clearer as using [ProjectSavedDir()] as the CI default output folder.
    UFUNCTION()
    FString GetSimOutputsBaseFolderPath() const
    {
        return FPaths::IsRelative(FOLDER_NAME_OUTPUTS_BASE)
                 ? FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / FOLDER_NAME_OUTPUTS_BASE)
                 : FOLDER_NAME_OUTPUTS_BASE;
    }

    UFUNCTION()
    FString GetSimLogFolderPath()
    {
        return GetSimOutputsBaseFolderPath() / SIM_LOG_DATA_FOLDER_NAME;
    }

    UFUNCTION()
    FString GetSimLogFilePath()
    {
        return GetSimLogFolderPath() / SIM_LOG_DATA_FILE_NAME;
    }

protected:
    virtual void CreateSceneInstance(int8 InSceneInstanceId);
    virtual void InitializeSim(int8 InSceneInstanceId);
    virtual void StartSubSim(int8 InSceneInstanceId);
    virtual void CreateServiceObjects(int8 InSceneInstanceId);

    virtual void FinalizeSim();
    virtual void PrintSimConfig() const;
    virtual void BeginPlay() override;
    virtual void BeginSubPlay();
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    virtual void Tick(float DeltaTime) override;
    virtual void OnTick(float DeltaTime);

protected:
    UPROPERTY()
    TSubclassOf<URRSceneInstance> SceneInstanceClass;
};
