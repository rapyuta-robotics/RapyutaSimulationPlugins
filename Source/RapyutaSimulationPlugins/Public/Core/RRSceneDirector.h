/**
 * @file RRSceneDirector.h
 * @brief SceneDirector
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"

// UE
#include "Engine/DirectionalLight.h"
#include "Engine/PostProcessVolume.h"
#include "GameFramework/GameStateBase.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRBaseActor.h"
#include "Core/RRCamera.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRPlayerController.h"
#include "Core/RRTypeUtils.h"
#include "RapyutaSimulationPlugins.h"

#include "RRSceneDirector.generated.h"

DECLARE_DELEGATE_OneParam(FOnSpawnedActorsSettled, bool /*bIsForNewOperationBatch*/);
/**
 * @brief SceneDirector
 * @todo add documentation
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRSceneDirector : public ARRBaseActor
{
    GENERATED_BODY()

public:
    ARRSceneDirector();

    UPROPERTY()
    int32 OperationBatchId = 0;

    UPROPERTY()
    FString SceneName;

    UPROPERTY()
    APostProcessVolume* MainPostProcessVolume = nullptr;

    UPROPERTY()
    ARRCamera* SceneCamera = nullptr;

    UFUNCTION()
    bool HasSceneInitialized()
    {
        return bSceneInitialized;
    }

    FOnSpawnedActorsSettled OnSpawnedActorsSettled;

    virtual bool HasOperationCompleted(bool bIsLogged = false);

protected:
    virtual bool Initialize() override;
    virtual void Tick(float DeltaTime) override;

    // Start (Initialize + Run) Operation
    virtual bool InitializeOperation();
    virtual void RunOperation();
    virtual void ContinueOperation(bool bIsLastOperationSuccessful, bool bContinueRGBRandomizing)
    {
    }

    virtual void OnDataCollectionPhaseDone(bool bIsFinalDataCollectingPhase);
    virtual void EndSceneInstance();

    UPROPERTY()
    FTimerHandle DataCollectionTimerHandle;
    // Also use this if needed: FTimerDelegate DataCollectionTimerDelegate;

    UPROPERTY()
    uint8 bIsDataCollecting : 1;

    virtual void DoDataCollecting()
    {
        bIsDataCollecting = true;
    }

    UPROPERTY()
    uint8 bIsOperating : 1;

    UPROPERTY()
    int32 OperationBatchLoopLeft = 0;
    virtual void SpawnActors()
    {
    }

    virtual void ResetScene();

private:
    UFUNCTION()
    void TryInitializeOperation();

    UPROPERTY()
    FDateTime LastTimeStamp;

    UPROPERTY()
    uint8 bSceneInitialized : 1;
};
