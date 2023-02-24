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
 * @brief Execute Init/Run/Continue Sim type-specific operations (Data synthesizer/collection or Robot operations, etc. or compound) with #ARRSceneInstance.
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

    UPROPERTY()
    double DataCollectionTimeStamp = 0.f;

    UPROPERTY()
    TArray<int32> SceneEntityMaskValueList;

protected:
    /**
    * @brief Call #TryInitializeOperation() repeatedly.
    * 
    * @return true 
    * @return false 
    */
    virtual bool Initialize() override;

    //! Start (Initialize + Run) Operation


    /**
     * @brief Get #RRActorCommon, ScneneCamera, MainPostProcessVolume, and #RunOperation().
     * 
     * @return true 
     * @return false 
     */
    virtual bool InitializeOperation();

    /**
     * @brief Called inside from #InitializeOperation and #SpawnActors().
     * 
     */
    virtual void RunOperation();

    virtual void ContinueOperation(bool bIsLastOperationSuccessful, bool bContinueRGBRandomizing)
    {
    }

    virtual void OnDataCollectionPhaseDone(bool bIsFinalDataCollectingPhase);
    virtual void EndSceneInstance();

    UPROPERTY()
    FTimerHandle InitializationTimerHandle;

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
    /**
     * @brief Initialize Scene by #InitializeOperation() or exit with timeout.
     * 
     */
    UFUNCTION()
    void TryInitializeOperation();

    UPROPERTY()
    FDateTime LastTimeStamp;

    UPROPERTY()
    uint8 bSceneInitialized : 1;
};
