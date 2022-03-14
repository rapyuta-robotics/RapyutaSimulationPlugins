// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"

// UE
#include "Engine/DirectionalLight.h"
#include "Engine/PostProcessVolume.h"
#include "GameFramework/GameStateBase.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRBaseActor.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRTypeUtils.h"
#include "RapyutaSimulationPlugins.h"

#include "RRSceneDirector.generated.h"

class ARRPlayerController;

DECLARE_DELEGATE_OneParam(FOnSpawnedActorsSettled, bool /*bIsForNewOperationBatch*/);

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRSceneDirector : public ARRBaseActor
{
    GENERATED_BODY()

public:
    UPROPERTY()
    int32 OperationBatchId = 0;

    UPROPERTY()
    FString SceneName;

    UPROPERTY()
    APostProcessVolume* MainPostProcessVolume = nullptr;

    UFUNCTION()
    bool HasSceneInitialized()
    {
        return bSceneInitialized;
    }

    FOnSpawnedActorsSettled OnSpawnedActorsSettled;

    virtual bool HasOperationCompleted(bool bIsLogged = false)
    {
        if (bIsLogged)
        {
            if (IsDataCollecting)
            {
                UE_LOG(LogRapyutaCore, Display, TEXT("SimModeInstance[%d] is still collecting data!"), SceneInstanceId);
            }
            else if (IsOperating)
            {
                UE_LOG(LogRapyutaCore, Display, TEXT("SimModeInstance[%d] is still operating!"), SceneInstanceId);
            }
        }
        return !(IsDataCollecting || IsOperating);
    }

protected:
    virtual bool Initialize() override;
    virtual void Tick(float DeltaTime) override;

    // Start (Initialize + Run) Operation
    virtual bool InitializeOperation();
    virtual void RunOperation();

    virtual void ContinueOperation(bool bIsLastOperationSuccessful, bool bContinueRandomizing)
    {
    }

    virtual void OnDataCollectionPhaseDone(bool bIsFinalDataCollectingPhase);
    virtual void EndSceneInstance();

    UPROPERTY()
    FTimerHandle DataCollectionTimerHandle;
    // Also use this if needed: FTimerDelegate DataCollectionTimerDelegate;

    UPROPERTY()
    bool IsDataCollecting = false;

    virtual void DoDataCollecting()
    {
        IsDataCollecting = true;
    }

    UPROPERTY()
    bool IsOperating = false;

    UPROPERTY()
    int32 OperationBatchLoopLeft = 0;
    virtual void SpawnActors()
    {
    }

private:
    UFUNCTION()
    void TryInitializeOperation();

    UPROPERTY()
    FDateTime LastTimeStamp;

    UPROPERTY()
    bool bSceneInitialized = false;
};
