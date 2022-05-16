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
#include "Core/RRGameSingleton.h"
#include "Core/RRTypeUtils.h"
#include "RapyutaSimulationPlugins.h"

#include "RRSceneDirector.generated.h"

class ARRPlayerController;

/**
 * @brief SceneDirector
 * @todo add documentation
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRSceneDirector : public ARRBaseActor
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FString SceneName;

    UPROPERTY()
    APostProcessVolume* MainPostProcessVolume = nullptr;

    UFUNCTION()
    bool HasSceneInitialized()
    {
        return bSceneInitialized;
    }

    virtual bool HasOperationCompleted(bool bIsLogged = false)
    {
        if (bIsLogged)
        {
            if (IsOperating)
            {
                UE_LOG(LogRapyutaCore, Display, TEXT("SimModeInstance[%d] is still operating!"), SceneInstanceId);
            }
        }
        return !IsOperating;
    }

protected:
    virtual bool Initialize() override;
    virtual void Tick(float DeltaTime) override;

    // Start (Initialize + Run) Operation
    virtual bool InitializeOperation();
    virtual void RunOperation();
    virtual void EndSceneInstance();

    UPROPERTY()
    bool IsOperating = false;

private:
    UFUNCTION()
    void TryInitializeOperation();

    UPROPERTY()
    FDateTime LastTimeStamp;

    UPROPERTY()
    bool bSceneInitialized = false;
};
