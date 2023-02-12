/**
 * @file RRPlayerController.h
 * @brief Player controller with ARRGameMode, ARRGameState, URRGameInstance and URRActorCommon.
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
// UE
#include "GameFramework/PlayerController.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"

#include "RRPlayerController.generated.h"

class ARRGameMode;
class ARRGameState;
class URRGameInstance;
class ARRActorCommon;
class ARRCamera;

/**
 * @brief Player controller with #ARRGameMode, #ARRGameState, #URRGameInstance and #URRActorCommon
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ARRPlayerController();

    UPROPERTY()
    ARRGameMode* GameMode = nullptr;

    UPROPERTY()
    ARRGameState* GameState = nullptr;

    UPROPERTY()
    URRGameInstance* GameInstance = nullptr;

    UPROPERTY()
    int8 SceneInstanceId = URRActorCommon::DEFAULT_SCENE_INSTANCE_ID;

    UPROPERTY()
    URRActorCommon* ActorCommon = nullptr;

    UPROPERTY()
    ARRCamera* SceneCamera = nullptr;

    UPROPERTY()
    FPostProcessSettings PostProcessSettings;
    UPROPERTY()
    APostProcessVolume* MainPostProcessVolume = nullptr;

    virtual bool HasInitialized(bool bIsLogged = false) const;

protected:
    virtual bool Initialize();

    /**
     * @brief Get and initialize ARRGameMode, ARRGameState, URRGameInstance and URRActorCommon.
     *
     */
    virtual void BeginPlay() override;
};
