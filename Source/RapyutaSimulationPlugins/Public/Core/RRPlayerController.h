// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#pragma once
// UE
#include "GameFramework/PlayerController.h"
#include "UnrealClient.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"

#include "RRPlayerController.generated.h"

class ARRGameMode;
class ARRGameState;
class URRGameInstance;
class ARRActorCommon;

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
    FPostProcessSettings PostProcessSettings;
    UPROPERTY()
    APostProcessVolume* MainPostProcessVolume = nullptr;

    virtual bool HasInitialized(bool bIsLogged = false) const;

protected:
    virtual bool Initialize();
    virtual void BeginPlay() override;
};
