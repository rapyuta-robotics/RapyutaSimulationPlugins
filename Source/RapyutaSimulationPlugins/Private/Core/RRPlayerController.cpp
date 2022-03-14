// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRPlayerController.h"

// UE
#include "Kismet/GameplayStatics.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameInstance.h"
#include "Core/RRGameState.h"
#include "Core/RRMathUtils.h"
#include "Core/RRROS2GameMode.h"
#include "Core/RRUObjectUtils.h"

ARRPlayerController::ARRPlayerController()
{
    bShowMouseCursor = true;
    DefaultMouseCursor = EMouseCursor::Crosshairs;
}

void ARRPlayerController::BeginPlay()
{
    Super::BeginPlay();

    GameMode = URRCoreUtils::GetGameMode<ARRGameMode>(this);
    check(GameMode);

    GameState = Cast<ARRGameState>(UGameplayStatics::GetGameState(this));
    check(GameState);

    GameInstance = Cast<URRGameInstance>(UGameplayStatics::GetGameInstance(this));
    check(GameInstance);

    // [Initialize()] is virtual, thus need to be run AFTER but OUTSIDE of [BeginPlay()]
    URRCoreUtils::PlanToExecuteOnNextTick(GetWorld(), [this]() { Initialize(); });
}

bool ARRPlayerController::Initialize()
{
    verify(GameState->HasInitialized(true));
    verify(GameState->HasSceneInstance(SceneInstanceId));

    ActorCommon = URRActorCommon::GetActorCommon(SceneInstanceId);
    check(ActorCommon);
    check(ActorCommon->SceneInstanceId == SceneInstanceId);

    // Not all maps has GlobalPostProcessVolume
    MainPostProcessVolume = Cast<APostProcessVolume>(URRUObjectUtils::FindPostProcessVolume(GetWorld()));

    return true;
}

bool ARRPlayerController::HasInitialized(bool bIsLogged) const
{
    if (!ActorCommon)
    {
        if (bIsLogged)
        {
            UE_LOG(LogRapyutaCore, Warning, TEXT("[ARRPlayerController]:: ActorCommon is NULL!"));
        }
        return false;
    }
    return true;
}
