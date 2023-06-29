// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRPlayerController.h"

// UE
#include "Kismet/GameplayStatics.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCamera.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameState.h"
#include "Core/RRMathUtils.h"
#include "Core/RRUObjectUtils.h"

ARRPlayerController::ARRPlayerController()
{
    bShowMouseCursor = true;
    DefaultMouseCursor = EMouseCursor::Crosshairs;
}

void ARRPlayerController::BeginPlay()
{
    Super::BeginPlay();

    if (IsNetMode(NM_Standalone))
    {
        GameMode = URRCoreUtils::GetGameMode<ARRGameMode>(this);
        GameState = URRCoreUtils::GetGameState<ARRGameState>(this);

        // [Initialize()] is virtual, thus need to be run AFTER but OUTSIDE of [BeginPlay()]
        URRCoreUtils::PlanToExecuteOnNextTick(GetWorld(), [this]() { Initialize(); });
    }
}

bool ARRPlayerController::Initialize()
{
    if (false == IsNetMode(NM_Standalone) || nullptr == GameMode || !GameMode->IsDataSynthSimType())
    {
        return true;
    }

#if RAPYUTA_USE_SCENE_DIRECTOR
    verify(GameState->HasInitialized(true));
    verify(GameState->HasSceneInstance(SceneInstanceId));
#endif

    ActorCommon = URRActorCommon::GetActorCommon(SceneInstanceId);

#if RAPYUTA_USE_SCENE_DIRECTOR
    check(ActorCommon);
    check(ActorCommon->SceneInstanceId == SceneInstanceId);

    SceneCamera = ActorCommon->SceneCamera;
    check(SceneCamera);
    if (GameMode->IsDataSynthSimType())
    {
        Possess(SceneCamera);
    }
    // Not all maps has GlobalPostProcessVolume
    MainPostProcessVolume = Cast<APostProcessVolume>(URRUObjectUtils::FindPostProcessVolume(GetWorld()));
#endif

    return true;
}

bool ARRPlayerController::HasInitialized(bool bIsLogged) const
{
    if (false == IsNetMode(NM_Standalone))
    {
        return true;
    }

#if RAPYUTA_USE_SCENE_DIRECTOR
    if (!ActorCommon)
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("ActorCommon is NULL!"));
        }
        return false;
    }

    if (!SceneCamera)
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("SceneCamera is NULL!"));
        }
        return false;
    }
#endif

    return true;
}
