// Copyright (C) Rapyuta Robotics
#include "RRGameMode.h"

#include "RRGameState.h"
#include "RRPlayerController.h"

ARRGameMode::ARRGameMode()
{
    GameStateClass = ARRGameState::StaticClass();
    PlayerControllerClass = ARRPlayerController::StaticClass();
#if !WITH_EDITOR
    DefaultPawnClass = nullptr;
    SpectatorClass = nullptr;
#endif
}
