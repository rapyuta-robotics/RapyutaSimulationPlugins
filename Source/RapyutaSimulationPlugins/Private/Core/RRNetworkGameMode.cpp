// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Core/RRNetworkGameMode.h"

// RapyutaSimulationPlugins
#include "Core/RRNetworkGameState.h"
#include "Core/RRNetworkPlayerController.h"
#include "Tools/RRROS2SimulationStateClient.h"

ARRNetworkGameMode::ARRNetworkGameMode()
{
    GameStateClass = ARRNetworkGameState::StaticClass();
    PlayerControllerClass = ARRNetworkPlayerController::StaticClass();
}

void ARRNetworkGameMode::PostLogin(APlayerController* InPlayerController)
{
    Super::PostLogin(InPlayerController);

    auto* networkPlayerController = CastChecked<ARRNetworkPlayerController>(InPlayerController);
    networkPlayerController->CreateROS2SimStateClient(ROS2SimStateClientClass);

    // Set [networkPlayerController's ServerSimState] -> [GameMode's MainSimState]
    networkPlayerController->ServerSimState = MainSimState;
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("ARRNetworkGameMode::PostLogin Logged-in PC[%d] %s"),
           NetworkClientControllerList.Num(),
           *InPlayerController->GetName());

#if WITH_EDITOR
#if RAPYUTA_SIM_DEBUG
    if (NetworkClientControllerList.Num() == 0)
    {
        // This should be configured in INI or passed in cmdline?
        networkPlayerController->SetName(URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME);
    }
#endif
#endif

    // Add to [ClientControllerList]
    NetworkClientControllerList.Add(networkPlayerController);
}
