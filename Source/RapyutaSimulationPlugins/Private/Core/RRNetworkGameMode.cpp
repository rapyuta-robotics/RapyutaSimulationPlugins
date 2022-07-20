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
    auto& simStateClient = networkPlayerController->SimulationStateClient;
    if (nullptr == simStateClient)
    {
        // Create [simStateClient]
        simStateClient = NewObject<URRROS2SimulationStateClient>(networkPlayerController, TEXT("ROS2SimStateClient"));
        simStateClient->SimulationState = SimulationState;

        // Update networkPlayerController's [SimulationState]
        networkPlayerController->SimulationState = SimulationState;
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("ARRNetworkGameMode::PostLogin Logged-in PC[%d] %s"),
               NetworkClientControllerList.Num(),
               *InPlayerController->GetName());
    }

#if WITH_EDITOR
    if (NetworkClientControllerList.Num() == 0)
    {
        networkPlayerController->SetName(URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME);
    }
    else
    {
        // TBD: "amr%d" is hardcoded to be the same as robot unique name?
        networkPlayerController->SetName(FString::Printf(TEXT("amr%d"), NetworkClientControllerList.Num()));
    }
#endif

    // Add to [ClientControllerList]
    NetworkClientControllerList.Add(networkPlayerController);
}
