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
#if WITH_EDITOR
    FString pcName = (NetworkClientControllerList.Num() == 0)
                         ? URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME
                         : FString::Printf(TEXT("NetworkPC%d"), NetworkClientControllerList.Num());
    // pc's server name
    networkPlayerController->SetName(pcName);
    // pc's client local name
    networkPlayerController->Rename(*pcName);
    // NOTE: PC's PlayerName would be updated later to Pawn's name after possession
    networkPlayerController->PlayerName = pcName;
#endif
    networkPlayerController->CreateROS2SimStateClient(ROS2SimStateClientClass);

    // Set [networkPlayerController's ServerSimState] -> [GameMode's MainSimState],
    // which should have been instantiated in [InitSim()]
    // NOTE: This could only be done here since from inside [networkPlayerController],
    // GameMode, which is server-only, is inaccessible
    check(MainSimState);
    networkPlayerController->ROS2SimStateClient->ServerSimState = MainSimState;
    networkPlayerController->ServerSimState = MainSimState;
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Log,
                     TEXT("Logged-in PC[%d] Name=%s, Id=%d"),
                     NetworkClientControllerList.Num(),
                     *networkPlayerController->GetName(),
                     networkPlayerController->GetPlayerState<APlayerState>()->GetPlayerId());

    // Add to [ClientControllerList]
    NetworkClientControllerList.Add(networkPlayerController);
}
