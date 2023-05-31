// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

// UE
#include "Core/RRNetworkGameState.h"

// RapyutaSimulationPlugins
#include "Core/RRNetworkPlayerController.h"

ARRNetworkGameState::ARRNetworkGameState()
{
    bReplicates = true;
    PrimaryActorTick.bCanEverTick = true;
}

double ARRNetworkGameState::GetServerWorldTimeSeconds() const
{
    // The simulated TimeSeconds on the server is the first network player controller's local time
    APlayerController* pc = GetGameInstance()->GetFirstLocalPlayerController(GetWorld());
    if (pc && IsNetMode(NM_Client))
    {
        auto* networkPC = CastChecked<ARRNetworkPlayerController>(pc);
        return networkPC->GetLocalTime();
    }
    else
    {
        return GetWorld()->GetTimeSeconds();
    }
}
