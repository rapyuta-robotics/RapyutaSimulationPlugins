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

float ARRNetworkGameState::GetServerWorldTimeSeconds() const
{
    APlayerController* pc = GetGameInstance()->GetFirstLocalPlayerController(GetWorld());
    if (pc && IsNetMode(NM_Client))
    {
        auto* networkPC = CastChecked<ARRNetworkPlayerController>(pc);
        return networkPC->GetServerTime();
    }
    else
    {
        return GetWorld()->GetTimeSeconds();
    }
}
