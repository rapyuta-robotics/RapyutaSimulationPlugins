// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

// UE
#include "CoreMinimal.h"
#include "GameFramework/GameStateBase.h"

#include "Core/RRNetworkPlayerController.h"

#include "Core/RRNetworkGameState.h"

ARRNetworkGameState::ARRNetworkGameState()
{
    bReplicates = true;
    PrimaryActorTick.bCanEverTick = true;
}

float ARRNetworkGameState::GetServerWorldTimeSeconds() const
{
    APlayerController* Player = GetGameInstance()->GetFirstLocalPlayerController(GetWorld());
    if(Player && GetNetMode() == NM_Client)
    {
        ARRNetworkPlayerController* NetworkPlayer = Cast<ARRNetworkPlayerController>(Player);
        return NetworkPlayer->GetServerTime();
    }
    else
    {
        return GetWorld()->GetTimeSeconds();
    }
}