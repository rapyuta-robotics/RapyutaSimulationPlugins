/**
 * @file RRNetworkGameState.h
 * @todo add doc
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */
#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/GameState.h"

#include "RRNetworkGameState.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRNetworkGameState : public AGameState
{
    GENERATED_BODY()

public:
    ARRNetworkGameState();

    virtual float GetServerWorldTimeSeconds() const override;
};
