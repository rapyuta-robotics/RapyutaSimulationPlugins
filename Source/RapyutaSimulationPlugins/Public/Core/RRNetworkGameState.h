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

/**
 * @brief Network Game State for client-server
 * @sa [GameMode and GameState](https://docs.unrealengine.com/5.1/en-US/game-mode-and-game-state-in-unreal-engine/)
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRNetworkGameState : public AGameState
{
    GENERATED_BODY()

public:
    ARRNetworkGameState();

    /**
     * @brief Get the Server World Time Seconds object
     *
     * @return float
     */
    virtual double GetServerWorldTimeSeconds() const override;
};
