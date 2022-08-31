/**
 * @file RRNetworkGameMode.h
 * @brief NetworkGameMode for server-client application
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Engine/EngineTypes.h"

// RapyutaSimulationPlugins
#include "Core/RRROS2GameMode.h"
#include "Tools/RRROS2SimulationStateClient.h"
#include "Tools/SimulationState.h"

#include "RRNetworkGameMode.generated.h"

class ARRNetworkPlayerController;

/**
 * @brief ROS2 GameMode which have Clock publisher and ROS2 services to interact with ROS2.
 * @sa [AGameMode](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AGameMode/)
 */
UCLASS() class RAPYUTASIMULATIONPLUGINS_API ARRNetworkGameMode : public ARRROS2GameMode
{
    GENERATED_BODY()

public:
    ARRNetworkGameMode();

    UPROPERTY()
    TArray<ARRNetworkPlayerController*> NetworkClientControllerList;

protected:
    /**
     * @brief Called after a successful login
     */
    virtual void PostLogin(APlayerController* InPlayer) override;
};
