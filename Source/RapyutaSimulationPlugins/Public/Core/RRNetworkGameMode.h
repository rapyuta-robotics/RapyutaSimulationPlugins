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
 * @brief GameMode for client-server. This class handles #ANetworkPlayerController initialization from #PostLogin.
 * @sa [AGameMode](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/AGameMode/)
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRNetworkGameMode : public ARRROS2GameMode
{
    GENERATED_BODY()

public:
    ARRNetworkGameMode();

    UPROPERTY()
    TArray<ARRNetworkPlayerController*> NetworkClientControllerList;

protected:
    /**
     * @brief Called after a successful login
     * Create #ANetworkPlayerController::ROS2SimStateClient for each client.
     */
    virtual void PostLogin(APlayerController* InPlayer) override;
};
