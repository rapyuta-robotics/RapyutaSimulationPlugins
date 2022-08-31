/**
 * @file RRROS2GameMode.h
 * @brief ROS2 GameMode which have Clock publisher and ROS2 services to interact with ROS2.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Engine/EngineTypes.h"
#include "GameFramework/GameMode.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2SimulationStateClient.h"
#include "Tools/SimulationState.h"

#include "RRROS2GameMode.generated.h"

class AROS2Node;
class URRROS2ClockPublisher;

/**
 * @brief ROS2 GameMode which have Clock publisher and ROS2 services to interact with ROS2.
 * @sa [AGameMode](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AGameMode/)
 * @sa [GameMode and GameState](https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Framework/GameMode/
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRROS2GameMode : public AGameMode
{
    GENERATED_BODY()

public:
    ARRROS2GameMode();

    //! Sim's Main ROS2 node. This is not used by client-server and #ARRNetworkPlayerController has ROS2Node instead.
    UPROPERTY(BlueprintReadOnly)
    AROS2Node* MainROS2Node = nullptr;

    //! Sim's Main ROS2 node name. This is not used by client-server and #ARRNetworkPlayerController has ROS2Node instead.
    UPROPERTY(BlueprintReadWrite)
    FString MainROS2NodeName = TEXT("UEROS2Node");

    //! Publish /clock. This is not used by client-server without editor and #ARRNetworkPlayerController has ClockPublisher instead.
    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    //! Provide ROS2 implementation of sim-wide operations like get/set actor state, spawn/delete actor, attach/detach actor.
    UPROPERTY(BlueprintReadOnly)
    ASimulationState* MainSimState = nullptr;

    //! Provide ROS2 interface of sim-wide operations implemented by #MainSimState.
    //! @note #ASimulationState and #URRROS2SimulationStateClient are separeted to be used in client-server.
    UPROPERTY(BlueprintReadOnly)
    URRROS2SimulationStateClient* MainROS2SimStateClient = nullptr;

    //! Custom type to instantiate #MainROS2SimStateClient, configurable in child classes
    UPROPERTY(BlueprintReadOnly)
    TSubclassOf<URRROS2SimulationStateClient> ROS2SimStateClientClass = URRROS2SimulationStateClient::StaticClass();

protected:
    /**
     * @brief Initialize Game and call #InitSim.
     *
     * @param InMapName
     * @param InOptions
     * @param OutErrorMessage
     *
     * @sa [InitGame](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AGameMode/InitGame/)
     */
    virtual void InitGame(const FString& InMapName, const FString& InOptions, FString& OutErrorMessage) override;

    /**
     * @brief Create #MainSimState and call #InitROS2
     *
     */
    virtual void InitSim();

private:
    /**
     * @brief Create and initialize #MainROS2Node, #ClockPublisher and #MainSimState.
     *
     */
    void InitROS2();
};
