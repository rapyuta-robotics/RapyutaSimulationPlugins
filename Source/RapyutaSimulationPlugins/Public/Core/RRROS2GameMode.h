/**
 * @file RRROS2GameMode.h
 * @brief ROS2 GameMode which have Clock publisher and RoS2 services to interact with ROS2.
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
 * @brief ROS2 GameMode which have Clock publisher and RoS2 services to interact with ROS2.
 * @sa [AGameMode](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AGameMode/)
 */
UCLASS() class RAPYUTASIMULATIONPLUGINS_API ARRROS2GameMode : public AGameMode
{
    GENERATED_BODY()

public:
    //! Sim's Main ROS2 node
    UPROPERTY(BlueprintReadOnly)
    AROS2Node* MainROS2Node = nullptr;

    //! Sim's Main ROS2 node name
    UPROPERTY(BlueprintReadWrite)
    FString MainROS2NodeName = TEXT("UEROS2Node");

    //! Publish /clock
    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    //! Provide ROS2 implementation of sim-wide operations like get/set actor state, spawn/delete actor, attach/detach actor.
    UPROPERTY(BlueprintReadOnly)
    ASimulationState* SimulationState = nullptr;

    //! Provide ROS2 interface of sim-wide operations implemented by #SimulationState
    UPROPERTY(BlueprintReadOnly)
    URRROS2SimulationStateClient* ROS2SimStateClient = nullptr;

    //! Custom type to instantiate #SimulationState, configurable in child classes
    UPROPERTY(BlueprintReadOnly)
    TSubclassOf<ASimulationState> SimulationStateClass = ASimulationState::StaticClass();

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
     * @brief Call #InitROS
     *
     */
    virtual void InitSim();

private:
    /**
     * @brief Create and initialize #ROS2Node, #ClockPublisher and #SimulationState.
     *
     */
    void InitROS2();
};
