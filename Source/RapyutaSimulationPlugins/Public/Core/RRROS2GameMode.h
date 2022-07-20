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
#include "Tools/SimulationState.h"
#include "Tools/SimulationStateClient.h"

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
    ARRROS2GameMode();

    UPROPERTY(BlueprintReadOnly)
    AROS2Node* ROS2Node = nullptr;

    //! Publish /clock
    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    //! Provide ROS2 interface to get/set actor state, spawn/delete actor, attach/detach actor.
    UPROPERTY(BlueprintReadOnly)
    ASimulationState* SimulationState = nullptr;

    UPROPERTY(BlueprintReadOnly)
    TSubclassOf<ASimulationState> SimulationStateClass = ASimulationState::StaticClass();

    UPROPERTY(BlueprintReadWrite)
    FString UENodeName = TEXT("UENode");

    TArray<class APlayerController*> ClientControllerList;

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
    virtual void PostLogin(APlayerController* InPlayer) override;

private:
   UPROPERTY()
   int32 numPlayers = 0;
    /**
     * @brief Create and initialize #ROS2Node, #ClockPublisher and #SimulationState.
     * 
     */
    void InitROS2();
};
