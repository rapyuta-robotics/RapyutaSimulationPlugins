// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Engine/EngineTypes.h"
#include "GameFramework/GameMode.h"

// RapyutaSimulationPlugins
#include "Tools/SimulationState.h"

#include "RRROS2GameMode.generated.h"

class AROS2Node;
class URRROS2ClockPublisher;
UCLASS() class RAPYUTASIMULATIONPLUGINS_API ARRROS2GameMode : public AGameMode
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadOnly)
    AROS2Node* ROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    UPROPERTY(BlueprintReadOnly)
    ASimulationState* SimulationState = nullptr;

    UPROPERTY(BlueprintReadOnly)
    TSubclassOf<ASimulationState> SimulationStateClass = ASimulationState::StaticClass();

    UPROPERTY(BlueprintReadWrite)
    FString UENodeName = TEXT("UENode");

protected:
    virtual void InitGame(const FString& InMapName, const FString& InOptions, FString& OutErrorMessage) override;
    virtual void InitSim();

private:
    void InitROS2();
};
