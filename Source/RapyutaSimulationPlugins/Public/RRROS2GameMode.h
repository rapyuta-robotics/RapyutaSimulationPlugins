// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Engine/EngineTypes.h"
#include "GameFramework/GameMode.h"

// RapyutaSimulationPlugins

#include "RRROS2GameMode.generated.h"

class AROS2Node;
class UROS2Publisher;
class ASimulationState;
UCLASS() class RAPYUTASIMULATIONPLUGINS_API ARRROS2GameMode : public AGameMode
{
    GENERATED_BODY()

public:
    UPROPERTY()
    AROS2Node* ROS2Node = nullptr;

    UPROPERTY()
    UROS2Publisher* ClockPublisher = nullptr;

    UPROPERTY()
    ASimulationState* SimulationState = nullptr;

protected:
    virtual void BeginPlay() override;
};
