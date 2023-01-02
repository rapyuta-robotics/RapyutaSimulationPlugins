// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"
#include "Tools/RRROS2StatePublisher.h"

#include "RRROS2StatePublisherTester.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRROS2StatePublisherTester : public AActor
{
    GENERATED_BODY()

public:
    ARRROS2StatePublisherTester();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;

    UPROPERTY()
    URRROS2StatePublisher* StatePub = nullptr;

    // should be its own node? one for each namespace?
    UPROPERTY()
    UROS2Node* ROS2Node = nullptr;

    UPROPERTY()
    ARobotVehicle* RobotVehicle = nullptr;
};
