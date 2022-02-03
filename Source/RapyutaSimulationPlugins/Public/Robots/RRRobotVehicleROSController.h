// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RRRobotROSController.h"
#include "Sensors/RRBaseLidarComponent.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"

#include "RRRobotVehicleROSController.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRRobotVehicleROSController : public ARRRobotROSController
{
    GENERATED_BODY()

protected:
    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    UFUNCTION(BlueprintCallable)
    void SubscribeToMovementCommandTopic(const FString& InTopicName);
};
