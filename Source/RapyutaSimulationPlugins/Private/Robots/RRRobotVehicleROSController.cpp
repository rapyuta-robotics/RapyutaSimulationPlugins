// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotVehicleROSController.h"

// UE
#include "Kismet/GameplayStatics.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RRRobotROS2Interface.h"
#include "Robots/RobotVehicle.h"

void ARRRobotVehicleROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // Create ROS2 interface
    if (nullptr == ROS2Interface)
    {
        ARobotVehicle* robot = CastChecked<ARobotVehicle>(InPawn);
        verify(robot->ROS2InterfaceClass);
        ROS2Interface = CastChecked<URRRobotROS2Interface>(URRUObjectUtils::CreateSelfSubobject(
            this, robot->ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));
    }
    ROS2Interface->Initialize(CastChecked<ARobotVehicle>(InPawn));
}

void ARRRobotVehicleROSController::OnUnPossess()
{
    if (ROS2Interface)
    {
        ROS2Interface->StopPublishers();
    }
    Super::OnUnPossess();
}
