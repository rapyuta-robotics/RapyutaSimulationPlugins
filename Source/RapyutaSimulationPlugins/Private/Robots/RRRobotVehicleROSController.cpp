// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotVehicleROSController.h"

// UE
#include "Kismet/GameplayStatics.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RRRobotBaseVehicle.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRRobotVehicleROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // Create ROS2 interface
    if (nullptr == ROS2Interface)
    {
        ARRRobotBaseVehicle* robot = CastChecked<ARRRobotBaseVehicle>(InPawn);
        verify(robot->ROS2InterfaceClass);
        ROS2Interface = CastChecked<URRRobotROS2Interface>(URRUObjectUtils::CreateSelfSubobject(
            this, robot->ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));
    }
    ROS2Interface->Initialize(CastChecked<ARRRobotBaseVehicle>(InPawn));
}

void ARRRobotVehicleROSController::OnUnPossess()
{
    if (ROS2Interface)
    {
        ROS2Interface->StopPublishers();
    }
    Super::OnUnPossess();
}
