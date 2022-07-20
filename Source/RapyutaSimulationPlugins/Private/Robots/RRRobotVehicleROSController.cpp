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

    // NOTE: Init InPawn's ROS2 interface here (not right after its creation in ARRBaseRobot) for:
    // + Controller, upon posses/unpossess, acts as the pivot to start/stop robot's ROS2Interface
    // + InPawn's ROS2Interface, due to requirements for also instantiatable in ARRBaseRobot's child BPs, may not have been
    // instantiated yet
    // + InPawn's child class' ros2-related accessories (ROS2 node, sensors, publishers/subscribers)
    //  may have not been fully accessible until now
    auto* robotVehicle = CastChecked<ARRRobotBaseVehicle>(InPawn);
    verify(IsValid(robotVehicle->ROS2Interface));
    ROS2Interface = robotVehicle->ROS2Interface;
    ROS2Interface->Initialize(robotVehicle);
}

void ARRRobotVehicleROSController::OnUnPossess()
{
    if (ROS2Interface)
    {
        ROS2Interface->StopPublishers();
    }
    Super::OnUnPossess();
}
