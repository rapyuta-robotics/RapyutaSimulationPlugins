// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotVehicleROSController.h"

// RapyutaSimulationPlugins
#include "Robots/RRRobotBaseVehicle.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRRobotVehicleROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // NOTE: in case of NetworkGameMode, this will be done by [ARRNetworkPlayerController]
    // Refer to ARRBaseRobot::CreateROS2Interface() for reasons why it is inited here but not earlier
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
