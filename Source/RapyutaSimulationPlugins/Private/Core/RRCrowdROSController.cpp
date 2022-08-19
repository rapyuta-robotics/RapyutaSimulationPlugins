// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#include "Core/RRCrowdROSController.h"

// RapyutaSimulationPlugins
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRCrowdROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // NOTE: Init InPawn's ROS2 interface here (not right after its creation in ARRBaseRobot) for:
    // + Controller, upon posses/unpossess, acts as the pivot to start/stop robot's ROS2Interface
    // + InPawn's ROS2Interface, due to requirements for also instantiatable in ARRBaseRobot's child BPs, may not have been
    // instantiated yet
    // + InPawn's child class' ros2-related accessories (ROS2 node, sensors, publishers/subscribers)
    //  may have not been fully accessible until now
    auto* robotVehicle = GetPawn<ARRBaseRobot>();
    if(robotVehicle)
    {
        robotVehicle->InitROS2Interface();
    }
}

void ARRCrowdROSController::OnUnPossess()
{
    auto* robotVehicle = GetPawn<ARRBaseRobot>();
    if (robotVehicle)
    {
        robotVehicle->StopROS2Interface();
    }
    Super::OnUnPossess();
}
