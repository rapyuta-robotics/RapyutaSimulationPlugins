// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotVehicleROSController.h"

// RapyutaSimulationPlugins
#include "Robots/RRRobotBaseVehicle.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRRobotVehicleROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    auto* robotVehicle = GetPawn<ARRRobotBaseVehicle>();
    if(robotVehicle)
    {
        robotVehicle->InitROS2Interface();
    }
}

void ARRRobotVehicleROSController::OnUnPossess()
{
    auto* robotVehicle = GetPawn<ARRBaseRobot>();
    if (robotVehicle)
    {
        robotVehicle->StopROS2Interface();
    }
    Super::OnUnPossess();
}
