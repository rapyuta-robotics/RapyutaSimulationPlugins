// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRBaseRobotROSController.h"

// RapyutaSimulationPlugins
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRBaseRobotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    auto* robotVehicle = Cast<ARRBaseRobot>(InPawn);
    if (robotVehicle)
    {
        robotVehicle->InitROS2Interface();
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s][ARRBaseRoboROSController::OnPossess] Pawn is not child class of ARRBaseRobot "),
               *GetName());
    }
}

void ARRBaseRobotROSController::OnUnPossess()
{
    auto* robotVehicle = GetPawn<ARRBaseRobot>();
    if (robotVehicle)
    {
        robotVehicle->StopROS2Interface();
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s][ARRBaseRobotROSController::OnUnPossess] Pawn is not child class of ARRBaseRobot "),
               *GetName());
    }
    Super::OnUnPossess();
}
