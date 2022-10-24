// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRBaseRobotROSController.h"

// RapyutaSimulationPlugins
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRBaseRobotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    auto* robot = Cast<ARRBaseRobot>(InPawn);
    if (robot)
    {
        if (robot->IsROS2SystemEnabled())
        {
            robot->InitROS2Interface();
        }
        else
        {
            UE_LOG(
                LogRapyutaCore, Warning, TEXT("[%s][ARRBaseRobotROSController::OnPossess] ROS2 System is not enabled"), *GetName());
        }
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
    auto* robot = GetPawn<ARRBaseRobot>();
    if (robot)
    {
        if (robot->IsROS2SystemEnabled())
        {
            robot->DeInitROS2Interface();
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s][ARRBaseRobotROSController::OnUnPossess] ROS2 System is not enabled"),
                   *GetName());
        }
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
