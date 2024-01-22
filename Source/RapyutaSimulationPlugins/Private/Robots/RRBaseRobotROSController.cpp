// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRBaseRobotROSController.h"

// RapyutaSimulationPlugins
#include "Robots/RRBaseROS2Interface.h"
#include "Robots/RRBaseRobot.h"

void ARRBaseRobotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    auto* robot = Cast<ARRBaseRobot>(InPawn);
    if (robot)
    {
        robot->InitROS2Interface();
    }
    else
    {
        URRBaseROS2InterfaceComponent* ROS2InterfaceComponent = InPawn->FindComponentByClass<URRBaseROS2InterfaceComponent>();
        if (ROS2InterfaceComponent)
        {
            ROS2InterfaceComponent->ROS2Interface->Initialize(InPawn);
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Pawn does not have ROS2Interface as a child component."));
        }
    }
}

void ARRBaseRobotROSController::OnUnPossess()
{
    auto* robot = GetPawn<ARRBaseRobot>();
    if (robot)
    {
        robot->DeInitROS2Interface();
    }
    else
    {
        URRBaseROS2InterfaceComponent* ROS2InterfaceComponent = GetPawn()->FindComponentByClass<URRBaseROS2InterfaceComponent>();
        if (ROS2InterfaceComponent)
        {
            ROS2InterfaceComponent->ROS2Interface->DeInitialize();
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Pawn does not have ROS2Interface as a child component."));
        }
    }
    Super::OnUnPossess();
}
