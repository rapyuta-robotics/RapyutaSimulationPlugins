// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/TurtlebotROSController.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Tools/RRGeneralUtils.h"
#include "Tools/RRROS2TFPublisher.h"

ATurtlebotROSController::ATurtlebotROSController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
}

bool ATurtlebotROSController::InitPublishers(APawn* InPawn)
{
    bPublishOdom = true;
    bPublishOdomTf = true;

    if (false == Super::InitPublishers(InPawn))
    {
        return false;
    }

    return true;
}

void ATurtlebotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // Subscribe to [cmd_vel]
    SubscribeToMovementCommandTopic(TEXT("cmd_vel"));
}
