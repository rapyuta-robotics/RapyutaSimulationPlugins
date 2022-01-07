// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/TurtlebotROSController.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/Turtlebot3/Turtlebot.h"
#include "Tools/RRGeneralUtils.h"
#include "Tools/RRROS2TFPublisher.h"

ATurtlebotROSController::ATurtlebotROSController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
}

bool ATurtlebotROSController::InitPublishers(APawn* InPawn)
{
    if (false == Super::InitPublishers(InPawn))
    {
        return false;
    }

    ATurtlebot* turtlebot = CastChecked<ATurtlebot>(InPawn);

    // TFPublisher
    check(OdomPublisher->TFPublisher);
    OdomPublisher->TFPublisher->FrameId = URRGeneralUtils::ComposeROSFullFrameId(turtlebot->RobotUniqueName, TEXT("odom"));
    OdomPublisher->TFPublisher->ChildFrameId =
        URRGeneralUtils::ComposeROSFullFrameId(turtlebot->RobotUniqueName, TEXT("base_footprint"));

    return true;
}

void ATurtlebotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // Subscribe to [cmd_vel]
    SubscribeToMovementCommandTopic(TEXT("cmd_vel"));
}
