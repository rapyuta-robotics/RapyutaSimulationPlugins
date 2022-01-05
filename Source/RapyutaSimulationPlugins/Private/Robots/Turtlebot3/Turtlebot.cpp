// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/Turtlebot.h"

void ATurtlebot::OnConstruction(const FTransform& InTransform)
{
    Super::OnConstruction(InTransform);

    if (RobotVehicleMoveComponent)
    {
        RobotVehicleMoveComponent->FrameId = TEXT("odom");
        RobotVehicleMoveComponent->ChildFrameId = TEXT("base_footprint");
    }
}
