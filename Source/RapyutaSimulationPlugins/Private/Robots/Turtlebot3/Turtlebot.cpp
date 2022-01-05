// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/Turtlebot.h"

void ATurtlebot::OnConstruction(const FTransform& InTransform)
{
    Super::OnConstruction(InTransform);
    if (RobotVehicleMoveComponent)
    {
        RobotVehicleMoveComponent->SetFrameIds(TEXT("odom"), TEXT("base_footprint"));
    }
}
