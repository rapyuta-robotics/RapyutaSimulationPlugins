// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/Turtlebot.h"

void ATurtlebot::ConfigureVehicleMoveComponent()
{
    Super::ConfigureVehicleMoveComponent();
    check(RobotVehicleMoveComponent);
    // [FrameIds] will then be used for odom init in Super::InitializeMoveComponent(),
    // where [RobotVehicleMoveComponent's PawnOwner] has been available thus they must be inited beforehands.
    RobotVehicleMoveComponent->SetFrameIds(TEXT("odom"), TEXT("base_footprint"));
}
