// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/TurtlebotROSController.h"

ATurtlebotROSController::ATurtlebotROSController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    TFFrameId = TEXT("odom");
    TFChildFrameId = TEXT("base_footprint");
    OdomTopicName = TEXT("odom");
    CommandTopicName = TEXT("cmd_vel");
}
