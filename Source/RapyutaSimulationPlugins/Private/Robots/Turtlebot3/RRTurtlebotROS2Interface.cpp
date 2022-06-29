// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/RRTurtlebotROS2Interface.h"

bool URRTurtlebotROS2Interface::InitPublishers()
{
    bPublishOdom = true;
    bPublishOdomTf = true;

    if (false == Super::InitPublishers())
    {
        return false;
    }

    return true;
}
