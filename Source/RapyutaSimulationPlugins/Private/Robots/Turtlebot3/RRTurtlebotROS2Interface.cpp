// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/RRTurtlebotROS2Interface.h"

// example of override of ros parameter
void URRTurtlebotROS2Interface::SetupROSParams()
{
    bPublishOdom = true;
    bPublishOdomTf = true;
}
