// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2LidarPublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/RR2DLidarComponent.h"
#include "Sensors/RR3DLidarComponent.h"
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "Msgs/ROS2LaserScan.h"
#include "Msgs/ROS2PointCloud2.h"

URRROS2LaserScanPublisher::URRROS2LaserScanPublisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("scan");
    MsgClass = UROS2LaserScanMsg::StaticClass();
}

URRROS2PointCloud2Publisher::URRROS2PointCloud2Publisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("scan");
    MsgClass = UROS2PointCloud2Msg::StaticClass();
}
