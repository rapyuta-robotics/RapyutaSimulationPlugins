// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2LidarPublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/RR2DLidarComponent.h"
#include "Sensors/RR3DLidarComponent.h"
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "Msgs/ROS2LaserScanMsg.h"
#include "Msgs/ROS2PointCloud2Msg.h"

URRROS2LidarPublisher::URRROS2LidarPublisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("scan");
}

void URRROS2LidarPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    RegisterToROS2Node(InROS2Node);

    // [ROS2LidarPublisher] must be already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::SensorData);
}

void URRROS2LidarPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    auto* lidar2D = Cast<URR2DLidarComponent>(LidarComponent);
    if (nullptr != lidar2D)
    {
        CastChecked<UROS2LaserScanMsg>(InMessage)->SetMsg(lidar2D->GetROS2Data());
    }
    else
    {
        auto* lidar3D = Cast<URR3DLidarComponent>(LidarComponent);
        if (nullptr != lidar3D)
        {
            CastChecked<UROS2PointCloud2Msg>(InMessage)->SetMsg(lidar3D->GetROS2Data());
        }
    }
}
