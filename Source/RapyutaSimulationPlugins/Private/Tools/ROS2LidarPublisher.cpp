// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2LidarPublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/RR2DLidarComponent.h"
#include "Sensors/RR3DLidarComponent.h"
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "Msgs/ROS2LaserScanMsg.h"
#include "Msgs/ROS2PointCloud2Msg.h"

UROS2LidarPublisher::UROS2LidarPublisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("scan");
}

void UROS2LidarPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    UpdateDelegate.BindDynamic(this, &UROS2LidarPublisher::UpdateMessage);
    Super::InitializeWithROS2(InROS2Node);

    // [ROS2LidarPublisher] must be already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::SensorData);
}

void UROS2LidarPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    // Noted: Elapsed time: time in seconds since world was brought up for play
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
