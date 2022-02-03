// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2EntityStatePublisher.h"

// rclUE
#include "Msgs/ROS2EntityStateMsg.h"

URRROS2EntityStatePublisher::URRROS2EntityStatePublisher()
{
   TopicName = TEXT("entity_state");
   MsgClass = UROS2EntityStateMsg::StaticClass();
}