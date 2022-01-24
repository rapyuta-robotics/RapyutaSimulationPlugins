// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2ImagePublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/ROS2CameraComponent.h"

// rclUE
#include "Msgs/ROS2ImageMsg.h"

UROS2ImagePublisher::UROS2ImagePublisher()
{
   TopicName = TEXT("raw_image");
   MsgClass = UROS2ImageMsg::StaticClass();
}