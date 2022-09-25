// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ImagePublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/RRROS2CameraComponent.h"

// rclUE
#include "Msgs/ROS2Img.h"

URRROS2ImagePublisher::URRROS2ImagePublisher()
{
   TopicName = TEXT("raw_image");
   MsgClass = UROS2ImgMsg::StaticClass();
}