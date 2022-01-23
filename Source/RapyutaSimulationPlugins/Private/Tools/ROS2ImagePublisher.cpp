// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2ImagePublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/ROS2CameraComponent.h"

// rclUE
#include "Msgs/ROS2ImageMsg.h"

UROS2ImagePublisher::UROS2ImagePublisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("img");
}

void UROS2ImagePublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    auto* comp = Cast<UROS2CameraComponent>(DataSourceComponent);
    if (nullptr != comp)
    {
        CastChecked<UROS2ImageMsg>(InMessage)->SetMsg(comp->GetROS2Data());
    }
}
