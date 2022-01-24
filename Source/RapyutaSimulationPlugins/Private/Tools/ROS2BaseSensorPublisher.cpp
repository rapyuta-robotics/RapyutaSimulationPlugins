// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2BaseSensorPublisher.h"

UROS2BaseSensorPublisher::UROS2BaseSensorPublisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("sensor_data");
    MsgClass = UROS2GenericMsg::StaticClass();
}

void UROS2BaseSensorPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (nullptr != DataSourceComponent)
    {
        DataSourceComponent->SetROS2Msg(InMessage);
    }
}
