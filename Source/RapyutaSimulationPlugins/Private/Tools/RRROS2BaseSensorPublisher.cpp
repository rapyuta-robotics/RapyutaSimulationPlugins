// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2BaseSensorPublisher.h"

#include "Sensors/RRROS2BaseSensorComponent.h"

URRROS2BaseSensorPublisher::URRROS2BaseSensorPublisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("sensor_data");
    MsgClass = UROS2GenericMsg::StaticClass();
    SetDefaultDelegates();    //use UpdateMessage as update delegate
}

void URRROS2BaseSensorPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (nullptr != DataSourceComponent && DataSourceComponent->bIsValid)
    {
        DataSourceComponent->SetROS2Msg(InMessage);
    }
}
