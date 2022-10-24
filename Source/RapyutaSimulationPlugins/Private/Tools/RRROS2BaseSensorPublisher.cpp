// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2BaseSensorPublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/RRROS2BaseSensorComponent.h"

URRROS2BaseSensorPublisher::URRROS2BaseSensorPublisher()
{
    // TopicName could be overridden later by users
    TopicName = TEXT("sensor_data");
    MsgClass = UROS2GenericMsg::StaticClass();
}

void URRROS2BaseSensorPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (nullptr != DataSourceComponent && DataSourceComponent->bLastScanDataValid)
    {
        StartPublishTimer();
        DataSourceComponent->SetROS2Msg(InMessage);
    }
    else
    {
        StopPublishTimer();
    }
}
