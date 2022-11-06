// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2StringPublisher.h"

// rclUE
#include "Msgs/ROS2Str.h"

URRROS2StringPublisher::URRROS2StringPublisher()
{
    MsgClass = UROS2StrMsg::StaticClass();
    PublicationFrequencyHz = 1;
    QoS = UROS2QoS::DynamicBroadcaster;
}

void URRROS2StringPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    FROSStr msg;
    msg.Data = Message;
    CastChecked<UROS2StrMsg>(InMessage)->SetMsg(msg);
}
