// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2StringPublisher.h"

// rclUE
#include "Msgs/ROS2StringMsg.h"

URRROS2StringPublisher::URRROS2StringPublisher()
{
    MsgClass = UROS2StringMsg::StaticClass();
    PublicationFrequencyHz = 1;
}

void URRROS2StringPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);
    Init(UROS2QoS::DynamicBroadcaster);
}

void URRROS2StringPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    FROSString msg;
    msg.Data = Message;
    CastChecked<UROS2StringMsg>(InMessage)->SetMsg(msg);
}
