// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2StringPublisher.h"

// rclUE
#include "Msgs/ROS2StringMsg.h"

void URRROS2StringPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2StringMsg::StaticClass();
    PublicationFrequencyHz = 1;
    Init(UROS2QoS::DynamicBroadcaster);
}

void URRROS2StringPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2StringMsg>(InMessage)->Update(Message);
}
