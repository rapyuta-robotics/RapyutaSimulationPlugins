// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2ClockPublisher.h"

void UROS2ClockPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    UpdateDelegate.BindDynamic(this, &UROS2ClockPublisher::UpdateMessage);
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2ClockMsg::StaticClass();
    TopicName = TEXT("clock");
    PublicationFrequencyHz = 100;

    // [ROS2ClockPublisher] must be already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::ClockPub);
}

void UROS2ClockPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2ClockMsg>(InMessage)->Update(FApp::GetDeltaTime());
}
