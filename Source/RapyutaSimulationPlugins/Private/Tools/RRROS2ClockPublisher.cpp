// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ClockPublisher.h"

void URRROS2ClockPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2ClockMsg::StaticClass();
    TopicName = TEXT("clock");
    PublicationFrequencyHz = 100;

    // [ROS2ClockPublisher] must have been already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::ClockPub);
}

void URRROS2ClockPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    // Noted: Elapsed time: time in seconds since world was brought up for play    
    auto* gameState = GetWorld()->GetGameState();
    if (gameState)
    {
        CastChecked<UROS2ClockMsg>(InMessage)->Update(gameState->GetServerWorldTimeSeconds());
    }
}
