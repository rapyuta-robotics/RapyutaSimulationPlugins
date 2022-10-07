// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ClockPublisher.h"
#include "rclcUtilities.h"

void URRROS2ClockPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2ClockMsg::StaticClass();
    TopicName = TEXT("clock");

    // Frequency should be same as Game fixed timestep but 
    // Since Timer frequency <= fixed timestep causes un precise loop, uses tick.
    PublicationFrequencyHz = -1;

    // [ROS2ClockPublisher] must have been already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::ClockPub);
}

void URRROS2ClockPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    // Noted: Elapsed time: time in seconds since world was brought up for play    
    auto* gameState = GetWorld()->GetGameState();
    if (gameState)
    {

        FROSClock msg;
        msg.Clock = URRConversionUtils::FloatToROSStamp(gameState->GetServerWorldTimeSeconds());
        CastChecked<UROS2ClockMsg>(InMessage)->SetMsg(msg);
    }
}

void URRROS2ClockPublisher::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    UpdateAndPublishMessage();
}
