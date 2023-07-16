// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ClockPublisher.h"

// rclUE
#include "Msgs/ROS2Clock.h"
#include "rclcUtilities.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

URRROS2ClockPublisher::URRROS2ClockPublisher()
{
    MsgClass = UROS2ClockMsg::StaticClass();
    TopicName = TEXT("clock");

    // Frequency should be same as Game fixed timestep but
    // Since Timer frequency <= fixed timestep causes un precise loop, uses tick.
    PublicationFrequencyHz = -1;

    // [ROS2ClockPublisher] must have been already registered to [InROS2Node] (in Super::) before being initialized
    QoS = UROS2QoS::ClockPub;
}

bool URRROS2ClockPublisher::Init()
{
    bool res = Super::Init();
    TickDelegate = FTickerDelegate::CreateUObject(this, &URRROS2ClockPublisher::Tick);
    TickDelegateHandle = FTSTicker::GetCoreTicker().AddTicker(TickDelegate);

    return res;
}

bool URRROS2ClockPublisher::Tick(float DeltaSeconds)
{
    // Noted: Elapsed time: time in seconds since world was brought up for play
    auto* gameState = GetWorld()->GetGameState();
    if (gameState)
    {
        // update msg
        FROSClock msg;
        msg.Clock = URRConversionUtils::FloatToROSStamp(gameState->GetServerWorldTimeSeconds());

        // publish
        Publish<UROS2ClockMsg, FROSClock>(msg);
    }

    return true;
}
