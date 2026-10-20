// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ClockPublisher.h"

// UE
#include "GameFramework/GameStateBase.h"

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

    FDateTime Now = FDateTime::UtcNow();
    InitialSystemTimeSec = Now.ToUnixTimestamp();

    FString TempInputString;
    if (FParse::Value(FCommandLine::Get(), TEXT("START_ROSTIME_FROM_WALLTIME"), TempInputString)) {
        TempInputString = TempInputString.Replace(TEXT("="), TEXT(""));
        bStartROSTimeFromWalltime = TempInputString.ToBool();
        UE_LOG(LogTemp, Log, TEXT("START_ROSTIME_FROM_WALLTIME %s"), *TempInputString);
    }else{
        UE_LOG(LogTemp, Warning, TEXT("Failed to parse  !"));
    }
    

    return res;
}

FROSTime URRROS2ClockPublisher::GetROSTime() const
{
    // Noted: Elapsed time: time in seconds since world was brought up for play
    const UWorld* world = GetWorld();
    const AGameStateBase* gameState = world ? world->GetGameState() : nullptr;
    const double worldTimeSec = gameState ? gameState->GetServerWorldTimeSeconds() : 0.0;
    // Split in double: URRConversionUtils::FloatToROSStamp takes a float, which loses sub-millisecond precision
    // after ~1 h of sim time.
    FROSTime rosTime;
    rosTime.Sec = static_cast<int32>(FMath::FloorToDouble(worldTimeSec));
    rosTime.Nanosec = static_cast<uint32>(FMath::Clamp((worldTimeSec - rosTime.Sec) * 1e9, 0.0, 999999999.0));
    if (bStartROSTimeFromWalltime)
    {
        rosTime.Sec = rosTime.Sec + InitialSystemTimeSec;
    }
    return rosTime;
}

bool URRROS2ClockPublisher::Tick(float DeltaSeconds)
{
    if (GetWorld()->GetGameState())
    {
        // update msg
        FROSClock msg;
        msg.Clock = GetROSTime();
        // publish
        Publish<UROS2ClockMsg, FROSClock>(msg);
    }

    return true;
}
