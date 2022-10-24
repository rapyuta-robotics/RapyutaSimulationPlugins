// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRBaseSensorComponent.h"

URRBaseSensorComponent::URRBaseSensorComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void URRBaseSensorComponent::Initialize()
{
    // Start getting sensor data
    Run();
}

void URRBaseSensorComponent::Run()
{
    GetWorld()->GetTimerManager().SetTimer(
        TimerHandle, this, &URRBaseSensorComponent::SensorUpdate, 1.f / static_cast<float>(ScanFrequencyHz), true);
}
