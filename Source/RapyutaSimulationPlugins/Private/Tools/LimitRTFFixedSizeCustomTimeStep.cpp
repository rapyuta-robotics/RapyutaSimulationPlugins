// Copyright Epic Games, Inc. All Rights Reserved.

#include "Tools/LimitRTFFixedSizeCustomTimeStep.h"

#include "Misc/App.h"
#include "Misc/ConfigCacheIni.h"

ULimitRTFFixedSizeCustomTimeStep::ULimitRTFFixedSizeCustomTimeStep(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    float frameRate = 100.f;
    if (GConfig->GetFloat(TEXT("/Script/Engine.Engine"), TEXT("FixedFrameRate"), frameRate, GEngineIni))
    {
        StepSize = 1.0 / frameRate;
    }
    FApp::SetUseFixedTimeStep(true);
    FApp::SetFixedDeltaTime(StepSize);

    GConfig->GetFloat(TEXT("/Script/Engine.Engine"), TEXT("TargetRTF"), TargetRTF, GEngineIni);
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("[ULimitRTFFixedSizeCustomTimeStep][ULimitRTFFixedSizeCustomTimeStep]: StepSize: %f, TargetRTFL %f"),
           StepSize,
           TargetRTF);

    LastPlatformTime = FPlatformTime::Seconds();
}

bool ULimitRTFFixedSizeCustomTimeStep::Initialize(UEngine* InEngine)
{
    return true;
}

void ULimitRTFFixedSizeCustomTimeStep::Shutdown(UEngine* InEngine)
{
    // Empty but implemented because it is PURE_VIRTUAL
}

bool ULimitRTFFixedSizeCustomTimeStep::UpdateTimeStep(UEngine* InEngine)
{
    // Copies "CurrentPlatformTime" (used during the previous frame) in "LastTime"
    UpdateApplicationLastTime();
    WaitForSync();
    // false means that the Engine's TimeStep should NOT be performed.
    return false;
}

ECustomTimeStepSynchronizationState ULimitRTFFixedSizeCustomTimeStep::GetSynchronizationState() const
{
    return ECustomTimeStepSynchronizationState::Synchronized;
}

float ULimitRTFFixedSizeCustomTimeStep::GetStepSize() const
{
    return StepSize;
}

void ULimitRTFFixedSizeCustomTimeStep::SetStepSize(const float InStepSize)
{
    float stepSize = InStepSize;
    if (InStepSize < 1e-10)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[ULimitRTFFixedSizeCustomTimeStep][SetStepSize]: Given step size is too small. Set to 0.001"));
        stepSize = 0.001f;
    }

    if (TargetRTF < stepSize)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[ULimitRTFFixedSizeCustomTimeStep][SetStepSize]: TargetRTF must be > StepSize."));
        return;
    }

    StepSize = stepSize;
}

float ULimitRTFFixedSizeCustomTimeStep::GetTargetRTF() const
{
    return TargetRTF;
}

void ULimitRTFFixedSizeCustomTimeStep::SetTargetRTF(const float InTargetRTF)
{
    float targetRTF = InTargetRTF;
    if (targetRTF < 0)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[ULimitRTFFixedSizeCustomTimeStep][SetTargetRTF]: TargetRTF must be > 0. Set to default value 1."));
        targetRTF = 1.f;
    }

    if (targetRTF < StepSize)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[ULimitRTFFixedSizeCustomTimeStep][SetTargetRTF]: TargetRTF must be > StepSize."));
        return;
    }

    TargetRTF = targetRTF;
}

bool ULimitRTFFixedSizeCustomTimeStep::WaitForSync()
{
    const double currentPlatformTime = FPlatformTime::Seconds();
    // const double LastTime = FApp::GetLastTime();

    // Calculate delta time
    double deltaRealTime = currentPlatformTime - LastPlatformTime;

    // Handle the unexpected case of a negative DeltaRealTime by forcing LastTime to CurrentPlatformTime.
    if (deltaRealTime < 0)
    {
        FApp::SetCurrentTime(currentPlatformTime);    // Necessary since we don't have direct access to FApp's LastTime
        FApp::UpdateLastTime();
        deltaRealTime = currentPlatformTime - FApp::GetLastTime();    // DeltaRealTime should be zero now, which will force a sleep
    }

    const double waitTime = FMath::Max(StepSize / TargetRTF - deltaRealTime, 0.0);

    double actualWaitTime = 0.0;
    {
        FSimpleScopeSecondsCounter ActualWaitTimeCounter(actualWaitTime);

        if (waitTime > 5.f / 1000.f)
        {
            FPlatformProcess::SleepNoStats(waitTime - 0.002f);
        }

        // Give up timeslice for remainder of wait time.
        const double WaitEndTime = LastPlatformTime + StepSize / TargetRTF;
        while (FPlatformTime::Seconds() < WaitEndTime)
        {
            FPlatformProcess::SleepNoStats(0.f);
        }
    }

    // Use fixed delta time and update time.
    FApp::SetDeltaTime(StepSize);
    FApp::SetIdleTime(actualWaitTime);
    FApp::SetCurrentTime(FApp::GetLastTime() + StepSize);

    LastPlatformTime = FPlatformTime::Seconds();

    return true;
}