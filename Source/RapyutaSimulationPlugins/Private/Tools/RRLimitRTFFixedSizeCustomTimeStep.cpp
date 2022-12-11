// Copyright Epic Games, Inc. All Rights Reserved.

#include "Tools/RRLimitRTFFixedSizeCustomTimeStep.h"

#include "Misc/App.h"
#include "Misc/ConfigCacheIni.h"

URRLimitRTFFixedSizeCustomTimeStep::URRLimitRTFFixedSizeCustomTimeStep(const FObjectInitializer& ObjectInitializer)
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
           TEXT("[URRLimitRTFFixedSizeCustomTimeStep][URRLimitRTFFixedSizeCustomTimeStep]: StepSize: %f, TargetRTFL %f"),
           StepSize,
           TargetRTF);

    LastPlatformTime = FPlatformTime::Seconds();
}

bool URRLimitRTFFixedSizeCustomTimeStep::Initialize(UEngine* InEngine)
{
    return true;
}

void URRLimitRTFFixedSizeCustomTimeStep::Shutdown(UEngine* InEngine)
{
    // Empty but implemented because it is PURE_VIRTUAL
}

bool URRLimitRTFFixedSizeCustomTimeStep::UpdateTimeStep(UEngine* InEngine)
{
    // Copies "CurrentPlatformTime" (used during the previous frame) in "LastTime"
    UpdateApplicationLastTime();
    WaitForSync();
    // false means that the Engine's TimeStep should NOT be performed.
    return false;
}

ECustomTimeStepSynchronizationState URRLimitRTFFixedSizeCustomTimeStep::GetSynchronizationState() const
{
    return ECustomTimeStepSynchronizationState::Synchronized;
}

float URRLimitRTFFixedSizeCustomTimeStep::GetStepSize() const
{
    return StepSize;
}

void URRLimitRTFFixedSizeCustomTimeStep::SetStepSize(const float InStepSize)
{
    float stepSize = InStepSize;
    if (InStepSize < 1e-10)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[URRLimitRTFFixedSizeCustomTimeStep][SetStepSize]: Given step size is too small. Set to 0.001"));
        stepSize = 0.001f;
    }

    if (TargetRTF < stepSize)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[URRLimitRTFFixedSizeCustomTimeStep][SetStepSize]: TargetRTF must be > StepSize."));
        return;
    }

    StepSize = stepSize;
    FApp::SetFixedDeltaTime(StepSize);
}

float URRLimitRTFFixedSizeCustomTimeStep::GetTargetRTF() const
{
    return TargetRTF;
}

void URRLimitRTFFixedSizeCustomTimeStep::SetTargetRTF(const float InTargetRTF)
{
    float targetRTF = InTargetRTF;
    if (targetRTF < 0)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[URRLimitRTFFixedSizeCustomTimeStep][SetTargetRTF]: TargetRTF must be > 0. Set to default value 1."));
        targetRTF = 1.f;
    }

    if (targetRTF < StepSize)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[URRLimitRTFFixedSizeCustomTimeStep][SetTargetRTF]: TargetRTF must be > StepSize."));
        return;
    }

    TargetRTF = targetRTF;
}

bool URRLimitRTFFixedSizeCustomTimeStep::WaitForSync()
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
