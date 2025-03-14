// Copyright Epic Games, Inc. All Rights Reserved.

#include "Tools/RRLimitRTFFixedSizeCustomTimeStep.h"

// UE
#include "Misc/App.h"
#include "Misc/ConfigCacheIni.h"
#include "Stats/StatsMisc.h"

// rclUE
#include "logUtilities.h"

// RapyutaSimulationPlugins
#include "RapyutaSimulationPlugins.h"

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
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("StepSize: %f, TargetRTFL %f"), StepSize, TargetRTF);

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
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Given step size is too small. Set to 0.001"));
        stepSize = 0.001f;
    }

    if (TargetRTF < stepSize)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("TargetRTF must be > StepSize."));
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
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("TargetRTF must be > 0. Set to default value 1."));
        targetRTF = 1.f;
    }

    if (targetRTF < StepSize)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("TargetRTF must be > StepSize."));
        return;
    }

    TargetRTF = targetRTF;
}

bool URRLimitRTFFixedSizeCustomTimeStep::WaitForSync()
{
    const double currentPlatformTime = FPlatformTime::Seconds();
    double deltaRealTime = currentPlatformTime - LastPlatformTime;

    // Handle the unexpected case of a negative DeltaRealTime by forcing LastTime to CurrentPlatformTime.
    if (deltaRealTime < 0)
    {
        FApp::SetCurrentTime(currentPlatformTime);    // Necessary since we don't have direct access to FApp's LastTime
        FApp::UpdateLastTime();
        deltaRealTime = 0.0;    // DeltaRealTime should be zero now, which will force a sleep
    }

     // Calculate the remaining time to maintain the target RTF
    const double desiredStepTime = StepSize / TargetRTF;
    double remainingWaitTime = FMath::Max(desiredStepTime - deltaRealTime, 0.0);

    double actualWaitTime = 0.0;
    {
        FSimpleScopeSecondsCounter ActualWaitTimeCounter(actualWaitTime);

        // If there's significant remaining time, sleep for most of it
        constexpr double MinSleepThreshold = 0.002;  // 2ms
        constexpr double SleepSafetyMargin = 0.0005;  // 0.5ms margin for finer adjustment

        if (remainingWaitTime > MinSleepThreshold)
        {
            FPlatformProcess::SleepNoStats(remainingWaitTime - MinSleepThreshold);
        }

        // Fine-tune waiting for precise synchronization
        const double waitEndTime = LastPlatformTime + desiredStepTime;
        while (FPlatformTime::Seconds() < waitEndTime)
        {
            FPlatformProcess::SleepNoStats(SleepSafetyMargin);  // Avoid busy waiting, sleep for 0.5ms
        }

    }

    // Use fixed delta time and update time.
    FApp::SetDeltaTime(StepSize);
    FApp::SetIdleTime(actualWaitTime);
    FApp::SetCurrentTime(FApp::GetLastTime() + StepSize);

    LastPlatformTime = FPlatformTime::Seconds();

    return true;
}
