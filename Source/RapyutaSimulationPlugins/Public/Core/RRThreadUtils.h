// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#pragma once

// UE
#include "Async/Async.h"
#include "Async/AsyncWork.h"
#include "HAL/PlatformProcess.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RenderCommandFence.h"
#include "RenderingThread.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGameSingleton.h"

#include "RRThreadUtils.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRThreadUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    static bool IsInsideConstructor()
    {
        auto& ThreadContext = FUObjectThreadContext::Get();
        return (ThreadContext.IsInConstructor > 0);
    }

    static void Sleep(float Seconds)
    {
        FPlatformProcess::Sleep(Seconds);
    }

    static uint32 GetCurrentProcessId()
    {
        return FPlatformProcess::GetCurrentProcessId();
    }

    static uint32 GetCurrentThreadId()
    {
        return FPlatformTLS::GetCurrentThreadId();
    }
};
