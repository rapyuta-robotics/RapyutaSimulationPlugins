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

    // ----------------------------------------------------------------------------------------------------------
    // [ASYNC TASK SERVICES] --
    //
    // https://stackoverflow.com/questions/47496358/c-lambdas-how-to-capture-variadic-parameter-pack-from-the-upper-scope
    // UE has equivalent ones of [std::forward] as Forward, [std::make_tuple] as MakeTuple,
    // but does not have one for [std::apply] :(
    template<typename FuncType, typename... ArgTypes>
    static void DoTaskInGameThread(FuncType&& TaskInGameThread, ArgTypes&&... Args)
    {
        if (IsInGameThread())
        {
            Forward<FuncType>(TaskInGameThread)(Forward<ArgTypes>(Args)...);
        }
        else
        {
            AsyncTask(ENamedThreads::GameThread,
                      [TaskInGameThread = Forward<FuncType>(TaskInGameThread), Args = MakeTuple(Forward<ArgTypes>(Args)...)]()
                      {
                          // This function is equivalent to [std::apply], which is only available since C++17!
                          Args.ApplyBefore(TaskInGameThread);
                      });    // AsyncTask(ENamedThreads::GameThread,...
        }
    }

    template<typename FuncType, typename... ArgTypes>
    static void DoTaskInGameThreadLater(FuncType&& TaskInGameThread, float InWaitingTime, ArgTypes&&... Args)
    {
        DoAsyncTaskInThread<void>(
            [InWaitingTime]()
            {
                // Wait for the physics to complete the computation given earlier vel cmds
                FPlatformProcess::Sleep(InWaitingTime);
            },
            [TaskInGameThread = Forward<FuncType>(TaskInGameThread)]() { DoTaskInGameThread(TaskInGameThread); });
    }

    template<typename TResult>
    static auto DoAsyncTaskInThread(TFunction<TResult()> Task,
                                    TFunction<void()> CompletionCallback,
                                    const EAsyncExecution ExecutionThread =
#if WITH_EDITOR
                                        EAsyncExecution::LargeThreadPool
#else
                                        EAsyncExecution::ThreadPool
#endif
    )
    {
        return Async(ExecutionThread,
                     MoveTemp(Task),
                     TUniqueFunction<void()>([CompletionCallback = MoveTemp(CompletionCallback)]() { CompletionCallback(); }));
    }

    template<typename TResult>
    static auto AddAsyncTaskInThreadPool(FRRAsyncJob& OutAsyncJob,
                                         const uint64& InCurrentCaptureBatchId,
                                         TFunction<TResult()> Task,
                                         TFunction<void()> CompletionCallback)
    {
        UE_LOG(LogTemp,
               Verbose,
               TEXT("[%ld:%s] ASYNC JOB NUM: %d"),
               InCurrentCaptureBatchId,
               *OutAsyncJob.JobName,
               OutAsyncJob.GetTasksNum());
        OutAsyncJob.AddAsyncTask(InCurrentCaptureBatchId, DoAsyncTaskInThread(MoveTemp(Task), MoveTemp(CompletionCallback)));
    }

    template<typename TAsyncTask>
    static void EnsureAsyncTasksCompletion(const TArray<TUniquePtr<FAsyncTask<TAsyncTask>>>& AsyncTasks)
    {
        for (auto& task : AsyncTasks)
        {
            if (!task->Cancel())
            {
                task->EnsureCompletion();
            }
        }
    }

    template<typename TAsyncTask>
    static void RefreshUpdateAsyncTasks(TArray<TUniquePtr<FAsyncTask<TAsyncTask>>>& AsyncTasks)
    {
        for (int8 i = 0; i < AsyncTasks.Num(); ++i)
        {
            if (AsyncTasks[i]->IsDone())
            {
                AsyncTasks.RemoveAt(i);
            }
        }
    }

    static bool AreGraphTasksRunningInThread(const ENamedThreads::Type ThreadType)
    {
        return FTaskGraphInterface::Get().IsThreadProcessingTasks(ThreadType);
    }

    static void WaitUntilAllGraphTasksCompleteInThread(const ENamedThreads::Type ThreadType)
    {
        DECLARE_CYCLE_STAT(TEXT("FlushGraphTasks"), STAT_FlushGraphTasksInThread, STATGROUP_TaskGraphTasks);
        FGraphEventRef FenceHandle = FSimpleDelegateGraphTask::CreateAndDispatchWhenReady(
            FSimpleDelegateGraphTask::FDelegate(), GET_STATID(STAT_FlushGraphTasksInThread), nullptr, ThreadType);
        FTaskGraphInterface::Get().WaitUntilTaskCompletes(FenceHandle, ThreadType);
    }

    // ----------------------------------------------------------------------------------------------------------
    // [RENDERING THREAD SERVICES] --
    // https://docs.unrealengine.com/en-US/Programming/Rendering/ThreadedRendering/index.html
    //
    static FRenderCommandFence SRenderCommandFence;
    static void SyncWithRenderingThread(bool bSyncToRHIAndGPU)
    {
        SRenderCommandFence.BeginFence(bSyncToRHIAndGPU);
        SRenderCommandFence.Wait();
    }

    static void WaitForRenderingThreadFinished()
    {
        // Blockthe game thread until the rendering thread has caught up.
        // This is useful for offline (editor) operations which modify memory being accessed by the rendering thread.
        FlushRenderingCommands();
    }
};
