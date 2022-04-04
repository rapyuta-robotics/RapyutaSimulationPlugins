/**
 * @file RRThreadUtils.h
 * @brief UE type related utils
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

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

    /**
     * @brief return true if called inside constructor.
     * 
     * @return true 
     * @return false 
     */
    static bool IsInsideConstructor()
    {
        auto& ThreadContext = FUObjectThreadContext::Get();
        return (ThreadContext.IsInConstructor > 0);
    }

    // ----------------------------------------------------------------------------------------------------------
    // [ASYNC TASK SERVICES] --
    //
    /**
     * @brief Used in #URRProceduralMeshComponent
     * UE has equivalent ones of [std::forward] as Forward, [std::make_tuple] as MakeTuple,
     * but does not have one for [std::apply] :(
     * @sa https://stackoverflow.com/questions/47496358/c-lambdas-how-to-capture-variadic-parameter-pack-from-the-upper-scope
     */
    template<typename TFunc, typename... TArgs>
    static void DoTaskInGameThread(TFunc&& InTaskInGameThread, TArgs&&... InArgs)
    {
        if (IsInGameThread())
        {
            Forward<TFunc>(InTaskInGameThread)(Forward<TArgs>(InArgs)...);
        }
        else
        {
            AsyncTask(ENamedThreads::GameThread,
                      [InTaskInGameThread = Forward<TFunc>(InTaskInGameThread), InArgs = MakeTuple(Forward<TArgs>(InArgs)...)]()
                      {
                          // This function is equivalent to [std::apply], which is only available since C++17!
                          InArgs.ApplyBefore(InTaskInGameThread);
                      });
        }
    }

    template<typename TFunc, typename... TArgs>
    static void DoTaskInGameThreadLater(TFunc&& InTaskInGameThread, float InWaitingTime)
    {
        DoAsyncTaskInThread<void>(
            [InWaitingTime]()
            {
                // Wait for the physics to complete the computation given earlier vel cmds
                FPlatformProcess::Sleep(InWaitingTime);
            },
            [InTaskInGameThread = Forward<TFunc>(InTaskInGameThread)]() { DoTaskInGameThread(InTaskInGameThread); });
    }

    template<typename TResult>
    static auto DoAsyncTaskInThread(TFunction<TResult()> InTask,
                                    TFunction<void()> InCompletionCallback,
                                    const EAsyncExecution InExecutionThread =
#if WITH_EDITOR
                                        EAsyncExecution::LargeThreadPool
#else
                                        EAsyncExecution::ThreadPool
#endif
    )
    {
        return Async(
            InExecutionThread,
            MoveTemp(InTask),
            TUniqueFunction<void()>([InCompletionCallback = MoveTemp(InCompletionCallback)]() { InCompletionCallback(); }));
    }
    template<typename TResult>
    static auto AddAsyncTaskInThreadPool(FRRAsyncJob& OutAsyncJob,
                                         const uint64& InCurrentCaptureBatchId,
                                         TFunction<TResult()> InTask,
                                         TFunction<void()> InCompletionCallback)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogTemp,
               Warning,
               TEXT("[%ld:%s] ASYNC JOB NUM: %d"),
               InCurrentCaptureBatchId,
               *OutAsyncJob.JobName,
               OutAsyncJob.GetTasksNum());
#endif
        OutAsyncJob.AddAsyncTask(InCurrentCaptureBatchId, DoAsyncTaskInThread(MoveTemp(InTask), MoveTemp(InCompletionCallback)));
    }
};
