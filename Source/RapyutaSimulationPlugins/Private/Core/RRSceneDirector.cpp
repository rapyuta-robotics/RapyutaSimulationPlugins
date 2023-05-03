// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRSceneDirector.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameInstance.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameState.h"
#include "Core/RRMathUtils.h"
#include "Core/RRPlayerController.h"
#include "Core/RRThreadUtils.h"
#include "Core/RRUObjectUtils.h"

ARRSceneDirector::ARRSceneDirector()
{
    bSceneInitialized = false;
    bIsDataCollecting = false;
    bIsOperating = false;
}

bool ARRSceneDirector::Initialize()
{
    if (false == Super::Initialize())
    {
        return false;
    }
    URRCoreUtils::RegisterRepeatedExecution(
        this, InitializationTimerHandle, [this] { TryInitializeOperation(); }, 0.1f);

    // Run [TryInitializeOperation()] until succeeded!
    UE_LOG_WITH_SCENE_ID(LogRapyutaCore, Display, TEXT("[%s] TRYING TO INTIALIZE..."), *GetName());
    LastTimeStamp = FDateTime::UtcNow();
    return true;
}

void ARRSceneDirector::TryInitializeOperation()
{
    double elapsedTime = FTimespan(FDateTime::UtcNow() - LastTimeStamp).GetTotalSeconds();
    bool bIsTimeOut = elapsedTime > ARRGameMode::SIM_START_TIMEOUT_SECS;

    bSceneInitialized = URRCoreUtils::HasSimInitialized(this) && InitializeOperation();

    if (bSceneInitialized || bIsTimeOut)
    {
        if (bSceneInitialized)
        {
            UE_LOG_WITH_SCENE_ID(LogRapyutaCore,
                                 Display,
                                 TEXT("SCENE INITIALIZED! - TOOK [%lf] secs => ABOUT TO START "
                                      "OPERATION!============="),
                                 elapsedTime);
            URRCoreUtils::StopRegisteredExecution(GetWorld(), InitializationTimerHandle);
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Error,
                             TEXT("SCENE FAILED INITIALIZING! - TIMEOUT IS [%lf] secs"),
                             ARRGameMode::SIM_START_TIMEOUT_SECS);
            EndSceneInstance();
        }
    }
}

bool ARRSceneDirector::InitializeOperation()
{
    OperationBatchLoopLeft = RRGameState->OPERATION_BATCHES_NUM;
    OperationBatchId = 1;

    // Plugin common objects (which should be valid only after Sim has initialized) --
    ActorCommon = URRActorCommon::GetActorCommon(SceneInstanceId);

    // Camera
    SceneCamera = ActorCommon->SceneCamera;
    verify(SceneCamera);

    // PostProcessVolume
    MainPostProcessVolume = Cast<APostProcessVolume>(URRUObjectUtils::FindPostProcessVolume(GetWorld()));

    // Plan to run the main operation after Initializing is finished, in the next tick --
    URRCoreUtils::PlanToExecuteOnNextTick(GetWorld(), [this]() { RunOperation(); });

    return true;
}

void ARRSceneDirector::RunOperation()
{
    bIsOperating = true;
    SpawnActors();
}

bool ARRSceneDirector::HasOperationCompleted(bool bIsLogged)
{
    if (bIsLogged)
    {
        if (bIsDataCollecting)
        {
            UE_LOG_WITH_SCENE_ID(LogRapyutaCore, Display, TEXT("is still collecting data!"));
        }
        else if (bIsOperating)
        {
            UE_LOG_WITH_SCENE_ID(LogRapyutaCore, Display, TEXT("is still operating!"));
        }
    }
    return !(bIsDataCollecting || bIsOperating);
}

void ARRSceneDirector::OnDataCollectionPhaseDone(bool bIsFinalDataCollectingPhase)
{
    if (bIsFinalDataCollectingPhase)
    {
        bIsDataCollecting = false;

        // PROFILING --
        if (URRCoreUtils::IsSimProfiling())
        {
            // Measure the running of the previous data collection
            UE_LOG_WITH_SCENE_ID(LogRapyutaCore,
                                 Log,
                                 TEXT("DATA COLLECTION DONE - TOOK [%lf] secs!"),
                                 URRCoreUtils::GetElapsedTime(DataCollectionTimeStamp));
        }
    }
}

void ARRSceneDirector::ResetScene()
{
    ActorCommon->LatestCustomDepthStencilValue = 0;
    SceneEntityMaskValueList.Reset();
}

void ARRSceneDirector::EndSceneInstance()
{
    bIsOperating = false;

    // [EndSceneInstance()] is virtual, and also wait polling for Sim's completion, thus it must be run on another thread,
    // then end the Sim in GameThread upon the waiting return.
    UE_LOG_WITH_SCENE_ID(
        LogRapyutaCore, Display, TEXT("[%s] - Still collecting data %d - THE END!"), *SceneName, bIsDataCollecting);
}
