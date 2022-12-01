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

void ARRSceneDirector::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!bSceneInitialized)
    {
        TryInitializeOperation();
    }
}

bool ARRSceneDirector::Initialize()
{
    if (false == Super::Initialize())
    {
        return false;
    }
    SetTickEnabled(true);

    // Run [TryInitializeOperation()] until succeeded!
    UE_LOG(LogRapyutaCore, Display, TEXT("[%d:SCENE DIRECTOR]::[%s] TRYING TO INTIALIZE..."), SceneInstanceId, *GetName());
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
            UE_LOG(LogRapyutaCore,
                   Display,
                   TEXT("[%d:ARRSceneDirector] SCENE INITIALIZED! - TOOK [%lf] secs => ABOUT TO START "
                        "OPERATION!============="),
                   SceneInstanceId,
                   elapsedTime);
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("[%d:ARRSceneDirector] SCENE FAILED INITIALIZING! - TIMEOUT IS [%lf] secs"),
                   SceneInstanceId,
                   ARRGameMode::SIM_START_TIMEOUT_SECS);
            EndSceneInstance();
        }
    }
}

bool ARRSceneDirector::InitializeOperation()
{
    OperationBatchLoopLeft = GameState->OPERATION_BATCHES_NUM;
    OperationBatchId = 1;

    // Plugin common objects (which should be valid only after Sim has initialized) --
    ActorCommon = URRActorCommon::GetActorCommon(SceneInstanceId);

    // Camera
    MainCamera = ActorCommon->MainCamera;
    verify(MainCamera);

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
            UE_LOG(LogRapyutaCore, Display, TEXT("SceneInstance[%d] is still collecting data!"), SceneInstanceId);
        }
        else if (bIsOperating)
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("SceneInstance[%d] is still operating!"), SceneInstanceId);
        }
    }
    return !(bIsDataCollecting || bIsOperating);
}

void ARRSceneDirector::OnDataCollectionPhaseDone(bool bIsFinalDataCollectingPhase)
{
    if (bIsFinalDataCollectingPhase)
    {
        bIsDataCollecting = false;
    }
}

void ARRSceneDirector::ResetScene()
{
    GameState->SetAllEntitiesActivated(false);
}

void ARRSceneDirector::EndSceneInstance()
{
    bIsOperating = false;

    // [EndSceneInstance()] is virtual, and also wait polling for Sim's completion, thus it must be run on another thread,
    // then end the Sim in GameThread upon the waiting return.
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("SCENE INSTANCE (%d) [%s] - Still collecting data %d - THE END!"),
           SceneInstanceId,
           *SceneName,
           bIsDataCollecting);
}
