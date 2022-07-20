// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRGameMode.h"

// UE
#include "Engine/Engine.h"
#include "GameFramework/GameUserSettings.h"
#include "Serialization/AsyncPackageLoader.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRGameInstance.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRGameState.h"
#include "Core/RRPlayerController.h"
#include "Tools/RRROS2ClockPublisher.h"

ARRGameMode::ARRGameMode()
{
    GameStateClass = ARRGameState::StaticClass();
    PlayerControllerClass = ARRPlayerController::StaticClass();
}

void ARRGameMode::PreInitializeComponents()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore, Warning, TEXT("[ARRGameMode] PreInitializeComponents()"))
#endif
    Super::PreInitializeComponents();
}

void ARRGameMode::InitGameState()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore, Warning, TEXT("[ARRGameMode] InitGameState()"));
#endif
    Super::InitGameState();
}

void ARRGameMode::PrintSimConfig() const
{
    UE_LOG(LogRapyutaCore, Display, TEXT("SIM GAME MODE CONFIG -----------------------------"));
    UE_LOG(LogRapyutaCore, Display, TEXT("- bBenchmark: %d"), bBenchmark);
}

void ARRGameMode::PrintUEPreprocessors() const
{
    UE_LOG(LogRapyutaCore, Display, TEXT("[ARRGameMode]: UE PREPROCESSORS:"));
    UE_LOG(LogRapyutaCore, Display, TEXT("* [DO_CHECK] %s!"), URRCoreUtils::GetBoolPreprocessorText<DO_CHECK>());

    UE_LOG(LogRapyutaCore, Display, TEXT("* [STATS] %s!"), URRCoreUtils::GetBoolPreprocessorText<STATS>());

    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("* [CPUPROFILERTRACE_ENABLED] (for UE Insights) %s!"),
           URRCoreUtils::GetBoolPreprocessorText<CPUPROFILERTRACE_ENABLED>());

    UE_LOG(LogRapyutaCore, Display, TEXT("* [WITH_EDITOR] %s!"), URRCoreUtils::GetBoolPreprocessorText<WITH_EDITOR>());

    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("* [RHI_RAYTRACING] %s %d!"),
           URRCoreUtils::GetBoolPreprocessorText<RHI_RAYTRACING>(),
           IsRayTracingEnabled());

    UE_LOG(LogRapyutaCore, Display, TEXT("* [WITH_UNREALPNG] %s!"), URRCoreUtils::GetBoolPreprocessorText<WITH_UNREALPNG>());

    UE_LOG(LogRapyutaCore, Display, TEXT("* [WITH_UNREALJPEG] %s!"), URRCoreUtils::GetBoolPreprocessorText<WITH_UNREALJPEG>());

#if (!WITH_EDITOR)
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("- bAsyncLoadingThreadEnabled: %d"),
           FAsyncLoadingThreadSettings::Get().bAsyncLoadingThreadEnabled);
#endif

    // Fetch [-physxDispatcher]
    // Ref: UnrealEngine/Engine/Source/Runtime/Engine/Private/PhysicsEngine/PhysScene_PhysX.cpp:1812
    int8 physXDispatcherNum = 0;
    if (PhysSingleThreadedMode())
    {
        // UnrealEngine/Engine/Source/Runtime/Engine/Private/PhysicsEngine/PhysScene_PhysX.cpp:1519
        UE_LOG(LogTemp, Display, TEXT("PHYSICS RUNS IN GAME THREAD!"));
    }
    else if (URRCoreUtils::GetCommandLineArgumentValue<int8>(URRCoreUtils::CCMDLINE_ARG_INT_PHYSX_DISPATCHER_NUM,
                                                             physXDispatcherNum))
    {
        UE_LOG(LogTemp, Display, TEXT("PHYSICS RUNS IN [%d] THREADS!"), physXDispatcherNum);
    }
    else
    {
        UE_LOG(LogTemp, Display, TEXT("PHYSICS RUNS IN A SINGLE THREAD!"));
    }
}

void ARRGameMode::ConfigureSimInPlay()
{
    URRCoreUtils::ExecuteConsoleCommand(this, URRCoreUtils::CMD_AO_USE_HISTORY_DISABLE);

    for (auto& moduleNames : URRGameSingleton::SASSET_OWNING_MODULE_NAMES)
    {
        moduleNames.Value.AddUnique(RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME);
    }
}

void ARRGameMode::StartPlay()
{
    UE_LOG(LogRapyutaCore, Display, TEXT("[ARRGameMode]: START PLAY!"))

    PrintSimConfig();
    PrintUEPreprocessors();

    GameInstance = Cast<URRGameInstance>(GetGameInstance());
    verify(GameInstance);

#if !WITH_EDITOR
    FApp::SetBenchmarking(bBenchmark);
    // The frequency is actually up to the Sim map purpose, which is for AI training or for user-watchable visualization
    // Generally, it should be the same as ROS ClockPublisher's PublicationFrequencyHz
    // (NOTE) Could only use fixed time step in non-Editor mode
    FApp::SetUseFixedTimeStep(true);
    FApp::SetFixedDeltaTime(1 / ClockPublisher->PublicationFrequencyHz);

    // Run benchmark, will freeze game for a bit. Higher workscale increases time it takes to run tests (10 is default and should be
    // used for shipping builds)
    UGameUserSettings::GetGameUserSettings()->RunHardwareBenchmark();

    // Call after the benchmark to apply settings to running game
    UGameUserSettings::GetGameUserSettings()->ApplyHardwareBenchmarkResults();
#endif
    UE_LOG(LogRapyutaCore, Display, TEXT("Is bench marking: %d"), FApp::IsBenchmarking());
    UE_LOG(LogRapyutaCore, Display, TEXT("Use Fixed time step: %d %f"), FApp::UseFixedTimeStep(), FApp::GetFixedDeltaTime());

    // 1- CONFIGURE GAME GLOBALLY
    ConfigureSimInPlay();

    // 2- START SIM
    StartSim();

    // PROCEED WITH PARENT'S PLAY LATER ONCE STARTING SIM IS FINISHED!
    // Please DO NOT call [Super::StartPlay()] here!
}

void ARRGameMode::StartSim()
{
    // 1- LOAD [ImageWrapperModule]
    // This must be loaded this early for possible external image-based texture loading at Sim initialization!
    URRCoreUtils::LoadImageWrapperModule();

    // 2- LOAD SIM STATIC GLOBAL RESOURCES --
    URRGameSingleton::Get()->InitializeResources();

    // 3- START SIM ONCE RESOURCES ARE LOADED --
    //
    // The reason for this scheduled delegate is some essential operation, which facilitates sim startup activities like
    // asynchronous resource loading, could only run after this [ARRGameState::BeginPlay()] ends!
    URRCoreUtils::ScreenMsg(FColor::Yellow, TEXT("LOADING SIM RESOURCES.."), 10.f);
    GetWorld()->GetTimerManager().SetTimer(
        OwnTimerHandle, [this]() { TryStartingSim(); }, 1.f, true);
}

bool ARRGameMode::TryStartingSim()
{
    static FDateTime sBeginTime(FDateTime::UtcNow());

    // !NOTE: This method was scheduled to be run by BeginPlay()
    // 1 - WAIT FOR RESOURCE LOADING, in prep for Sim initialization
    UWorld* world = GetWorld();
    bool bResult = URRCoreUtils::CheckWithTimeOut(
        []() { return URRGameSingleton::Get()->HaveAllResourcesBeenLoaded(); },
        [this, world]()
        {
            // Clear the timer to avoid repeated call to the method
            URRCoreUtils::StopRegisteredTimer(world, OwnTimerHandle);
            UE_LOG(LogRapyutaCore, Fatal, TEXT("[ARRGameMode] DYNAMIC RESOURCES LOADING TIMEOUT -> SHUTTING DOWN THE SIM..."))
        },
        sBeginTime,
        ARRGameMode::SIM_START_TIMEOUT_SECS);

    // Either resources have yet to be fully loaded or time out!
    if (!bResult)
    {
        return false;
    }
    // Clear the timer to avoid repeated call to the method
    URRCoreUtils::StopRegisteredTimer(world, OwnTimerHandle);

    URRCoreUtils::ScreenMsg(FColor::Yellow, TEXT("ALL DYNAMIC RESOURCES LOADED!"), 10.f);
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("[ARRGameMode] ALL DYNAMIC RESOURCES LOADED! -> BRING UP THE SIM NOW... ========================"));

    // 1 - [GameState]::StartSim()
    auto gameState = GetGameState<ARRGameState>();
    if (!gameState)
    {
        UE_LOG(LogRapyutaCore, Fatal, TEXT("[ARRGameMode]::[StartSim] GAME STATE IS NULL!"))
        return false;
    }
    gameState->StartSim();
    UE_LOG(
        LogRapyutaCore, Display, TEXT("[ARRGameMode] SIM STARTED, SIM SCENE'S ACTORS ARE ACCESSIBLE NOW! ========================"))

    // 2- START PARENT'S PLAY, WHICH TRIGGER OTHERS PLAY FROM GAME STATE, PLAYER CONTROLLER, ETC.
    ARRROS2GameMode::StartPlay();
    return true;
}
