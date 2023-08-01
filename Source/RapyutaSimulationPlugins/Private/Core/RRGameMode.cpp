// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRGameMode.h"

// UE
#include "Engine/Engine.h"
#include "GameFramework/GameUserSettings.h"
#include "Misc/ConfigCacheIni.h"
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
#if RAPYUTA_USE_SCENE_DIRECTOR
    GameStateClass = ARRGameState::StaticClass();
    PlayerControllerClass = ARRPlayerController::StaticClass();
#endif
}

void ARRGameMode::PreInitializeComponents()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Verbose, TEXT("PreInitializeComponents()"))
#endif
    Super::PreInitializeComponents();
}

void ARRGameMode::InitGameState()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("InitGameState()"));
#endif
    Super::InitGameState();
}

void ARRGameMode::PrintSimConfig() const
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("SIM GAME MODE CONFIG -----------------------------"));
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("- bBenchmark: %d"), bBenchmark);
}

void ARRGameMode::PrintUEPreprocessors()
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("UE PREPROCESSORS:"));
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("* [DO_CHECK] %s!"), URRCoreUtils::GetBoolPreprocessorText<DO_CHECK>());

    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("* [STATS] %s!"), URRCoreUtils::GetBoolPreprocessorText<STATS>());

    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Display,
                     TEXT("* [CPUPROFILERTRACE_ENABLED] (for UE Insights) %s!"),
                     URRCoreUtils::GetBoolPreprocessorText<CPUPROFILERTRACE_ENABLED>());

    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("* [WITH_EDITOR] %s!"), URRCoreUtils::GetBoolPreprocessorText<WITH_EDITOR>());

    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Display,
                     TEXT("* [RHI_RAYTRACING] %s %d!"),
                     URRCoreUtils::GetBoolPreprocessorText<RHI_RAYTRACING>(),
                     IsRayTracingEnabled());

    UE_LOG_WITH_INFO(
        LogRapyutaCore, Display, TEXT("* [WITH_UNREALPNG] %s!"), URRCoreUtils::GetBoolPreprocessorText<WITH_UNREALPNG>());

    UE_LOG_WITH_INFO(
        LogRapyutaCore, Display, TEXT("* [WITH_UNREALJPEG] %s!"), URRCoreUtils::GetBoolPreprocessorText<WITH_UNREALJPEG>());

    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Display,
                     TEXT("* [USING_THREAD_SANITISER] %s!"),
                     URRCoreUtils::GetBoolPreprocessorText<USING_THREAD_SANITISER>());

#if (!WITH_EDITOR)
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Display,
                     TEXT("- bAsyncLoadingThreadEnabled: %d"),
                     FAsyncLoadingThreadSettings::Get().bAsyncLoadingThreadEnabled);
#endif

    // Physics [Single/Multi-threaded mode]
    if (PhysSingleThreadedMode())
    {
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("PHYSICS RUNS IN GAME THREAD!"));
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("PHYSICS RUNS IN A MULTI-THREADED MODE!"));
    }
}

void ARRGameMode::ConfigureSimInPlay()
{
    if (IsNetMode(NM_Standalone))
    {
        URRCoreUtils::ExecuteConsoleCommand(this, URRCoreUtils::CMD_AO_USE_HISTORY_DISABLE);
    }

    for (auto& moduleNames : URRGameSingleton::SASSET_OWNING_MODULE_NAMES)
    {
        moduleNames.Value.AddUnique(RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME);
    }
}

void ARRGameMode::StartPlay()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("START PLAY!"))
    PrintUEPreprocessors();
#endif
    PrintSimConfig();

    if (URRGameSingleton::Get() == nullptr)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("GameSingleton is not child class of URRGameSingleton."));
    }

#if !WITH_EDITOR
    FApp::SetBenchmarking(bBenchmark);

    // Run benchmark, will freeze game for a bit. Higher workscale increases time it takes to run tests (10 is default and should be
    // used for shipping builds)
    UGameUserSettings::GetGameUserSettings()->RunHardwareBenchmark();

    // Call after the benchmark to apply settings to running game
    UGameUserSettings::GetGameUserSettings()->ApplyHardwareBenchmarkResults();
#endif
    UE_LOG_WITH_INFO(LogRapyutaCore, Verbose, TEXT("Is bench marking: %d"), FApp::IsBenchmarking());
    UE_LOG_WITH_INFO(
        LogRapyutaCore, Verbose, TEXT("Use Fixed time step: %d %f"), FApp::UseFixedTimeStep(), FApp::GetFixedDeltaTime());

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
    auto* gameSingleton = URRGameSingleton::Get();
    if (gameSingleton)
    {
        gameSingleton->PrintSimConfig();
        gameSingleton->InitializeResources();
    }

    // 3- START SIM ONCE RESOURCES ARE LOADED --
    //
    // The reason for this scheduled delegate is some essential operation, which facilitates sim startup activities like
    // asynchronous resource loading, could only run after this [ARRGameState::BeginPlay()] ends!
    URRCoreUtils::ScreenMsg(FColor::Yellow, TEXT("LOADING SIM RESOURCES.."), 10.f);
    BeginTimeStampSec = URRCoreUtils::GetSeconds();
    GetWorld()->GetTimerManager().SetTimer(
        OwnTimerHandle, [this]() { TryStartingSim(); }, 1.f, true);
}

bool ARRGameMode::TryStartingSim()
{
    // !NOTE: This method was scheduled to be run by BeginPlay()
    // 1 - WAIT FOR RESOURCE LOADING, in prep for Sim initialization
    UWorld* world = GetWorld();

    auto* gameSingleton = URRGameSingleton::Get();
    if (gameSingleton)
    {
        bool bResult = URRCoreUtils::CheckWithTimeOut(
            [gameSingleton]() { return gameSingleton->HaveAllResourcesBeenLoaded(); },
            [this, world]()
            {
                // Clear the timer to avoid repeated call to the method
                URRCoreUtils::StopRegisteredTimer(world, OwnTimerHandle);
                UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("DYNAMIC RESOURCES LOADING TIMEOUT -> SHUTTING DOWN THE SIM..."))
            },
            BeginTimeStampSec,
            ARRGameMode::SIM_START_TIMEOUT_SECS);

        // Either resources have yet to be fully loaded or time out!
        if (!bResult)
        {
            return false;
        }
    }

    // Clear the timer to avoid repeated call to the method
    URRCoreUtils::StopRegisteredTimer(world, OwnTimerHandle);

    URRCoreUtils::ScreenMsg(FColor::Yellow, TEXT("ALL DYNAMIC RESOURCES LOADED!"), 10.f);
#if RAPYUTA_SIM_VERBOSE
    UE_LOG(LogRapyutaCore, Warning, TEXT("ALL DYNAMIC RESOURCES LOADED! -> BRING UP THE SIM NOW... ========================"));
#endif

    // 1 - [GameState]::StartSim()
    auto gameState = GetGameState<ARRGameState>();
    if (gameState)
    {
        gameState->StartSim();
        UE_LOG(LogRapyutaCore, Log, TEXT("SIM STARTED, GLOBAL ACTORS ARE ACCESSIBLE NOW! ========================"));
    }
    else
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Error, TEXT("GameState is NOT of [ARRGameState], as required to initialize global actors"));
    }

    // 2- START PARENT'S PLAY, WHICH TRIGGER OTHERS PLAY FROM GAME STATE, PLAYER CONTROLLER, ETC.
    ARRROS2GameMode::StartPlay();
    return true;
}

void ARRGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // These resources were initialized in [ARRGameMode::StartSim()]
    URRGameSingleton::Get()->FinalizeResources();

    Super::EndPlay(EndPlayReason);
}
