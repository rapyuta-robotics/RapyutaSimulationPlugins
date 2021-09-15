// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "RRGameMode.h"

// UE
#include "Engine/Engine.h"
#include "HAL/PlatformMisc.h"

// RapyutaSim
#include "RRGameSingleton.h"
#include "Tools/RRGeneralUtils.h"

ARRGameMode::ARRGameMode()
{
    // GameState and PlayerControllerClass could be configured herein. For example:
    // GameStateClass = ARRGameState::StaticClass();
    // PlayerControllerClass = ARRPlayerController::StaticClass();
}

void ARRGameMode::StartPlay()
{
    UE_LOG(LogRapyutaCore, Display, TEXT("[ARRGameMode]: START PLAY!"))
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("NUM OF CPU CORES: [%d] - WITH HYPERTHREADS: [%d] - RECOMMENDED NUM OF WORKER THREADS: [%d]"),
           FPlatformMisc::NumberOfCores(),
           FPlatformMisc::NumberOfCoresIncludingHyperthreads(),
           FPlatformMisc::NumberOfWorkerThreadsToSpawn());
    UE_LOG(LogRapyutaCore, Display, TEXT("ShouldUseThreadingForPerformance: %d"), FApp::ShouldUseThreadingForPerformance());

    // 1- LOAD SIM RESOURCES --
    // Initialize global resources loading
    URRGameSingleton::Get()->InitializeResources();

    // 2- START SIM ONLY ONCE RESOURCES HAVE BEEN LOADED --
    GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Yellow, TEXT("LOADING SIM DYNAMIC RESOURCES.."));
    GetWorld()->GetTimerManager().SetTimer(
        OwnTimerHandle, [this]() { TryStartingSim(); }, 1.f, true);

    // Proceed with Super's Play later once Sim's start finishes.
    // Thus, [Super::StartPlay()] is not called here!
}

void ARRGameMode::TryStartingSim()
{
    static FDateTime sBeginTime(FDateTime::UtcNow());

    // !NOTE: This method was scheduled to be run by BeginPlay()
    // 1 - WAIT FOR RESOURCE LOADING, in prep for Sim mode initialization
    UWorld* world = GetWorld();
    bool bResult = URRGeneralUtils::CheckWithTimeOut(
        []() { return URRGameSingleton::Get()->HaveAllResourcesBeenLoaded(); },
        [this, world]()
        {
            // Clear the timer to avoid repeated call to the method
            URRGeneralUtils::StopRegisteredTimer(world, OwnTimerHandle);
            UE_LOG(LogRapyutaCore, Fatal, TEXT("[ARRGameMode] DYNAMIC RESOURCES LOADING TIMEOUT -> SHUTTING DOWN THE SIM..."))
        },
        sBeginTime,
        SIM_START_TIMEOUT_SECS);

    // Either resources have yet to be fully loaded or time out!
    if (!bResult)
    {
        return;
    }
    // Clear the timer to avoid repeated call to the method
    URRGeneralUtils::StopRegisteredTimer(world, OwnTimerHandle);

    GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Yellow, TEXT("ALL DYNAMIC RESOURCES LOADED!"));
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("[ARRGameMode] ALL DYNAMIC RESOURCES LOADED! -> BRING UP THE SIM NOW... ========================"));

    // 2 - START PARENT'S PLAY, WHICH TRIGGERS THE BEGIN-PLAY OF GAME STATE, PLAYER CONTROLLER, ETC.
    Super::StartPlay();
}
