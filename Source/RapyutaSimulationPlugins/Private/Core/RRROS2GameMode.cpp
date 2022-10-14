// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRROS2GameMode.h"

// UE
#include "HAL/PlatformMisc.h"

// rclUE
#include "Msgs/ROS2Clock.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRNetworkGameMode.h"
#include "Tools/RRGhostPlayerPawn.h"
#include "Tools/RRROS2ClockPublisher.h"

ARRROS2GameMode::ARRROS2GameMode()
{
    DefaultPawnClass = ARRGhostPlayerPawn::StaticClass();
}

void ARRROS2GameMode::InitGame(const FString& InMapName, const FString& InOptions, FString& OutErrorMessage)
{
    Super::InitGame(InMapName, InOptions, OutErrorMessage);
    UE_LOG(LogRapyutaCore,
           Log,
           TEXT("[ARRROS2GameMode] INIT GAME [%s/%s] - Options: %s\n%s"),
           *InMapName,
           *GetWorld()->GetName(),
           *InOptions,
           *OutErrorMessage);
    UE_LOG(LogRapyutaCore,
           Log,
           TEXT("NUM OF CPU CORES: [%d] - WITH HYPERTHREADS: [%d] - RECOMMENDED NUM OF WORKER THREADS: [%d]"),
           FPlatformMisc::NumberOfCores(),
           FPlatformMisc::NumberOfCoresIncludingHyperthreads(),
           FPlatformMisc::NumberOfWorkerThreadsToSpawn());
    UE_LOG(LogRapyutaCore, Display, TEXT("ShouldUseThreadingForPerformance: %d"), FApp::ShouldUseThreadingForPerformance());

    // Init Sim main components
    InitSim();
}

void ARRROS2GameMode::InitSim()
{
    // 1- Simulation state
    MainSimState = GetWorld()->SpawnActor<ASimulationState>();
    // Fetch Entities in the map first regardless of ROS2
    MainSimState->InitEntities();

    // 2 - Init Sim-wide Main ROS2 node, but only in case of a Network standalone app
    // For Server-client app, each client will have its own ROS2 Node inited upon Network player controller possessing
    if (IsNetMode(NM_Standalone) && (nullptr == Cast<ARRNetworkGameMode>(this)))
    {
        InitROS2();
    }
#if WITH_EDITOR    // Since ROSNode in each client is namespaced with editor in network mode, need clock publsiher without
                   // namespace
    else if (nullptr != Cast<ARRNetworkGameMode>(this))
    {
        UE_LOG(LogRapyutaCore, Display, TEXT("Init ROS2 Node with editor in gamemode"));
        InitROS2();
    }
#endif
}

void ARRROS2GameMode::InitROS2()
{
    if (IsValid(MainROS2Node))
    {
        return;
    }

    // MainROS2Node
    MainROS2Node = GetWorld()->SpawnActor<AROS2Node>();
    MainROS2Node->Namespace.Reset();
    MainROS2Node->Name = MainROS2NodeName;
    MainROS2Node->Init();

    // MainSimState
    check(MainSimState);

    // MainROS2SimStateClient
    MainROS2SimStateClient = NewObject<URRROS2SimulationStateClient>(this, ROS2SimStateClientClass, TEXT("MainROS2SimStateClient"));
    MainROS2SimStateClient->Init(MainROS2Node);
    // NOTE: Inside [URRROS2SimulationStateClient], GameMode, which is server-only, is inaccessible
    MainROS2SimStateClient->ServerSimState = MainSimState;

    // Create Clock publisher
    ClockPublisher = NewObject<URRROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
    ClockPublisher->InitializeWithROS2(MainROS2Node);
}

void ARRROS2GameMode::StartPlay()
{
    Super::StartPlay();
    
    UE_LOG(LogRapyutaCore, Display, TEXT("[ARRROS2GameMode]: START PLAY!"));

#if !WITH_EDITOR
    // The frequency is actually up to the Sim map purpose, which is for AI training or for user-watchable visualization
    // Generally, it should be the same as ROS ClockPublisher's PublicationFrequencyHz
    // (NOTE) Could only use fixed time step in non-Editor mode
    FApp::SetUseFixedTimeStep(true);
#endif
}

void ARRROS2GameMode::SetFixedTimestep(float InStepSize)
{
    if (InStepSize < 1e-10)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[ARRROS2GameMode][SetFixedTimestep]: Given step size is too small. Set to 0.001"));
        InStepSize = 0.001;
    }
    FApp::SetUseFixedTimeStep(true);
    FApp::SetFixedDeltaTime(InStepSize);
}

float ARRROS2GameMode::GetFixedTimestep()
{
    return FApp::GetFixedDeltaTime();
}
