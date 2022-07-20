// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRROS2GameMode.h"

// UE
#include "HAL/PlatformMisc.h"

// rclUE
#include "Msgs/ROS2ClockMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRNetworkGameMode.h"
#include "Tools/RRROS2ClockPublisher.h"

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
    // Init Sim-wide Main ROS2 node, but only in case of a Network standalone app
    // For Server-client app, each client will have its own ROS2 Node inited upon Network player controller possessing
    if (IsNetMode(NM_Standalone) && (nullptr == Cast<ARRNetworkGameMode>(this)))
    {
        InitROS2();
    }

    // Simulation state
    SimulationState = GetWorld()->SpawnActor<ASimulationState>(SimulationStateClass);
    SimulationState->InitEntities();
}

void ARRROS2GameMode::InitROS2()
{
    if (IsValid(MainROS2Node))
    {
        return;
    }

    MainROS2Node = GetWorld()->SpawnActor<AROS2Node>();
    MainROS2Node->Namespace.Reset();
    MainROS2Node->Name = MainROS2NodeName;
    MainROS2Node->Init();

    // Create Clock publisher
    ClockPublisher = NewObject<URRROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
    ClockPublisher->InitializeWithROS2(MainROS2Node);
}
