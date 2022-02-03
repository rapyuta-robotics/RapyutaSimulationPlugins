// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "RRROS2GameMode.h"

// UE
#include "HAL/PlatformMisc.h"

// rclUE
#include "Msgs/ROS2ClockMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2ClockPublisher.h"
#include "Tools/SimulationState.h"

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
    InitROS2();
}

void ARRROS2GameMode::InitROS2()
{
    if (IsValid(ROS2Node))
    {
        return;
    }

    UWorld* currentWorld = GetWorld();
    ROS2Node = currentWorld->SpawnActor<AROS2Node>();
    ROS2Node->Namespace.Reset();
    ROS2Node->Name = UENodeName;
    ROS2Node->Init();

    // Create Clock publisher
    ClockPublisher = NewObject<URRROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
    ClockPublisher->InitializeWithROS2(ROS2Node);

    // Simulation state
    SimulationState = currentWorld->SpawnActor<ASimulationState>();
    SimulationState->Init(ROS2Node);
}
