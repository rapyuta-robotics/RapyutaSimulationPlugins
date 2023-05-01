// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRROS2GameMode.h"

// UE
#include "HAL/PlatformMisc.h"

// rclUE
#include "Msgs/ROS2Clock.h"
#include "ROS2NodeComponent.h"

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
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Log,
                     TEXT("INIT GAME [%s/%s] - Options: %s\n%s"),
                     *InMapName,
                     *GetWorld()->GetName(),
                     *InOptions,
                     *OutErrorMessage);
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Log,
                     TEXT("NUM OF CPU CORES: [%d] - WITH HYPERTHREADS: [%d] - RECOMMENDED NUM OF WORKER THREADS: [%d]"),
                     FPlatformMisc::NumberOfCores(),
                     FPlatformMisc::NumberOfCoresIncludingHyperthreads(),
                     FPlatformMisc::NumberOfWorkerThreadsToSpawn());
    UE_LOG_WITH_INFO(
        LogRapyutaCore, Display, TEXT("ShouldUseThreadingForPerformance: %d"), FApp::ShouldUseThreadingForPerformance());

    // 1- Simulation state
    MainSimState = GetWorld()->SpawnActor<ASimulationState>();
    // 1.1- Register BP spawnable classes
    MainSimState->RegisterSpawnableBPEntities(BPSpawnableClassNames);
    // 1.2- Fetch Entities in the map first regardless of ROS2
    MainSimState->InitEntities();
}

void ARRROS2GameMode::InitSim()
{
    // 1 - Init Sim-wide Main ROS2 node, but only in case of a Network standalone app
    // For Server-client app, each client will have its own ROS2 Node inited upon Network player controller possessing
    if (IsNetMode(NM_Standalone) && (nullptr == Cast<ARRNetworkGameMode>(this)))
    {
        InitROS2();
    }
#if WITH_EDITOR    // Since ROSNode in each client is namespaced with editor in network mode, need clock publsiher without namespace
    else if (nullptr != Cast<ARRNetworkGameMode>(this))
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("Init ROS2 Node with editor in gamemode"));
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
    MainROS2Node = UROS2NodeComponent::CreateNewNode(this, MainROS2NodeName, TEXT("/"));

    // MainSimState
    if(MainSimState == nullptr)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Failed to create MainSimState."));
        return;
    }

    // MainROS2SimStateClient
    MainROS2SimStateClient = NewObject<URRROS2SimulationStateClient>(this, ROS2SimStateClientClass, TEXT("MainROS2SimStateClient"));
    MainROS2SimStateClient->Init(MainROS2Node);
    // NOTE: Inside [URRROS2SimulationStateClient], GameMode, which is server-only, is inaccessible
    MainROS2SimStateClient->ServerSimState = MainSimState;

    // Create Clock publisher
    ClockPublisher =
        CastChecked<URRROS2ClockPublisher>(MainROS2Node->CreatePublisherWithClass(URRROS2ClockPublisher::StaticClass()));

    // Signal [OnROS2Initialized]
    OnROS2Initialized.Broadcast();
}

void ARRROS2GameMode::StartPlay()
{
    Super::StartPlay();

    // Init Sim main components
    InitSim();

    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("START PLAY!"));
}

void ARRROS2GameMode::SetFixedTimeStep(const float InStepSize)
{
    auto ct = Cast<URRLimitRTFFixedSizeCustomTimeStep>(GEngine->GetCustomTimeStep());
    if (ct)
    {
        ct->SetStepSize(InStepSize);
    }
    FApp::SetUseFixedTimeStep(true);
    FApp::SetFixedDeltaTime(InStepSize);
}

float ARRROS2GameMode::GetFixedTimeStep() const
{
    return FApp::GetFixedDeltaTime();
}

void ARRROS2GameMode::SetTargetRTF(const float InTargetRTF)
{
    auto ct = Cast<URRLimitRTFFixedSizeCustomTimeStep>(GEngine->GetCustomTimeStep());
    if (ct)
    {
        ct->SetTargetRTF(InTargetRTF);
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Warning,
                         TEXT("CustomTimeStep Class needs to be URRLimitRTFFixedSizeCustomTimeStep. "
                              "Return 0."));
    }
}
float ARRROS2GameMode::GetTargetRTF() const
{
    float targetRTF = 0;
    auto ct = Cast<URRLimitRTFFixedSizeCustomTimeStep>(GEngine->GetCustomTimeStep());
    if (ct)
    {
        targetRTF = ct->GetTargetRTF();
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Warning,
                         TEXT("CustomTimeStep Class needs to be URRLimitRTFFixedSizeCustomTimeStep. "
                              "Return 0."));
    }
    return targetRTF;
}
