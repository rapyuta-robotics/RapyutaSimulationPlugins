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
#if WITH_EDITOR    // Since ROSNode in each client is namespaced with editor in network mode, need clock publsiher without namespace
    else if (nullptr != Cast<ARRNetworkGameMode>(this))
    {
        UE_LOG(LogRapyutaCore, Display, TEXT("[ARRROS2GameMode] Init ROS2 Node with editor in gamemode"));
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
    MainROS2Node = NewObject<UROS2NodeComponent>(this);
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
    MainROS2Node->AddPublisher(ClockPublisher);
}

void ARRROS2GameMode::StartPlay()
{
    Super::StartPlay();

    UE_LOG(LogRapyutaCore, Display, TEXT("[ARRROS2GameMode]: START PLAY!"));
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
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[ARRROS2GameMode][SetTargetRTF]: CustomTimeStep Class needs to be URRLimitRTFFixedSizeCustomTimeStep. "
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
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[ARRROS2GameMode][GetTargetRTF]: CustomTimeStep Class needs to be URRLimitRTFFixedSizeCustomTimeStep. "
                    "Return 0."));
    }
    return targetRTF;
}
