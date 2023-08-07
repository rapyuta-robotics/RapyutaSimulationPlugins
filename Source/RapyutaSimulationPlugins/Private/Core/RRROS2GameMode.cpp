// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Core/RRROS2GameMode.h"

// UE
#include "HAL/PlatformMisc.h"

// rclUE
#include "Msgs/ROS2Clock.h"
#include "ROS2NodeComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRAssetUtils.h"
#include "Core/RRNetworkGameMode.h"
#include "Core/RRTypeUtils.h"
#include "Robots/Turtlebot3/TurtlebotBurger.h"
#include "Robots/Turtlebot3/TurtlebotBurgerVehicle.h"
#include "Tools/RRGhostPlayerPawn.h"
#include "Tools/RRROS2ClockPublisher.h"

ARRROS2GameMode::ARRROS2GameMode()
{
    DefaultPawnClass = ARRGhostPlayerPawn::StaticClass();
}

void ARRROS2GameMode::PrintSimConfig() const
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("ROS2 GAME MODE CONFIG -----------------------------"));
    UE_LOG(LogRapyutaCore, Display, TEXT("BPSpawnableClassNames:"));
    for (const auto& bpSpawnableClassName : BPSpawnableClassNames)
    {
        UE_LOG(LogRapyutaCore, Display, TEXT("- %s"), *bpSpawnableClassName);
    }
    UE_LOG(LogRapyutaCore, Display, TEXT("NativeSpawnableClassPaths:"));
    for (const auto& [entityModelName, nativeSpawnableClassPath] : NativeSpawnableClassPaths)
    {
        UE_LOG(LogRapyutaCore, Display, TEXT("- [%s]: %s"), *entityModelName, *nativeSpawnableClassPath);
    }
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
    UE_LOG(LogRapyutaCore,
           Log,
           TEXT("NUM OF CPU CORES: [%d] - WITH HYPERTHREADS: [%d] - RECOMMENDED NUM OF WORKER THREADS: [%d]"),
           FPlatformMisc::NumberOfCores(),
           FPlatformMisc::NumberOfCoresIncludingHyperthreads(),
           FPlatformMisc::NumberOfWorkerThreadsToSpawn());
    UE_LOG(LogRapyutaCore, Display, TEXT("ShouldUseThreadingForPerformance: %d"), FApp::ShouldUseThreadingForPerformance());

    // 1- Simulation state
    MainSimState = GetWorld()->SpawnActor<ASimulationState>();

    // 1.1- Register BP spawnable classes
    MainSimState->RegisterSpawnableBPEntities(BPSpawnableClassNames);

    // 1.2- Register native spawnable classes
    TMap<FString /*EntityModelName*/, TSubclassOf<AActor>> nativeSpawnableClasses;
    for (const auto& [entityModelName, nativeSpawnableClassPath] : NativeSpawnableClassPaths)
    {
        UClass* entityClass = URRAssetUtils::FindClassFromPathName(nativeSpawnableClassPath);
        if (entityClass)
        {
            nativeSpawnableClasses.Add({entityModelName, entityClass});
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Error,
                             TEXT("[%s] Failed to find class from path [%s]"),
                             *entityModelName,
                             *nativeSpawnableClassPath);
        }
    }
    MainSimState->AddSpawnableEntityTypes(nativeSpawnableClasses);

    // 1.3- Fetch Entities in the map first regardless of ROS2
    MainSimState->InitEntities();
}

void ARRROS2GameMode::InitSim()
{
    // 1 - Init Sim-wide Main ROS 2 node, but only in case of a Network standalone app
    // For Server-client app, each client will have its own ROS 2 Node inited upon Network player controller possessing
    if (IsNetMode(NM_Standalone) && (nullptr == Cast<ARRNetworkGameMode>(this)))
    {
        InitROS2();
    }
#if WITH_EDITOR    // Since ROSNode in each client is namespaced with editor in network mode, need clock publsiher without namespace
    else if (nullptr != Cast<ARRNetworkGameMode>(this))
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("Init ROS 2 Node with editor in gamemode"));
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
    if (MainSimState == nullptr)
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
    UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Display, TEXT("Fixed Timestep Updated: %f"), InStepSize);
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
        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Display, TEXT("Custom RTF Updated: %f"), InTargetRTF);
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
