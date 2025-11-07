// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Core/RRROS2GameMode.h"

// UE
#include "HAL/PlatformMisc.h"
#include "Engine/World.h"
#include "TimerManager.h"

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

ARRROS2GameMode::~ARRROS2GameMode()
{
    // Clean up RTF logging timer
    if (RTFLoggingTimerHandle.IsValid() && IsValid(GetWorld()))
    {
        GetWorld()->GetTimerManager().ClearTimer(RTFLoggingTimerHandle);
    }
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
    for (auto It = NativeSpawnableClassPaths.CreateConstIterator(); It; ++It)
    {
        UE_LOG(LogRapyutaCore, Display, TEXT("- [%s]: %s"), *It.Key(), *It.Value());
    }
    
    // Print RTF configuration
    UE_LOG(LogRapyutaCore, Display, TEXT("RTF Configuration:"));
    UE_LOG(LogRapyutaCore, Display, TEXT("- Fixed TimeStep: %.4fs"), GetFixedTimeStep());
    UE_LOG(LogRapyutaCore, Display, TEXT("- Target RTF: %.3f"), GetTargetRTF());
    if (bRTFCalculationInitialized)
    {
        UE_LOG(LogRapyutaCore, Display, TEXT("- Current RTF: %.3f"), GetCurrentRTF());
    }
    UE_LOG(LogRapyutaCore, Display, TEXT("- RTF Logging: %s (Interval: %.2fs)"), 
           bRTFLoggingEnabled ? TEXT("Enabled") : TEXT("Disabled"), RTFLoggingInterval);
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
    for (auto It = NativeSpawnableClassPaths.CreateConstIterator(); It; ++It)
    {
        const FString& entityModelName = It.Key();
        const FString& nativeSpawnableClassPath = It.Value();
        UClass* entityClass = URRAssetUtils::FindClassFromPathName(nativeSpawnableClassPath);
        if (entityClass)
        {
            nativeSpawnableClasses.Add(entityModelName, entityClass);
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
    if (nativeSpawnableClasses.Num() > 0)
    {
        MainSimState->AddSpawnableEntityTypes(MoveTemp(nativeSpawnableClasses));
    }

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

    // Initialize RTF calculation
    ResetRTFCalculation();

    // Enable RTF logging by default (can be disabled later if needed)
    // Set logging interval to match window size to avoid duplicate logging
    SetRTFLogging(true, RTFWindowSize);

    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("START PLAY! RTF calculation initialized."));
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

float ARRROS2GameMode::GetCurrentRTF() const
{
    // Initialize RTF calculation if not done yet
    if (!bRTFCalculationInitialized)
    {
        RTFWindowStartTime = FPlatformTime::Seconds();
        RTFWindowStartSimTime = FApp::GetCurrentTime();
        LastCalculatedRTF = 0.0f;
        bRTFCalculationInitialized = true;
        return 0.0f; // Return 0 for the first frame
    }

    // Get current times
    const double currentRealTime = FPlatformTime::Seconds();
    const double currentSimTime = FApp::GetCurrentTime();
    
    // Check if enough time has passed for the window
    const double elapsedRealTime = currentRealTime - RTFWindowStartTime;
    
    // If we haven't reached the window size yet, return the last calculated value
    if (elapsedRealTime < RTFWindowSize)
    {
        return LastCalculatedRTF;
    }
    
    // Calculate RTF over the window period
    const double elapsedSimTime = currentSimTime - RTFWindowStartSimTime;

    // Avoid division by zero
    if (elapsedRealTime <= 0.0)
    {
        return LastCalculatedRTF;
    }

    // Calculate RTF = Simulation Time / Real Time
    const float currentRTF = static_cast<float>(elapsedSimTime / elapsedRealTime);
    
    // Update the last calculated value
    LastCalculatedRTF = currentRTF;
    
    // Reset window for next measurement period
    RTFWindowStartTime = currentRealTime;
    RTFWindowStartSimTime = currentSimTime;
    
    return currentRTF;
}

void ARRROS2GameMode::ResetRTFCalculation()
{
    // Reset window start times and last calculated value
    RTFWindowStartTime = FPlatformTime::Seconds();
    RTFWindowStartSimTime = FApp::GetCurrentTime();
    LastCalculatedRTF = 0.0f;
    bRTFCalculationInitialized = true;
    
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("RTF calculation reset"));
}

void ARRROS2GameMode::SetRTFLogging(bool bEnable, float LogInterval)
{
    bRTFLoggingEnabled = bEnable;
    // Ensure logging interval is at least as long as RTF window size to avoid duplicate logging
    RTFLoggingInterval = FMath::Max(RTFWindowSize, FMath::Max(0.1f, LogInterval));

    // Clear existing timer
    if (RTFLoggingTimerHandle.IsValid())
    {
        GetWorld()->GetTimerManager().ClearTimer(RTFLoggingTimerHandle);
    }

    // Set up new timer if enabled
    if (bRTFLoggingEnabled && IsValid(GetWorld()))
    {
        GetWorld()->GetTimerManager().SetTimer(
            RTFLoggingTimerHandle,
            this,
            &ARRROS2GameMode::LogCurrentRTF,
            RTFLoggingInterval,
            true // Loop
        );
        
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("RTF periodic logging %s (interval: %.2fs, window: %.1fs)"), 
                         bEnable ? TEXT("enabled") : TEXT("disabled"), RTFLoggingInterval, RTFWindowSize);
    }
}

void ARRROS2GameMode::SetRTFWindowSize(float WindowSize)
{
    RTFWindowSize = FMath::Max(1.0f, WindowSize); // Minimum 1 second window
    
    // Update logging interval if it's currently less than the new window size
    if (bRTFLoggingEnabled && RTFLoggingInterval < RTFWindowSize)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("Adjusting RTF logging interval from %.2fs to %.2fs to match window size"), 
                         RTFLoggingInterval, RTFWindowSize);
        SetRTFLogging(bRTFLoggingEnabled, RTFWindowSize);
    }
    
    // Reset calculation to use new window size
    ResetRTFCalculation();
    
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("RTF window size set to %.2f seconds"), RTFWindowSize);
}

void ARRROS2GameMode::LogCurrentRTF()
{
    const float currentRTF = GetCurrentRTF();
    const float targetRTF = GetTargetRTF();
    
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("RTF Status - Current: %.3f, Target: %.3f, Ratio: %.1f%% (Window: %.1fs)"), 
                     currentRTF, targetRTF, targetRTF > 0 ? (currentRTF / targetRTF * 100.0f) : 0.0f, RTFWindowSize);
}