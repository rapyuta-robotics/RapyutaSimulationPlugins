// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#include "Core/RRNetworkPlayerController.h"

// UE
#include "Kismet/GameplayStatics.h"
#include "Math/Rotator.h"
#include "Misc/CommandLine.h"
#include "Net/UnrealNetwork.h"

// rclUE
#include "Msgs/ROS2ClockMsg.h"
#include "Msgs/ROS2TwistMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRUObjectUtils.h"
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotBaseVehicle.h"
#include "Robots/RRRobotROS2Interface.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/RRROS2SimulationStateClient.h"
#include "Tools/SimulationState.h"

ARRNetworkPlayerController::ARRNetworkPlayerController()
{
    bShowMouseCursor = true;
    DefaultMouseCursor = EMouseCursor::Crosshairs;
    bReplicates = true;
    PrimaryActorTick.bCanEverTick = true;
}

void ARRNetworkPlayerController::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
#if WITH_EDITOR
    if (PlayerName == URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME)
    {
        FRotator InitRot = FRotator(-50, -90, 0);
        SetControlRotation(InitRot);
    }
#endif
    UpdateLocalClock(DeltaSeconds);
}

void ARRNetworkPlayerController::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(ARRNetworkPlayerController, SimStateClientROS2Node);
    DOREPLIFETIME(ARRNetworkPlayerController, SimStateClientClockPublisher);
    DOREPLIFETIME(ARRNetworkPlayerController, ServerSimState);
    DOREPLIFETIME(ARRNetworkPlayerController, ROS2SimStateClient);
    DOREPLIFETIME(ARRNetworkPlayerController, PlayerName);
    DOREPLIFETIME(ARRNetworkPlayerController, PossessedPawn);
}

void ARRNetworkPlayerController::CreateROS2SimStateClient(const TSubclassOf<URRROS2SimulationStateClient>& InSimStateClientClass)
{
    if (ROS2SimStateClient)
    {
        return;
    }

    ROS2SimStateClient = NewObject<URRROS2SimulationStateClient>(
        this, InSimStateClientClass, FName(*FString::Printf(TEXT("%sROS2SimStateClient"), *GetName())));
    ROS2SimStateClient->RegisterComponent();
    AddOwnedComponent(ROS2SimStateClient);

    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[PC:%u] ROS2SimStateClient[%s:%u] created"),
           this,
           *ROS2SimStateClient->GetName(),
           ROS2SimStateClient);
}

void ARRNetworkPlayerController::InitServerROS2()
{
    if (SimStateClientROS2Node || (false == IsNetMode(NM_Client)))
    {
        return;
    }

    // Init SimStateClient's [ROS2Node] & [ClockPublisher]
    UWorld* currentWorld = GetWorld();
    SimStateClientROS2Node = currentWorld->SpawnActor<AROS2Node>();
    SimStateClientROS2Node->Namespace.Reset();
    // NOTE: Its NameSpace will be set to [PlayerName] in [ServerSetPlayerName()]
    SimStateClientROS2Node->Name = FString::Printf(TEXT("%s_ROS2Node"), *PlayerName);
    SimStateClientROS2Node->Init();

    // Init [ROS2SimStateClient] with [SimStateClientROS2Node]
    check(ROS2SimStateClient);
    ROS2SimStateClient->Init(SimStateClientROS2Node);

    // Create Clock publisher
    SimStateClientClockPublisher = NewObject<URRROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
    SimStateClientClockPublisher->InitializeWithROS2(SimStateClientROS2Node);

    UE_LOG(LogRapyutaCore, Warning, TEXT("ARRNetworkPlayerController ServerROS2Node[%s] created"), *SimStateClientROS2Node->Name);
}

void ARRNetworkPlayerController::WaitToPossessPawn()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("ARRNetworkPlayerController::WaitToPossessPawn[%u:%s] %s NM_Client(%d) ROS2SimStateClient(%u)"),
           this,
           *GetName(),
           *PlayerName,
           IsNetMode(NM_Client),
           ROS2SimStateClient);
#endif

    if ((false == IsNetMode(NM_Client)) || (PlayerName == URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME))
    {
        return;
    }

    // 1- Init [ServerROS2] only once [ROS2SimStateClient] is created
    TInlineComponentArray<URRROS2SimulationStateClient*> simStateComponents(this);
    ROS2SimStateClient = (simStateComponents.Num() > 0) ? simStateComponents[0] : nullptr;
    if (ROS2SimStateClient)
    {
        InitServerROS2();
    }

    // 2- Keep waiting for a spawned entity with a matching player name in [ServerSimState->EntityList]
    if (PlayerState && !PossessedPawn)
    {
        if (nullptr == ServerSimState)
        {
            return;
        }

        // 2.1- Possess any newly spawned entity that is not possessed yet by this controller
        for (auto& entity : ServerSimState->EntityList)
        {
            APawn* entityPawn = Cast<APawn>(entity);
            if (IsValid(entityPawn) && (nullptr == entityPawn->GetController()) && (GetPawn() != entityPawn))
            {
                // 2.2- Possess [matchingEntity] + Init its ROS2Inteface + MoveComp if as a robot
                ServerPossessPawn(entityPawn);
                PossessedPawn = entityPawn;

                // 2.3- ClientInitPawn
                // Refer to ARRBaseRobot::CreateROS2Interface() for reasons why it is inited here but not earlier
                ClientInitPawn(entity);

                GetWorld()->GetTimerManager().ClearTimer(PossessTimerHandle);

                // 2.3- Set Controller's PlayerName -> entity's Name
                PlayerName = entity->GetName();
                ServerSetPlayerName(PlayerName);

                break;
            }
        }

#if RAPYUTA_SIM_DEBUG
        if (nullptr == GetPawn())
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("NetworkPlayerController [%s] found no Pawn of its matching name to possess yet"),
                   *PlayerName);
        }
#endif
    }
}

void ARRNetworkPlayerController::ServerPossessPawn_Implementation(APawn* InPawn)
{
    Possess(InPawn);
    // NOTE: Logging PlayerName, InPawn->GetName() here might crash
}

void ARRNetworkPlayerController::ClientInitPawn_Implementation(AActor* InActor)
{
    if (ARRBaseRobot* robot = Cast<ARRBaseRobot>(InActor))
    {
        robot->ROS2Interface->Initialize(robot);
    }

    if (ARRRobotBaseVehicle* robotVehicle = Cast<ARRRobotBaseVehicle>(InActor))
    {
        robotVehicle->InitMoveComponent();
    }
}

void ARRNetworkPlayerController::BeginPlay()
{
    Super::BeginPlay();

    // Set PlayerName to [robotname] if specified as an Sim executor arg
    FString robotName;
    if (FParse::Value(FCommandLine::Get(), TEXT("robotname"), robotName))
    {
        // Remove '"' & '=' in Robot Name
        robotName = robotName.Replace(TEXT("="), TEXT("")).Replace(TEXT("\""), TEXT(""));
        if (false == robotName.IsEmpty())
        {
            PlayerName = robotName;
            SimStateClientROS2Node->Namespace = robotName;
            SetName(robotName);
            ServerSetPlayerName(robotName);
        }
    }

    if (PlayerName == URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME)
    {
        SetControlRotation(FRotator(-50.f, -90.f, 0));
    }

    if (IsLocalController())
    {
        FTimerManager& timerManager = GetWorld()->GetTimerManager();
        timerManager.SetTimer(PossessTimerHandle, this, &ARRNetworkPlayerController::WaitToPossessPawn, 1.f, true);
        timerManager.SetTimer(ClockRequestTimerHandle, this, &ARRNetworkPlayerController::RequestServerTime, 5.f, true);
    }
}

void ARRNetworkPlayerController::ServerSetPlayerName_Implementation(const FString& InPlayerName)
{
    PlayerName = InPlayerName;
    if (SimStateClientROS2Node)
    {
        SimStateClientROS2Node->Namespace = InPlayerName;
    }
}

// Client Requesting Server to send time, Client Clock at time of request is sent as well
void ARRNetworkPlayerController::ClientRequestClock_Implementation(float InClientRequestTime)
{
    float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    ServerSendClock(serverCurrentTime, InClientRequestTime);
}

void ARRNetworkPlayerController::ServerSendClock_Implementation(float InServerCurrentTime, float InClientRequestTime)
{
    float clientRequestRoundTrip = ServerTime - InClientRequestTime;
    float latencyAdjustedTime = InServerCurrentTime + (clientRequestRoundTrip * 0.5f);
    ServerTime = latencyAdjustedTime;
}

void ARRNetworkPlayerController::RequestServerTime()
{
    if (IsLocalController())
    {
        ClientRequestClock(ServerTime);
    }
}

void ARRNetworkPlayerController::UpdateLocalClock(float InDeltaSeconds)
{
    if (IsLocalController())
    {
        ServerTime += InDeltaSeconds;
    }
}

void ARRNetworkPlayerController::ReceivedPlayer()
{
    if (IsLocalController())
    {
        ClientRequestClock(ServerTime);
    }
}
