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

    // After pawn possession, PossessedPawn & Super::Pawn are actually referring to the same one.
    // But we could not setup replication for Pawn here due to its being private in Super
    DOREPLIFETIME(ARRNetworkPlayerController, PossessedPawn);
}

void ARRNetworkPlayerController::CreateROS2SimStateClient(const TSubclassOf<URRROS2SimulationStateClient>& InSimStateClientClass)
{
    if (ROS2SimStateClient)
    {
        return;
    }

    ROS2SimStateClient = NewObject<URRROS2SimulationStateClient>(
        this, InSimStateClientClass, FName(*FString::Printf(TEXT("%s_ROS2SimStateClient"), *PlayerName)));
    ROS2SimStateClient->RegisterComponent();
    AddOwnedComponent(ROS2SimStateClient);

    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[PC:%u] ROS2SimStateClient[%s:%u] created"),
           this,
           *ROS2SimStateClient->GetName(),
           ROS2SimStateClient);
}

void ARRNetworkPlayerController::InitSimStateClientROS2()
{
    if (SimStateClientROS2Node || (false == IsNetMode(NM_Client)))
    {
        return;
    }

    // Init SimStateClient's [ROS2Node] & [ClockPublisher]
    UWorld* currentWorld = GetWorld();
    SimStateClientROS2Node = currentWorld->SpawnActor<AROS2Node>();
    SimStateClientROS2Node->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
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

    UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] SimStateClient ROS2Node[%s] created"), *PlayerName, *SimStateClientROS2Node->Name);
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

    // 1- Init [SimStateClientROS2Node] only once [ROS2SimStateClient] is created
    TInlineComponentArray<URRROS2SimulationStateClient*> simStateComponents(this);
    ROS2SimStateClient = (simStateComponents.Num() > 0) ? simStateComponents[0] : nullptr;
    if (ROS2SimStateClient)
    {
        InitSimStateClientROS2();
    }

    // 2- Keep waiting for a spawned entity with a matching player name in [ServerSimState->EntityList]
    if (PlayerState && !PossessedPawn)
    {
        if (nullptr == ServerSimState)
        {
            return;
        }

        // 2.1- Possess any newly spawned entity that is not possessed yet by this controller
        APawn* matchingPawn = FindPawnToPossess();
        if (IsValid(matchingPawn))
        {
            // 2.2- Possess [matchingEntity] + Init its ROS2Inteface + MoveComp if as a robot
            ServerPossessPawn(matchingPawn);
            // NOTE: [ClientInitPawn(matchingPawn)] will be done at its own Client side only at [AcknowledgePossession()],
            // when it is sure it has been possessed by Server

            // 2.3 - Stop [PossessTimerHandle]
            GetWorld()->GetTimerManager().ClearTimer(PossessTimerHandle);

            PossessedPawn = matchingPawn;
            UE_LOG(LogRapyutaCore, Warning, TEXT("Player[%s] possessed pawn %s"), *PlayerName, *PossessedPawn->GetName());

#if WITH_EDITOR
            // 2.4- Set Controller's PlayerName -> entity's Name
            FString prevPlayerName = PlayerName;
            PlayerName = matchingPawn->GetName();
            ServerSetPlayerName(PlayerName);
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("Player[%s] possessed pawn %s and renamed to same name as pawn name"),
                   *prevPlayerName,
                   *PossessedPawn->GetName());
#else
            // PlayerName is provided from [robotname] param passed to Sim client executor for to-be-possesed robot matching
#endif
        }
#if RAPYUTA_SIM_DEBUG
        else
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("NetworkPlayerController [%s] found no Pawn of its matching name to possess yet"),
                   *PlayerName);
        }
#endif
    }
}

APawn* ARRNetworkPlayerController::FindPawnToPossess()
{
    APawn* matchingPawn = nullptr;
#if WITH_EDITOR
    for (auto& entity : ServerSimState->EntityList)
    {
        matchingPawn = Cast<APawn>(entity);
        ARRBaseRobot* robot = Cast<ARRBaseRobot>(entity);
        if (IsValid(robot) && !robot->isPossessed && (GetPawn() != robot))
        {
            break;
        }
        else
        {
            matchingPawn = nullptr;
        }
    }
#else
    for (AActor* entity : ServerSimState->EntityList)
    {
        // [entity->GetName()] is not reliable due to UObject::NamePrivate is not replicated by default
        ARRBaseRobot* robot = Cast<ARRBaseRobot>(entity);
        if (IsValid(robot) && (robot->RobotUniqueName == PlayerName))
        {
            matchingPawn = Cast<APawn>(entity);
            if (!matchingPawn)
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("Player [%s] found an entity of matching name %s BUT it's not a pawn. "
                            "Please recheck [robotname] param value."),
                       *PlayerName);
            }
            break;
        }
    }
#endif
    return matchingPawn;
}

void ARRNetworkPlayerController::ServerPossessPawn_Implementation(APawn* InPawn)
{
    Possess(InPawn);
}

void ARRNetworkPlayerController::AcknowledgePossession(APawn* InPawn)
{
    // NOTE: [AcknowledgePossession] runs on Client only
    Super::AcknowledgePossession(InPawn);

    if (false == PlayerName.IsEmpty())
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("Player[%s] AcknowledgePossession %s"), *PlayerName, *InPawn->GetName());
        // Refer to ARRBaseRobot::CreateROS2Interface() for reasons why it is inited here but not earlier
        ClientInitPawn(InPawn);
    }
}

void ARRNetworkPlayerController::ClientInitPawn_Implementation(AActor* InActor)
{
    ARRBaseRobot* robot = Cast<ARRBaseRobot>(InActor);
    if (robot)
    {
        // Instantiate ROS2 interface, of which the instance is not replicated, for each possessed robot
        verify(nullptr == robot->ROS2Interface);
        robot->CreateROS2Interface();
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

#if !WITH_EDITOR
    // Set PlayerName to [robotname] if specified as an Sim executor arg
    FString robotName;
    if (URRCoreUtils::GetCommandLineArgumentValue<FString>(CMDLINE_ARG_NET_CLIENT_ROBOT_NAME, robotName))
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
#endif

    if (PlayerName == URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME)
    {
        SetControlRotation(FRotator(-50.f, -90.f, 0));
    }

    if (IsLocalController())
    {
        FTimerManager& timerManager = GetWorld()->GetTimerManager();
        timerManager.SetTimer(PossessTimerHandle, this, &ARRNetworkPlayerController::WaitToPossessPawn, 1.f, true);
        timerManager.SetTimer(ClockRequestTimerHandle, this, &ARRNetworkPlayerController::RequestServerTimeUpdate, 5.f, true);
    }
}

void ARRNetworkPlayerController::ServerSetPlayerName_Implementation(const FString& InPlayerName)
{
    PlayerName = InPlayerName;
}

// Client Requesting Server to send time, Client Clock at time of request is sent as well
void ARRNetworkPlayerController::ClientRequestLocalClockUpdate_Implementation(float InClientRequestTime)
{
    float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    ServerSendLocalClockUpdate(serverCurrentTime, InClientRequestTime);
}

void ARRNetworkPlayerController::ServerSendLocalClockUpdate_Implementation(float InServerCurrentTime, float InClientRequestTime)
{
    float clientRequestRoundTrip = LocalTime - InClientRequestTime;
    float latencyAdjustedTime = InServerCurrentTime + (clientRequestRoundTrip * 0.5f);
    LocalTime = latencyAdjustedTime;
}

void ARRNetworkPlayerController::RequestServerTimeUpdate()
{
    if (IsLocalController())
    {
        ClientRequestLocalClockUpdate(LocalTime);
    }
}

void ARRNetworkPlayerController::UpdateLocalClock(float InDeltaSeconds)
{
    if (IsLocalController())
    {
        LocalTime += InDeltaSeconds;
    }
}

void ARRNetworkPlayerController::ReceivedPlayer()
{
    Super::ReceivedPlayer();
    if (IsLocalController())
    {
        ClientRequestLocalClockUpdate(LocalTime);
    }
}
