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
    DOREPLIFETIME(ARRNetworkPlayerController, ClientROS2Node);
    DOREPLIFETIME(ARRNetworkPlayerController, ClockPublisher);
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

    UE_LOG(LogRapyutaCore, Warning, TEXT("[PC:%u] ROS2SimStateClient[%s] created"), this, *ROS2SimStateClient->GetName());
}

void ARRNetworkPlayerController::InitClientROS2()
{
    if (ClientROS2Node || (false == IsNetMode(NM_Client)))
    {
        return;
    }

    // Init [ClientROS2Node] & [ClockPublisher]
    UWorld* currentWorld = GetWorld();
    ClientROS2Node = currentWorld->SpawnActor<AROS2Node>();
    ClientROS2Node->Namespace.Reset();
    // ClientROS2Node->Namespace = PlayerName;
    ClientROS2Node->Name = FString::Printf(TEXT("%s_ROS2Node"), *PlayerName);
    ClientROS2Node->Init();

    // Init [ROS2SimStateClient] with [ClientROS2Node]
    // Also [ROS2SimStateClient]'s ServerSimState is set here-in
    ROS2SimStateClient->Init(ClientROS2Node);

    // Create Clock publisher
    ClockPublisher = NewObject<URRROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
    ClockPublisher->InitializeWithROS2(ClientROS2Node);

    UE_LOG(LogRapyutaCore, Warning, TEXT("ARRNetworkPlayerController ClientROS2Node[%s] created"), *ClientROS2Node->Name);
}

void ARRNetworkPlayerController::WaitForPawnPossess()
{
#if 1    // RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("ARRNetworkPlayerController::WaitForPawnPossess[%u:%s] %s NM_Client(%d) ROS2SimStateClient(%ld)"),
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

    // 1- Init [ClientROS2] only once [ROS2SimStateClient] is created
    if (ROS2SimStateClient)
    {
        InitClientROS2();
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
            if (IsValid(entityPawn) && (GetPawn() != entityPawn))
            {
                // 2.2- Possess [matchingEntity] + Init its ROS2Inteface + MoveComp if as a robot
                ServerPossessPawn(entityPawn);
                PossessedPawn = entityPawn;

                // Refer to ARRBaseRobot::CreateROS2Interface() for reasons why it is inited here but not earlier
                ClientInitRobotROS2Interface(entity);
                ClientInitRobotMoveComp(entity);

                GetWorld()->GetTimerManager().ClearTimer(PossessTimerHandle);

#if WITH_EDITOR
                // 2.3- Set Controller's PlayerName -> entity's Name
                PlayerName = entity->GetName();
                ServerSetPlayerName(PlayerName);
#endif
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
    UE_LOG(LogRapyutaCore, Warning, TEXT("Player[%s] possessed Pawn[%s]"), *PlayerName, *InPawn->GetName());
}

void ARRNetworkPlayerController::ClientInitRobotROS2Interface_Implementation(AActor* InActor)
{
    if (ARRBaseRobot* robot = Cast<ARRBaseRobot>(InActor))
    {
        robot->ROS2Interface->Initialize(robot);
    }
}

void ARRNetworkPlayerController::ClientInitRobotMoveComp_Implementation(AActor* InActor)
{
    if (ARRRobotBaseVehicle* robotVehicle = Cast<ARRRobotBaseVehicle>(InActor))
    {
        robotVehicle->InitMoveComponent();
    }
}

void ARRNetworkPlayerController::BeginPlay()
{
    Super::BeginPlay();

#if !WITH_EDITOR
    FString robotName;
    if (FParse::Value(FCommandLine::Get(), TEXT("robotname"), robotName))
    {
        // Remove '"' & '=' in Robot Name
        robotName = robotName.Replace(TEXT("="), TEXT("")).Replace(TEXT("\""), TEXT(""));
        PlayerName = robotName;
        ClientROS2Node->Namespace = robotName;
        this->SetName(robotName);
        ServerSetPlayerName(robotName);
    }
#endif

    if (PlayerName == URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME)
    {
        SetControlRotation(FRotator(-50.f, -90.f, 0));
    }

    if (IsLocalController())
    {
        FTimerManager& timerManager = GetWorld()->GetTimerManager();
        timerManager.SetTimer(PossessTimerHandle, this, &ARRNetworkPlayerController::WaitForPawnPossess, 1.f, true);
        timerManager.SetTimer(ClockRequestTimerHandle, this, &ARRNetworkPlayerController::RequestServerTime, 5.f, true);
    }
}

void ARRNetworkPlayerController::ServerSetPlayerName_Implementation(const FString& InPlayerName)
{
    PlayerName = InPlayerName;
    if (ClientROS2Node)
    {
        ClientROS2Node->Namespace = InPlayerName;
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
