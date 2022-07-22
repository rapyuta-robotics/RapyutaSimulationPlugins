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
    LocalClockUpdate(DeltaSeconds);
}

void ARRNetworkPlayerController::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(ARRNetworkPlayerController, ClientROS2Node);
    DOREPLIFETIME(ARRNetworkPlayerController, ClockPublisher);
    DOREPLIFETIME(ARRNetworkPlayerController, SimulationState);
    DOREPLIFETIME(ARRNetworkPlayerController, SimulationStateClient);
    DOREPLIFETIME(ARRNetworkPlayerController, PlayerName);
    DOREPLIFETIME(ARRNetworkPlayerController, PossessedPawn);
}

void ARRNetworkPlayerController::WaitForPawnPossess()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(
        LogRapyutaCore, Warning, TEXT("ARRNetworkPlayerController::WaitForPawnPossess %s %d"), *PlayerName, IsNetMode(NM_Client));
#endif
    // Init [ClientROS2Node] & [ClockPublisher]
    if (!ClientROS2Node && IsNetMode(NM_Client) && !PlayerName.IsEmpty() &&
        (PlayerName != URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME) && SimulationStateClient)
    {
        UWorld* currentWorld = GetWorld();
        ClientROS2Node = currentWorld->SpawnActor<AROS2Node>();
        ClientROS2Node->Namespace.Reset();
        // ClientROS2Node->Namespace = PlayerName;
        ClientROS2Node->Name = FString::Printf(TEXT("%s_ROS2Node"), *PlayerName);
        ClientROS2Node->Init();

        // Init both [SimulationState + SimulationStateClient] with [ClientROS2Node]
        SimulationState->Init(ClientROS2Node);
        SimulationStateClient->Init(ClientROS2Node);

        // Create Clock publisher
        ClockPublisher = NewObject<URRROS2ClockPublisher>(this);
        // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
        ClockPublisher->InitializeWithROS2(ClientROS2Node);

        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("ARRNetworkPlayerController::WaitForPawnPossess ClientROS2Node[%s] created"),
               *ClientROS2Node->Name);
    }

    // Fetch on with matching player name in [SimulationState->EntityList]
    if (PlayerState && !PossessedPawn && IsNetMode(NM_Client) && (PlayerName != URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME))
    {
#if WITH_EDITOR
        if (PlayerName.IsEmpty())
        {
            PlayerName = PlayerState->GetPlayerName();
            ServerSetPlayerName(PlayerState->GetPlayerName());
        }
#endif

        if (SimulationState)
        {
            AActor* matchingEntity = nullptr;
            UROS2Spawnable* matchingEntitySpawnParams = nullptr;

            // Find [matchingEntity] of the same name as PlayerName
            for (auto& entity : SimulationState->EntityList)
            {
                if (entity)
                {
                    UROS2Spawnable* rosSpawnParameters = entity->FindComponentByClass<UROS2Spawnable>();
                    if (rosSpawnParameters)
                    {
                        if (rosSpawnParameters->GetName() == PlayerName)
                        {
                            matchingEntity = entity;
                            matchingEntitySpawnParams = rosSpawnParameters;
                            break;
                        }
                    }
                }
            }

            // Possess [matchingEntity] + Init its ROS2Inteface + MoveComp if as a robot
            if (matchingEntity)
            {
                if (APawn* matchingEntityPawn = Cast<APawn>(matchingEntity))
                {
                    ServerPossessPawn(matchingEntityPawn);
                    PossessedPawn = matchingEntityPawn;
                }

                // Refer to ARRBaseRobot::CreateROS2Interface() for reasons why it is inited here but not earlier
                ClientInitRobotROS2Interface(matchingEntity);
                ClientInitRobotMoveComp(matchingEntity);

                GetWorld()->GetTimerManager().ClearTimer(PossessTimerHandle);
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
}

void ARRNetworkPlayerController::ServerPossessPawn_Implementation(APawn* InPawn)
{
    UE_LOG(LogTemp, Warning, TEXT("Player [%s] Possessing Pawn"), *PlayerName);
    Possess(InPawn);
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
    ClientROS2Node->Namespace = InPlayerName;
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

void ARRNetworkPlayerController::LocalClockUpdate(float InDeltaSeconds)
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
