// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#include "Core/RRNetworkPlayerController.h"

// UE
#include "Kismet/GameplayStatics.h"
#include "Math/Rotator.h"
#include "Misc/CommandLine.h"
#include "Net/UnrealNetwork.h"

// rclUE
#include "Msgs/ROS2Clock.h"
#include "Msgs/ROS2Twist.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRUObjectUtils.h"
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotBaseVehicle.h"
#include "Robots/RRRobotROS2Interface.h"
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
    UpdateLocalClock(DeltaSeconds);
}

void ARRNetworkPlayerController::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(ARRNetworkPlayerController, ServerSimState);
    DOREPLIFETIME(ARRNetworkPlayerController, ROS2SimStateClient);
    DOREPLIFETIME(ARRNetworkPlayerController, PlayerName);
}

void ARRNetworkPlayerController::CreateROS2SimStateClient(const TSubclassOf<URRROS2SimulationStateClient>& InSimStateClientClass)
{
    if (ROS2SimStateClient)
    {
        return;
    }

    ROS2SimStateClient = NewObject<URRROS2SimulationStateClient>(
        this, InSimStateClientClass, FName(*FString::Printf(TEXT("%s_ROS2SimStateClient"), *PlayerName)));
    ROS2SimStateClient->SetNetworkPlayerId(GetPlayerState<APlayerState>()->GetPlayerId());
    ROS2SimStateClient->RegisterComponent();
    AddOwnedComponent(ROS2SimStateClient);

    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[PC:%u][NetworkPlayerId:%d] ROS2SimStateClient[%s:%u] created"),
           this,
           ROS2SimStateClient->GetNetworkPlayerId(),
           *ROS2SimStateClient->GetName(),
           ROS2SimStateClient);
}

void ARRNetworkPlayerController::ClientInitSimStateClientROS2_Implementation()
{
    if (SimStateClientROS2Node)
    {
        return;
    }

    if (nullptr == ROS2SimStateClient)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s][ARRNetworkPlayerController::ClientInitSimStateClientROS2] ROS2SimStateClient not found."),
               *GetName());
        return;
    }

    // Init SimStateClient's [ROS2Node] & [ClockPublisher]
    SimStateClientROS2Node = NewObject<UROS2Node>(this);
    SimStateClientROS2Node->Namespace.Reset();
    // NOTE: Its NameSpace will be set to [PlayerName] in [ServerSetPlayerName()]
    SimStateClientROS2Node->Name = FString::Printf(TEXT("%s_ROS2Node"), *PlayerName);
#if WITH_EDITOR
    SimStateClientROS2Node->Namespace = PlayerName;
#endif
    SimStateClientROS2Node->Init();

    // Init [ROS2SimStateClient] with [SimStateClientROS2Node]
    check(ROS2SimStateClient);
    ROS2SimStateClient->Init(SimStateClientROS2Node);

    // Create Clock publisher
    SimStateClientClockPublisher = NewObject<URRROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2NodeActor::AddPublisher()]
    SimStateClientClockPublisher->InitializeWithROS2(SimStateClientROS2Node);

    UE_LOG(LogRapyutaCore, Log, TEXT("[%s] SimStateClient ROS2Node[%s] created"), *PlayerName, *SimStateClientROS2Node->Name);
}

void ARRNetworkPlayerController::OnRep_SimStateClient()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[%s] [ARRNetworkPlayerController::OnRep_SimStateClient] Playername: %s, IsNetMode(NM_Client):%d."),
           *GetName(),
           (true == IsNetMode(NM_Client)), );

#endif
    if (IsLocalController())
    {
        if ((true == IsNetMode(NM_Client)) && (PlayerName != URRCoreUtils::PIXEL_STREAMER_PLAYER_NAME))
        {
            // 1- Init [SimStateClientROS2Node] only once [ROS2SimStateClient] is created
            TInlineComponentArray<URRROS2SimulationStateClient*> simStateComponents(this);
            ROS2SimStateClient = (simStateComponents.Num() > 0) ? simStateComponents[0] : nullptr;
            UE_LOG(LogRapyutaCore, Log, TEXT("[%s] [ARRNetworkPlayerController::OnRep_SimStateClient"), *GetName());

            ClientInitSimStateClientROS2();
        }
    }
}

void ARRNetworkPlayerController::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] [ARRNetworkPlayerController::BeginPlay]"), *GetName());

    if (IsLocalController())
    {
        FTimerManager& timerManager = GetWorld()->GetTimerManager();
        timerManager.SetTimer(ClockRequestTimerHandle, this, &ARRNetworkPlayerController::RequestServerTimeUpdate, 5.f, true);

        // Temporaryr hack to sync CameraManager to PlayerStarts sinc camera pose become (0,0,0) for multiplayer.
        if (IsNetMode(NM_Client))
        {
            auto* playerStart = UGameplayStatics::GetActorOfClass(GetWorld(), APlayerStart::StaticClass());
            SetControlRotation(playerStart->GetActorRotation());
        }
    }

    if (IsNetMode(NM_Standalone))
    {
        ClientInitSimStateClientROS2();
    }
}

void ARRNetworkPlayerController::ServerSetPlayerName_Implementation(const FString& InPlayerName)
{
    PlayerName = InPlayerName;
}

// Client Requesting Server to send time, Client Clock at time of request is sent as well
void ARRNetworkPlayerController::ServerRequestLocalClockUpdate_Implementation(float InClientRequestTime)
{
    float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    ClientSendLocalClockUpdate(serverCurrentTime, InClientRequestTime);
}

void ARRNetworkPlayerController::ClientSendLocalClockUpdate_Implementation(float InServerCurrentTime, float InClientRequestTime)
{
    float clientRequestRoundTrip = LocalTime - InClientRequestTime;
    float latencyAdjustedTime = InServerCurrentTime + (clientRequestRoundTrip * 0.5f);
    LocalTime = latencyAdjustedTime;

    GetWorld()->TimeSeconds = LocalTime;
}

void ARRNetworkPlayerController::RequestServerTimeUpdate()
{
    if (IsLocalController())
    {
        ServerRequestLocalClockUpdate(LocalTime);
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
        ServerRequestLocalClockUpdate(LocalTime);
    }
}

void ARRNetworkPlayerController::ServerSetLinearVel_Implementation(ARRBaseRobot* InServerRobot,
                                                                   float InClientTimeStamp,
                                                                   const FTransform& InClientRobotTransform,
                                                                   const FVector& InLinearVel)
{
    // todo: donot work with physics model. GetActoLocaion return constant values.
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[%s] [ServerSetLinearVel_Implementation] %s %s"),
           *GetName(),
           *InClientRobotPosition.ToString(),
           *InServerRobot->GetActorLocation().ToString());
#endif
    auto* robot = Cast<ARRRobotBaseVehicle>(InServerRobot);
    if (robot)
    {
        float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
        robot->SetActorLocation(InClientRobotTransform.GetTranslation() +
                                InClientRobotTransform.GetRotation() * InLinearVel * (serverCurrentTime - InClientTimeStamp));
        //NOTE: Don't use ARRRobotBaseVehicle::SetLinearVel() here, which is only for client
        robot->TargetLinearVel = InLinearVel;
    }
}

void ARRNetworkPlayerController::ServerSetAngularVel_Implementation(ARRBaseRobot* InServerRobot,
                                                                    float InClientTimeStamp,
                                                                    const FRotator& InClientRobotRotation,
                                                                    const FVector& InAngularVel)
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[%s] [ServerSetAngularVel_Implementation] %s %s"),
           *GetName(),
           *InClientRobotRotation.ToString(),
           *InServerRobot->GetActorRotation().ToString());
#endif
    auto* robot = Cast<ARRRobotBaseVehicle>(InServerRobot);
    if (robot)
    {
        float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
        robot->SetActorRotation(InClientRobotRotation + InAngularVel.Rotation() * (serverCurrentTime - InClientTimeStamp));
        //NOTE: Don't use ARRRobotBaseVehicle::SetAngularVel() here, which is only for client
        robot->TargetAngularVel = InAngularVel;
    }
}
