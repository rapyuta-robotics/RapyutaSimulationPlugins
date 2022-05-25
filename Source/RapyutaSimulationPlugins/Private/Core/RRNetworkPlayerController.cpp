// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRNetworkPlayerController.h"

// UE
#include "Kismet/GameplayStatics.h"
#include "Net/UnrealNetwork.h"
#include "Misc/CommandLine.h"
#include "Math/Rotator.h"

// rclUE
#include "Msgs/ROS2TwistMsg.h"
#include "Msgs/ROS2ClockMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameInstance.h"
#include "Core/RRGameState.h"
#include "Core/RRMathUtils.h"
#include "Core/RRROS2GameMode.h"
#include "Core/RRUObjectUtils.h"
#include "Tools/SimulationStateData.h"
#include "Tools/RRROS2ClockPublisher.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"
#include "Robots/RobotVehicle.h"
#include "Sensors/RR2DLidarComponent.h"

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
    if(PlayerName == "pixelstreamer") {
        FRotator InitRot = FRotator(-50, -90, 0);
        SetControlRotation(InitRot);
    }
#endif
    WaitForPawnToPossess();
}

void ARRNetworkPlayerController::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME( ARRNetworkPlayerController, ROS2Node );
    DOREPLIFETIME( ARRNetworkPlayerController, ClockPublisher );
    DOREPLIFETIME( ARRNetworkPlayerController, SimulationStateData );
    DOREPLIFETIME( ARRNetworkPlayerController, PlayerName );
    DOREPLIFETIME( ARRNetworkPlayerController, Namespace );
    DOREPLIFETIME( ARRNetworkPlayerController, RobotROS2Node );
    DOREPLIFETIME( ARRNetworkPlayerController, PossessedPawn );
}

void ARRNetworkPlayerController::WaitForPawnToPossess()
{
    if(this->PlayerState && !PossessedPawn && GetNetMode() == NM_Client && PlayerName != "pixelstreamer") {

#if WITH_EDITOR
        if(this->PlayerState) {
            if (PlayerName == "") {
                PlayerName = this->PlayerState->GetPlayerName();
                SetServerPlayerName(this->PlayerState->GetPlayerName());
            }
        }
#endif

        if(!SimulationStateData) {
            SimulationStateData = Cast<ASimulationStateData>(UGameplayStatics::GetActorOfClass(GetWorld(), ASimulationStateData::StaticClass()));
        }
        if (SimulationStateData) {
            AActor *MatchingEntity = nullptr;
            UROS2Spawnable *MatchingEntitySpawnParams = nullptr;

            for (AActor *Entity: SimulationStateData->EntityList) {
                if(Entity) {
                    UROS2Spawnable *rosSpawnParameters = Entity->FindComponentByClass<UROS2Spawnable>();
                    if (rosSpawnParameters) {
                        if (rosSpawnParameters->GetName() == PlayerName) {
                            MatchingEntity = Entity;
                            MatchingEntitySpawnParams=rosSpawnParameters;
                        }
                    }
                }
            }
            if (MatchingEntity) {
                UE_LOG(LogTemp, Warning, TEXT("Player [%s] Possessing Robot"), *PlayerName);
                ServerPossessPawn(MatchingEntity);
                PossessedPawn = Cast<APawn>(MatchingEntity);
                ClientInitMoveComp(MatchingEntity);
            } else {
                UE_LOG(LogTemp, Warning, TEXT("Player [%s] has not found a Robot to possess yet"), *PlayerName);
            }
        }
    }
}

void ARRNetworkPlayerController::ServerPossessPawn_Implementation(AActor* InActor)
{
    this->Possess(Cast<APawn>(InActor));
}
void ARRNetworkPlayerController::ClientInitMoveComp_Implementation(AActor* InActor)
{
    Cast<ARobotVehicle>(InActor)->InitMoveComponent();
}
void ARRNetworkPlayerController::InitRobotROS2Node(APawn* InPawn)
{
    if (nullptr == RobotROS2Node) {
        UE_LOG(LogTemp, Warning, TEXT("Player [%s] Spawning ROS2 Node"), *PlayerName);
        RobotROS2Node = GetWorld()->SpawnActor<AROS2Node>();
    }

    RobotROS2Node->AttachToActor(InPawn, FAttachmentTransformRules::KeepRelativeTransform);
    // GUID is to make sure the node name is unique, even for multiple Sims?
    RobotROS2Node->Name = URRGeneralUtils::GetNewROS2NodeName(InPawn->GetName());

    // Set robot's [ROS2Node] namespace from spawn parameters if existing
    UROS2Spawnable *rosSpawnParameters = InPawn->FindComponentByClass<UROS2Spawnable>();
    if (rosSpawnParameters) {
        Namespace = rosSpawnParameters->GetNamespace();
        RobotROS2Node->Namespace = Namespace;
    } else {
        Namespace = CastChecked<ARobotVehicle>(InPawn)->RobotUniqueName;
        RobotROS2Node->Namespace = Namespace;
    }
    RobotROS2Node->Init();
}



bool ARRNetworkPlayerController::InitPublishers(APawn* InPawn)
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // OdomPublisher (with TF)
    if (bPublishOdom)
    {
        if (nullptr == OdomPublisher)
        {
            OdomPublisher = NewObject<URRROS2OdomPublisher>(this);
            OdomPublisher->SetupUpdateCallback();
            OdomPublisher->bPublishOdomTf = bPublishOdomTf;
        }
        OdomPublisher->InitializeWithROS2(RobotROS2Node);
        OdomPublisher->RobotVehicle = CastChecked<ARobotVehicle>(InPawn);
    }

    return true;
}

void ARRNetworkPlayerController::InitRobotPublisher(APawn* InPawn)
{
    verify(CastChecked<ARobotVehicle>(InPawn)->InitSensors(RobotROS2Node));

    if (bPublishOdom)
    {
        if (nullptr == OdomPublisher)
        {
            OdomPublisher = NewObject<URRROS2OdomPublisher>(this);
            OdomPublisher->SetupUpdateCallback();
            OdomPublisher->bPublishOdomTf = bPublishOdomTf;
        }
        OdomPublisher->InitializeWithROS2(RobotROS2Node);
        OdomPublisher->RobotVehicle = CastChecked<ARobotVehicle>(InPawn);
    }
}

void ARRNetworkPlayerController::BeginPlay()
{
    Super::BeginPlay();

#if !WITH_EDITOR
    FString RobotName;
    if (FParse::Value(FCommandLine::Get(), TEXT("robotname"), RobotName)) {
        RobotName = RobotName.Replace(TEXT("="), TEXT("")).Replace(TEXT("\""), TEXT("")); // replace quotes
        PlayerName = RobotName;
        Namespace = RobotName;
        this->SetName(RobotName);
        SetServerPlayerName(RobotName);
    }
#endif

    if(PlayerName == "pixelstreamer") {
        FRotator InitRot = FRotator(-50, -90, 0);
        SetControlRotation(InitRot);
    }

}

void ARRNetworkPlayerController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    if(PlayerName != "") {
        // Set Name
        UROS2Spawnable *RobotInfo = InPawn->FindComponentByClass<UROS2Spawnable>();
        ARobotVehicle *Robot = Cast<ARobotVehicle>(InPawn);

        Robot->RobotUniqueName = RobotInfo->GetName();

    }
}

void ARRNetworkPlayerController::OnUnPossess()
{
    if (bPublishOdom && OdomPublisher)
    {
        OdomPublisher->RevokeUpdateCallback();
        OdomPublisher->RobotVehicle = nullptr;
    }
    Super::OnUnPossess();
}

void ARRNetworkPlayerController::AcknowledgePossession(APawn* InPawn)
{
    Super::AcknowledgePossession(InPawn);

    if(PlayerName != "") {
        UE_LOG(LogTemp, Warning, TEXT("Player [%s] checking if Pawn is Valid for instantiation"), *PlayerName);
        if(Cast<ARobotVehicle>(InPawn)) {
            InitialPosition = InPawn->GetActorLocation();
            InitialOrientation = InPawn->GetActorRotation();
            InitialOrientation.Yaw += 180.f;

            // Instantiate a ROS2 node for each possessed [InPawn]
            InitRobotROS2Node(InPawn);
        }
    }
}

void ARRNetworkPlayerController::SetServerPlayerName_Implementation(const FString& InPlayerName)
{
    this->PlayerName = InPlayerName;
    this->Namespace = InPlayerName;
}

void ARRNetworkPlayerController::SubscribeToMovementCommandTopic_Implementation(const FString& InTopicName)
{
    // Subscription with callback to enqueue vehicle spawn info.
    if (ensure(IsValid(RobotROS2Node)))
    {
        FSubscriptionCallback cb;
        cb.BindDynamic(this, &ARRNetworkPlayerController::MovementCallback);
        RobotROS2Node->AddSubscription(InTopicName, UROS2TwistMsg::StaticClass(), cb);
    }
}

void ARRNetworkPlayerController::MovementCallback(const UROS2GenericMsg* Msg)
{
    const UROS2TwistMsg* twistMsg = Cast<UROS2TwistMsg>(Msg);
    if (IsValid(twistMsg))
    {
        // TODO refactoring will be needed to put units and system of reference conversions in a consistent location
        // probably should not stay in msg though
        FROSTwist twist;
        twistMsg->GetMsg(twist);
        const FVector linear(ConversionUtils::VectorROSToUE(twist.linear));
        const FVector angular(ConversionUtils::RotationROSToUE(twist.angular));

        // (Note) In this callback, which could be invoked from a ROS working thread,
        // the ROSController itself (this) could have been garbage collected,
        // thus any direct referencing to its member in this GameThread lambda needs to be verified.
        AsyncTask(ENamedThreads::GameThread,
                  [linear, angular, vehicle = CastChecked<ARobotVehicle>(GetPawn())]
                  {
                      if (IsValid(vehicle))
                      {
                          vehicle->SetLinearVel(linear);
                          vehicle->SetAngularVel(angular);
                      }
                  });
    }
}






