// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2PlayerController.h"

// UE
#include "HAL/PlatformMisc.h"
#include "Kismet/GameplayStatics.h"
#include "Net/UnrealNetwork.h"

// rclUE
#include "Msgs/ROS2TwistMsg.h"
#include "Msgs/ROS2ClockMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2ClockPublisher.h"
#include "Tools/SimulationState.h"
#include "Robots/RobotVehicle.h"
#include "Sensors/RR2DLidarComponent.h"
#include "Tools/RRGeneralUtils.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"
#include "Tools/UEUtilities.h"
#include "RRROS2GameMode.h"

ARRROS2PlayerController::ARRROS2PlayerController()
{
    //TODO ADD default name for client
    UE_LOG(LogTemp, Warning, TEXT("PLAYER Created"));
    bReplicates = true;
    PrimaryActorTick.bCanEverTick = true;
    bShowMouseCursor = true;
    bEnableClickEvents = true;
    bEnableMouseOverEvents = true;
}

void ARRROS2PlayerController::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    WaitForPawnToPossess();
}

void ARRROS2PlayerController::WaitForPawnToPossess_Implementation()
{
    if(SimulationStateData && !PossessedPawn) {
        AActor* MatchingEntity = nullptr;
        for(AActor* Entity : SimulationStateData->EntityList) {
            UROS2Spawnable *rosSpawnParameters = Entity->FindComponentByClass<UROS2Spawnable>();
            if(rosSpawnParameters) {
                if(rosSpawnParameters->GetName() == PlayerName) {
                    MatchingEntity = Entity;
                }
            }
        }
        if(MatchingEntity){
            UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s] Possessing Robot"), *PlayerName);
            this->Possess(Cast<APawn>(MatchingEntity));
            PossessedPawn = Cast<APawn>(MatchingEntity);
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s] has not found a Robot to possess yet"), *PlayerName);
        }
    }
}

void ARRROS2PlayerController::CheckClientTiming_Implementation()
{

}

void ARRROS2PlayerController::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME( ARRROS2PlayerController, ROS2Node );
    DOREPLIFETIME( ARRROS2PlayerController, ClockPublisher );
    DOREPLIFETIME( ARRROS2PlayerController, SimulationStateData );
    DOREPLIFETIME( ARRROS2PlayerController, PlayerName );
    DOREPLIFETIME( ARRROS2PlayerController, Namespace );
    DOREPLIFETIME( ARRROS2PlayerController, RobotROS2Node );
    DOREPLIFETIME( ARRROS2PlayerController, PossessedPawn );
}

void ARRROS2PlayerController::Init(FString InName, ASimulationStateData* InSimulationStateData)
{
    PlayerName = InName;
    SimulationStateData = InSimulationStateData;

    UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s] IN LOGGED"), *PlayerName);


}


void ARRROS2PlayerController::InitRobotROS2Node(APawn* InPawn)
{

    if (nullptr == RobotROS2Node) {
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



bool ARRROS2PlayerController::InitPublishers(APawn* InPawn)
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

void ARRROS2PlayerController::InitRobotPublisher(APawn* InPawn)
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

void ARRROS2PlayerController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    if(PlayerName != "") {
        // Set Name
        UROS2Spawnable *RobotInfo = InPawn->FindComponentByClass<UROS2Spawnable>();
        ARobotVehicle *Robot = Cast<ARobotVehicle>(InPawn);

        Robot->RobotUniqueName = RobotInfo->GetName();

    }
}

void ARRROS2PlayerController::OnUnPossess()
{
    if (bPublishOdom && OdomPublisher)
    {
        OdomPublisher->RevokeUpdateCallback();
        OdomPublisher->RobotVehicle = nullptr;
    }
    Super::OnUnPossess();
}

void ARRROS2PlayerController::AcknowledgePossession(APawn* InPawn)
{
    Super::AcknowledgePossession(InPawn);

    if(PlayerName != "") {
        if(Cast<ARobotVehicle>(InPawn)) {
            InitialPosition = InPawn->GetActorLocation();
            InitialOrientation = InPawn->GetActorRotation();
            InitialOrientation.Yaw += 180.f;

            // Instantiate a ROS2 node for each possessed [InPawn]
            InitRobotROS2Node(InPawn);
        }
    }
}

void ARRROS2PlayerController::GetClientPawn_Implementation(APawn* InPawn) {

//    UE_LOG(LogTemp, Warning, TEXT("Finding matching Client Pawn"));
//
//    if (GetNetMode() == NM_DedicatedServer) {
//        //          Get Client version of InPawn
//
//        FString LenActors = FString::FromInt(SimulationStateData->SpawnedEntityList.Num());
//        UE_LOG(LogTemp, Warning, TEXT("[%s] IN Client SIMULATION STATE LIST"), *LenActors);
//
//        for (AActor* Actor: SimulationStateData->SpawnedEntityList) {
//            ARobotVehicle *TempRobotVehicle = Cast<ARobotVehicle>(Actor);
//            if(Actor){
//                UE_LOG(LogTemp, Warning, TEXT("STUFFFF"));
//            }
//        }
//        ARobotVehicle *ClientRobotVehicle = nullptr;
//        for (AActor* Actor: SimulationStateData->EntityList) {
//            ARobotVehicle* TempRobotVehicle = Cast<ARobotVehicle>(Actor);
//            if(!Actor){
//                UE_LOG(LogTemp, Warning, TEXT("EntityList Actor NULL"));
//            }
//            if (TempRobotVehicle) {
//                UE_LOG(LogTemp, Warning, TEXT("Found a Pawn"));
//                if (TempRobotVehicle->RobotUniqueName == PlayerName) {
//                    ClientRobotVehicle = TempRobotVehicle;
//                    UE_LOG(LogTemp, Warning, TEXT("Found matching pawn"));
//                }
//            }
//
//        }
//
//        RepPawn = Cast<APawn>(ClientRobotVehicle);
//        TArray < AActor * > RobotActors;
//        UGameplayStatics::GetAllActorsOfClass(GetWorld(), ARobotVehicle::StaticClass(), RobotActors);
//
//        FString LenActors = FString::FromInt(RobotActors.Num());
//        UE_LOG(LogTemp, Warning, TEXT("[%s] IN LIST"), *LenActors);
//        for (AActor *RobotActor: RobotActors) {
//            ARobotVehicle *ClientRobot = Cast<ARobotVehicle>(RobotActor);
//            UE_LOG(LogTemp, Warning, TEXT("Client [%s] IN LOGGED"), *ClientRobot->RobotUniqueName);
//            UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s] IN LOGGED"), *PlayerName);
//            if (ClientRobot->RobotUniqueName == PlayerName) {
//
//                APawn *ClientPawn = Cast<APawn>(ClientRobot);
//                InitPawn(ClientPawn);
//            }
//        }

    }


void ARRROS2PlayerController::InitPawn_Implementation(APawn* InPawn)
{
    UE_LOG(LogTemp, Warning, TEXT("TRYING TO POSSESS PAWN"));
    UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s] IN LOGGED"), *PlayerName);




    if(!InPawn){
        UE_LOG(LogTemp, Warning, TEXT("Client NULL PAWN"));
    }

    if( GetNetMode() == NM_Client ) {
        // Track InPawn's initial pose
        InitialPosition = InPawn->GetActorLocation();
        InitialOrientation = InPawn->GetActorRotation();
        InitialOrientation.Yaw += 180.f;

        // Instantiate a ROS2 node for each possessed [InPawn]
        UE_LOG(LogTemp, Warning, TEXT("CLIENT DOING STUFF"));
        InitRobotROS2Node(InPawn);

        //            // Initialize Pawn's sensors (lidar, etc.)
        //            InitRobotPublisher(InPawn);
        //
        //
        //            // Refresh TF, Odom publishers
        //            InitPublishers(InPawn);

        //            GaussianRNGOdom = std::normal_distribution<>{OdomNoiseMean, OdomNoiseVariance};

        SubscribeToMovementCommandTopic(TEXT("cmd_vel"));

    }

}

void ARRROS2PlayerController::SubscribeToMovementCommandTopic_Implementation(const FString& InTopicName)
{
    // Subscription with callback to enqueue vehicle spawn info.
    if (ensure(IsValid(RobotROS2Node)))
    {
        FSubscriptionCallback cb;
        cb.BindDynamic(this, &ARRROS2PlayerController::MovementCallback);
        RobotROS2Node->AddSubscription(InTopicName, UROS2TwistMsg::StaticClass(), cb);
    }
}

void ARRROS2PlayerController::MovementCallback(const UROS2GenericMsg* Msg)
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
