// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2PlayerController.h"

// UE
#include "HAL/PlatformMisc.h"
#include "Kismet/GameplayStatics.h"

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


}

void ARRROS2PlayerController::Init(FString InName)
{
    PlayerName = InName;

    UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s] IN LOGGED"), *PlayerName);


}


void ARRROS2PlayerController::InitRobotROS2Node(APawn* InPawn)
{

    if( GetLocalRole() == ROLE_Authority ) {
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
//            RobotROS2Node->Namespace = rosSpawnParameters->GetNamespace();
            RobotROS2Node->Namespace = Namespace;
        } else {
//            RobotROS2Node->Namespace = CastChecked<ARobotVehicle>(InPawn)->RobotUniqueName;
            Namespace = CastChecked<ARobotVehicle>(InPawn)->RobotUniqueName;
            RobotROS2Node->Namespace = Namespace;
        }
        RobotROS2Node->Init();
    }
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

void ARRROS2PlayerController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);


    if(PlayerName != "") {
        if( GetLocalRole() == ROLE_Authority ) {
            // Track InPawn's initial pose
            InitialPosition = InPawn->GetActorLocation();
            InitialOrientation = InPawn->GetActorRotation();
            InitialOrientation.Yaw += 180.f;

            // Instantiate a ROS2 node for each possessed [InPawn]
            UE_LOG(LogTemp, Warning, TEXT("PLAYER POSSESSING PAWN"));
            InitRobotROS2Node(InPawn);

            // Initialize Pawn's sensors (lidar, etc.)
            verify(CastChecked<ARobotVehicle>(InPawn)->InitSensors(RobotROS2Node));


            // Refresh TF, Odom publishers
            verify(InitPublishers(InPawn));

//            GaussianRNGOdom = std::normal_distribution<>{OdomNoiseMean, OdomNoiseVariance};
//
//            SubscribeToMovementCommandTopic(TEXT("cmd_vel"));


        }
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

void ARRROS2PlayerController::SubscribeToMovementCommandTopic(const FString& InTopicName)
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
