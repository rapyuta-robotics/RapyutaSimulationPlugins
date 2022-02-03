// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotROSController.h"

// UE
#include "Kismet/GameplayStatics.h"

// rclUE
#include "Msgs/ROS2JointTrajectoryControllerStateMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"
#include "Sensors/RR2DLidarComponent.h"
#include "Tools/RRGeneralUtils.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"
#include "Tools/UEUtilities.h"

void ARRRobotROSController::InitRobotROS2Node(APawn* InPawn)
{
    if (nullptr == RobotROS2Node)
    {
        RobotROS2Node = GetWorld()->SpawnActor<AROS2Node>();
    }
    RobotROS2Node->AttachToActor(InPawn, FAttachmentTransformRules::KeepRelativeTransform);
    // GUID is to make sure the node name is unique, even for multiple Sims?
    RobotROS2Node->Name = URRGeneralUtils::GetNewROS2NodeName(InPawn->GetName());

    // Set robot's [ROS2Node] namespace from spawn parameters if existing
    UROS2Spawnable* rosSpawnParameters = InPawn->FindComponentByClass<UROS2Spawnable>();
    if (rosSpawnParameters)
    {
        RobotROS2Node->Namespace = rosSpawnParameters->GetNamespace();
    }
    else
    {
        RobotROS2Node->Namespace = CastChecked<ARobotVehicle>(InPawn)->RobotUniqueName;
    }
    RobotROS2Node->Init();
}

bool ARRRobotROSController::InitPublishers(APawn* InPawn)
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

    if (bPublishJointStates)
    {
        if (nullptr == JointStatesPublisher)
        {
            JointStatesPublisher = NewObject<URRROS2JointStatesPublisher>(this);
            JointStatesPublisher->SetupUpdateCallback();
        }
        JointStatesPublisher->InitializeWithROS2(RobotROS2Node);
        JointStatesPublisher->SetTargetRobot(CastChecked<ARobotVehicle>(InPawn));
    }
    return true;
}

void ARRRobotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // Track InPawn's initial pose
    InitialPosition = InPawn->GetActorLocation();
    InitialOrientation = InPawn->GetActorRotation();
    InitialOrientation.Yaw += 180.f;

    // Instantiate a ROS2 node for each possessed [InPawn]
    InitRobotROS2Node(InPawn);

    // Initialize Pawn's sensors (lidar, etc.)
    verify(CastChecked<ARobotVehicle>(InPawn)->InitSensors(RobotROS2Node));

    // Refresh TF, Odom publishers
    verify(InitPublishers(InPawn));
}

void ARRRobotROSController::OnUnPossess()
{
    if (bPublishOdom && OdomPublisher)
    {
        OdomPublisher->RevokeUpdateCallback();
        OdomPublisher->RobotVehicle = nullptr;
    }
    Super::OnUnPossess();
}

void ARRRobotROSController::SubscribeToJointTrajControllerStatesTopic(const FString& InTopicName)
{
    // Subscription with callback to enqueue vehicle spawn info.
    if (ensure(IsValid(RobotROS2Node)))
    {
        FSubscriptionCallback cb;
        cb.BindDynamic(this, &ARRRobotROSController::JointTrajectoryControllerStateCallback);
        RobotROS2Node->AddSubscription(InTopicName, UROS2JointTrajectoryControllerStateMsg::StaticClass(), cb);
    }
}
