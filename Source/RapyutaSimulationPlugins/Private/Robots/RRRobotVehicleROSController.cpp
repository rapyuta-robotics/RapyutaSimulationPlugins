// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotVehicleROSController.h"

// UE
#include "Kismet/GameplayStatics.h"

// rclUE
#include "Msgs/ROS2TwistMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"
#include "Sensors/RR2DLidarComponent.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"
#include "Tools/UEUtilities.h"

ARRRobotVehicleROSController::ARRRobotVehicleROSController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    LidarComponentClass = URR2DLidarComponent::StaticClass();
}

void ARRRobotVehicleROSController::InitVehicleROS2Node(APawn* InPawn)
{
    if (nullptr == VehicleROS2Node)
    {
        VehicleROS2Node = GetWorld()->SpawnActor<AROS2Node>();
    }
    VehicleROS2Node->AttachToActor(InPawn, FAttachmentTransformRules::KeepRelativeTransform);
    VehicleROS2Node->Name = FString::Printf(TEXT("%s_ROS2Node"), *InPawn->GetName());
    VehicleROS2Node->Namespace = FString();
    VehicleROS2Node->Init();
}

bool ARRRobotVehicleROSController::InitPublishers(APawn* InPawn)
{
    if (false == IsValid(VehicleROS2Node))
    {
        return false;
    }

    // TFPublisher
    if (nullptr == TFPublisher)
    {
        TFPublisher = NewObject<URRROS2TFPublisher>(this);
        TFPublisher->SetupUpdateCallback();
    }
    TFPublisher->InitializeWithROS2(VehicleROS2Node);

    // OdomPublisher
    if (nullptr == OdomPublisher)
    {
        OdomPublisher = NewObject<URRROS2OdomPublisher>(this);
        OdomPublisher->SetupUpdateCallback();
    }
    OdomPublisher->InitializeWithROS2(VehicleROS2Node);
    OdomPublisher->RobotVehicle = Cast<ARobotVehicle>(InPawn);
    OdomPublisher->TFPublisher = TFPublisher;
    return true;
}

bool ARRRobotVehicleROSController::InitSensors(APawn* InPawn)
{
    if (false == IsValid(VehicleROS2Node))
    {
        return false;
    }

    TInlineComponentArray<UActorComponent*> lidarComponents;
    InPawn->GetComponents(LidarComponentClass, lidarComponents);
    for (auto& lidarComp : lidarComponents)
    {
        URRBaseLidarComponent* lidar = Cast<URRBaseLidarComponent>(lidarComp);
        if (lidar->IsValidLowLevel())
        {
            lidar->InitLidar(VehicleROS2Node);
        }
    }

    return true;
}

void ARRRobotVehicleROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // Track InPawn's initial pose
    InitialPosition = InPawn->GetActorLocation();
    InitialOrientation = InPawn->GetActorRotation();
    InitialOrientation.Yaw += 180.f;

    // Refresh ROS2Node
    InitVehicleROS2Node(InPawn);

    // Initialize Pawn's sensors (lidar, etc.)
    verify(InitSensors(InPawn));

    // Refresh TF, Odom publishers
    verify(InitPublishers(InPawn));
}

void ARRRobotVehicleROSController::OnUnPossess()
{
    TFPublisher->RevokeUpdateCallback();
    OdomPublisher->RevokeUpdateCallback();
    Super::OnUnPossess();
}

void ARRRobotVehicleROSController::SubscribeToMovementCommandTopic(const FString& InTopicName)
{
    // Subscription with callback to enqueue vehicle spawn info.
    if (ensure(IsValid(VehicleROS2Node)))
    {
        FSubscriptionCallback cb;
        cb.BindUObject(this, &ARRRobotVehicleROSController::MovementCallback);
        VehicleROS2Node->AddSubscription(InTopicName, UROS2TwistMsg::StaticClass(), cb);
    }
}

void ARRRobotVehicleROSController::MovementCallback(const UROS2GenericMsg* Msg)
{
    const UROS2TwistMsg* Concrete = Cast<UROS2TwistMsg>(Msg);

    if (IsValid(Concrete))
    {
        FROSTwist Output;
        Concrete->GetMsg(Output);
        const FVector linear(ConversionUtils::VectorROSToUE(Output.linear));
        const FVector angular(ConversionUtils::RotationROSToUE(Output.angular));

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
