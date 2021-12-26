// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotVehicleROSController.h"

// UE
#include "Kismet/GameplayStatics.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RobotVehicle.h"
#include "Sensors/RR2DLidarComponent.h"
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
    VehicleROS2Node->SetActorLocation(InPawn->GetActorLocation());
    VehicleROS2Node->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
    VehicleROS2Node->Name = FString::Printf(TEXT("%s_ROS2Node"), *InPawn->GetName());
    VehicleROS2Node->Namespace = FString();
    VehicleROS2Node->Init();
}

void ARRRobotVehicleROSController::InitPublishers(APawn* InPawn)
{
    check(VehicleROS2Node);

    // TFPublisher
    if (nullptr == TFPublisher)
    {
        TFPublisher = NewObject<UROS2TFPublisher>(this);
        TFPublisher->RegisterComponent();

        URobotVehicleMovementComponent* vehicleMovementComponent =
            CastChecked<URobotVehicleMovementComponent>(InPawn->GetMovementComponent());
        TFPublisher->FrameId = vehicleMovementComponent->FrameId = TEXT("odom");
        TFPublisher->ChildFrameId = vehicleMovementComponent->ChildFrameId = TEXT("base_footprint");
        TFPublisher->PublicationFrequencyHz = 50;
    }
    TFPublisher->InitializeWithROS2(VehicleROS2Node);

    // OdomPublisher
    if (nullptr == OdomPublisher)
    {
        OdomPublisher = NewObject<UROS2Publisher>(this);
        OdomPublisher->RegisterComponent();
        OdomPublisher->TopicName = TEXT("odom");
        OdomPublisher->PublicationFrequencyHz = 30;
        OdomPublisher->MsgClass = UROS2OdometryMsg::StaticClass();

        OdomPublisher->UpdateDelegate.BindDynamic(this, &ARRRobotVehicleROSController::OdomMessageUpdate);
        VehicleROS2Node->AddPublisher(OdomPublisher);
        OdomPublisher->Init(UROS2QoS::KeepLast);
    }
}

void ARRRobotVehicleROSController::SubscribeToMovementCommandTopic(const FString& InTopicName)
{
    // Subscription with callback to enqueue vehicle spawn info.
    if (ensure(IsValid(VehicleROS2Node)))
    {
        FSubscriptionCallback cb;
        cb.BindDynamic(this, &ARRRobotVehicleROSController::MovementCallback);
        VehicleROS2Node->AddSubscription(InTopicName, UROS2TwistMsg::StaticClass(), cb);
    }
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

    // Initialize Pawn's Lidar components
    TInlineComponentArray<UActorComponent*> lidarComponents;
    InPawn->GetComponents(LidarComponentClass, lidarComponents);
    for (auto& lidarComp : lidarComponents)
    {
        auto* lidar = Cast<URRBaseLidarComponent>(lidarComp);
        if (lidar)
        {
            lidar->InitLidar(VehicleROS2Node);
        }
    }

    // Initialize TF, Odom publishers
    InitPublishers(InPawn);

    // Subscribe to [cmd_vel]
    SubscribeToMovementCommandTopic(TEXT("cmd_vel"));
}

void ARRRobotVehicleROSController::OnUnPossess()
{
    TFPublisher->UpdateDelegate.RemoveDynamic(TFPublisher, &UROS2TFPublisher::UpdateTFMsg);
    OdomPublisher->UpdateDelegate.RemoveDynamic(this, &ARRRobotVehicleROSController::OdomMessageUpdate);

    Super::OnUnPossess();
}

void ARRRobotVehicleROSController::OdomMessageUpdate(UROS2GenericMsg* TopicMessage)
{
    FROSOdometry odomData;
    bool bIsOdomDataValid = GetOdomData(odomData);
    if (bIsOdomDataValid)
    {
        UROS2OdometryMsg* odomMessage = CastChecked<UROS2OdometryMsg>(TopicMessage);
        odomMessage->SetMsg(odomData);
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

        // (Note) In this callback, which is invoked from ROS, the ROSController itself (this) could have been garbage collected,
        // thus any direct referencing to its member in the lambda needs to be verified.
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

bool ARRRobotVehicleROSController::GetOdomData(FROSOdometry& OutOdomData) const
{
    ARobotVehicle* vehicle = Cast<ARobotVehicle>(GetPawn());
    const URobotVehicleMovementComponent* moveComponent = vehicle ? vehicle->RobotVehicleMoveComponent : nullptr;
    if (moveComponent && moveComponent->OdomData.IsValid())
    {
        TFPublisher->TF = moveComponent->GetOdomTF();
        OutOdomData = ConversionUtils::OdomUEToROS(moveComponent->OdomData);
        return true;
    }
    else
    {
        return false;
    }
}
