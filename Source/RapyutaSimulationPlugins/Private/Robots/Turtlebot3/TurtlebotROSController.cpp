// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/TurtlebotROSController.h"

#include "Kismet/GameplayStatics.h"

#include <Drives/RobotVehicleMovementComponent.h>
#include <Tools/UEUtilities.h>

ATurtlebotROSController::ATurtlebotROSController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    LidarClass = ASensorLidar::StaticClass();
}

void ATurtlebotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // Initialize ROS2Node
    if (TurtleNode == nullptr)
    {
        FActorSpawnParameters SpawnParamsNode;
        TurtleNode = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
        TurtleNode->SetActorLocation(InPawn->GetActorLocation());
        TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
        TurtleNode->Name = TEXT("UE4Node_" + FGuid::NewGuid().ToString());
        TurtleNode->Namespace = FString();
        TurtleNode->Init();
    }

    // Initialize Lidars attached to Pawn
    TArray<AActor*> ChildrenActors;
    InPawn->GetAttachedActors(ChildrenActors);
    for (int32 Idx = 0; Idx < ChildrenActors.Num(); Idx++)
    {
        if (ChildrenActors[Idx]->GetClass() == ASensorLidar::StaticClass())
        {
            ASensorLidar* TurtleLidar = Cast<ASensorLidar>(ChildrenActors[Idx]);
            TurtleLidar->InitToNode(TurtleNode);
            TurtleLidar->Run();
        }
    }

    // TFPublisher
    URobotVehicleMovementComponent* RobotVehicleMovementComponent =
        Cast<URobotVehicleMovementComponent>(InPawn->GetMovementComponent());
    TFPublisher = NewObject<UROS2TFPublisher>(this, UROS2TFPublisher::StaticClass());
    TFPublisher->RegisterComponent();
    TFPublisher->FrameId = RobotVehicleMovementComponent->FrameId = TEXT("odom");
    TFPublisher->ChildFrameId = RobotVehicleMovementComponent->ChildFrameId = TEXT("base_footprint");
    TFPublisher->PublicationFrequencyHz = 50;
    TFPublisher->InitializeWithROS2(TurtleNode);

    // OdomPublisher
    OdomPublisher = NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
    OdomPublisher->RegisterComponent();
    OdomPublisher->TopicName = TEXT("odom");
    OdomPublisher->PublicationFrequencyHz = 30;
    OdomPublisher->MsgClass = UROS2OdometryMsg::StaticClass();
    OdomPublisher->UpdateDelegate.BindDynamic(this, &ATurtlebotROSController::OdomMessageUpdate);
    TurtleNode->AddPublisher(OdomPublisher);
    OdomPublisher->Init(UROS2QoS::KeepLast);

    if (Turtlebot != nullptr)
    {
        InitialPosition = Turtlebot->GetActorLocation();
        InitialOrientation = Turtlebot->GetActorRotation();
        InitialOrientation.Yaw += 180;

        // Subscribe cmd_vel
        SetupCommandTopicSubscription(Turtlebot);
    }
}

void ATurtlebotROSController::OnUnPossess()
{
    // TurtleLidar = nullptr;
    TurtleNode = nullptr;
    TFPublisher = nullptr;
    OdomPublisher = nullptr;

    Super::OnUnPossess();
}

void ATurtlebotROSController::SetPawn(APawn* InPawn)
{
    Super::SetPawn(InPawn);

    Turtlebot = Cast<ARobotVehicle>(InPawn);
}

void ATurtlebotROSController::SetupCommandTopicSubscription(ARobotVehicle* InPawn)
{
    if (IsValid(InPawn))
    {
        // Subscription with callback to enqueue vehicle spawn info.
        if (ensure(IsValid(TurtleNode)))
        {
            FSubscriptionCallback cb;
            cb.BindDynamic(this, &ATurtlebotROSController::MovementCallback);
            TurtleNode->AddSubscription(TEXT("cmd_vel"), UROS2TwistMsg::StaticClass(), cb);
        }
    }
}

void ATurtlebotROSController::OdomMessageUpdate(UROS2GenericMsg* TopicMessage)
{
    FROSOdometry odomData;
    bool bIsOdomDataValid = GetOdomData(odomData);
    if (bIsOdomDataValid)
    {
        UROS2OdometryMsg* odomMessage = CastChecked<UROS2OdometryMsg>(TopicMessage);
        odomMessage->SetMsg(odomData);
    }
}

void ATurtlebotROSController::MovementCallback(const UROS2GenericMsg* Msg)
{
    const UROS2TwistMsg* Concrete = Cast<UROS2TwistMsg>(Msg);

    if (IsValid(Concrete))
    {
        FROSTwist Output;
        Concrete->GetMsg(Output);
        const FVector linear(ConversionUtils::VectorROSToUE(Output.linear));
        const FVector angular(ConversionUtils::RotationROSToUE(Output.angular));

        AsyncTask(ENamedThreads::GameThread,
                  [this, linear, angular]
                  {
                      if (IsValid(Turtlebot))
                      {
                          Turtlebot->SetLinearVel(linear);
                          Turtlebot->SetAngularVel(angular);
                      }
                  });
    }
}

bool ATurtlebotROSController::GetOdomData(FROSOdometry& OutOdomData) const
{
    ARobotVehicle* Vehicle = Turtlebot;
    const URobotVehicleMovementComponent* moveComponent = Vehicle->RobotVehicleMoveComponent;
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
