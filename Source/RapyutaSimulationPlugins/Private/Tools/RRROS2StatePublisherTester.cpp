// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2StatePublisherTester.h"

ARRROS2StatePublisherTester::ARRROS2StatePublisherTester()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ARRROS2StatePublisherTester::BeginPlay()
{
    Super::BeginPlay();

    ROS2Node = NewObject<UROS2NodeComponent>(this);
    ROS2Node->Name = FString::Printf(TEXT("%s_UE4ROS2Node"), *GetName());
    ROS2Node->Namespace = FString();
    ROS2Node->Init();

    StatePub = NewObject<URRROS2StatePublisher>(this);
    StatePub->InitializeWithROS2(ROS2Node);
    StatePub->Robot = RobotVehicle;

#if 0
    FROSEntityState BodyState;
    BodyState.name = TEXT("Body");
    BodyState.position = FVector::ZeroVector;
    BodyState.orientation = FQuat::Identity;
    BodyState.reference_frame = TEXT("Body");
    StatePub->StatesToPublish.Add(BodyState);

    FROSEntityState LWheelState;
    LWheelState.name = TEXT("LeftWheel");
    LWheelState.position = FVector::XAxisVector;
    LWheelState.orientation = FQuat::Identity;
    LWheelState.reference_frame = TEXT("Body");
    StatePub->StatesToPublish.Add(LWheelState);

    FROSEntityState RWheelState;
    RWheelState.name = TEXT("RightWheel");
    RWheelState.position = -FVector::XAxisVector;
    RWheelState.orientation = FQuat::Identity;
    RWheelState.reference_frame = TEXT("Body");
    StatePub->StatesToPublish.Add(RWheelState);
#endif
}

void ARRROS2StatePublisherTester::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // modify RWheel
    // StatePub->StatesToPublish[2].position -= FVector::XAxisVector;
}
