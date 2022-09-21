// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2StatePublisher.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/SimulationState.h"

void URRROS2StatePublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2EntityStateMsg::StaticClass();
    TopicName = TEXT("state");
    PublicationFrequencyHz = 100;

    // [URRROS2StatePublisher] must have been already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::DynamicBroadcaster);
}

void URRROS2StatePublisher::SetTargetRobot(ARobotVehicle* InRobot)
{
    verify(IsValid(InRobot));
    Robot = InRobot;
}

void URRROS2StatePublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (StatesToPublish.IsValidIndex(Idx))
    {
        CastChecked<UROS2EntityStateMsg>(InMessage)->SetMsg(StatesToPublish[Idx]);
    }

    if ((++Idx) >= StatesToPublish.Num())
    {
        Idx = 0;
    }
}

void URRROS2StatePublisher::AddEntityToPublish(const FString& InName,
                                               const FVector& InPosition,
                                               const FRotator& InOrientation,
                                               const FString& InRefFrame)
{
    FROSEntityState BodyState;
    BodyState.Name = InName;
    BodyState.PosePosition.X = InPosition.X;
    BodyState.PosePosition.Y = -InPosition.Y;
    BodyState.PosePosition.Z = InPosition.Z;
    BodyState.PoseOrientation = InOrientation.Quaternion();
    BodyState.PoseOrientation.X = -BodyState.PoseOrientation.X;
    BodyState.PoseOrientation.Z = -BodyState.PoseOrientation.Z;
    BodyState.ReferenceFrame = InRefFrame;
    StatesToPublish.Emplace(MoveTemp(BodyState));
}
