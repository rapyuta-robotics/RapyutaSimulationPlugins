// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2StatePublisher.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/SimulationState.h"

void URRROS2StatePublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    RegisterToROS2Node(InROS2Node);

    MsgClass = UROS2EntityStateMsg::StaticClass();
    TopicName = TEXT("state");
    PublicationFrequencyHz = 100;

    // [URRROS2StatePublisher] must have been already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::DynamicBroadcaster);
}

void URRROS2StatePublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if ((++Idx) >= StatesToPublish.Num())
    {
        Idx = 0;
    }
    CastChecked<UROS2EntityStateMsg>(InMessage)->SetMsg(StatesToPublish[Idx]);
}

void URRROS2StatePublisher::AddEntityToPublish(const FString& InName,
                                               const FVector& InPosition,
                                               const FRotator& InOrientation,
                                               const FString& InRefFrame)
{
    FROSEntityState BodyState;
    BodyState.name = InName;
    BodyState.pose_position_x = InPosition.X;
    BodyState.pose_position_y = -InPosition.Y;
    BodyState.pose_position_z = InPosition.Z;
    BodyState.pose_orientation = InOrientation.Quaternion();
    BodyState.pose_orientation.X = -BodyState.pose_orientation.X;
    BodyState.pose_orientation.Z = -BodyState.pose_orientation.Z;
    BodyState.reference_frame = InRefFrame;
    StatesToPublish.Emplace(MoveTemp(BodyState));
}
