// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2StatePublisher.h"

// rclUE
#include "ROS2NodeComponent.h"

// RapyutaSimulationPlugins
#include "Tools/SimulationState.h"

URRROS2StatePublisher::URRROS2StatePublisher()
{
    MsgClass = UROS2EntityStateMsg::StaticClass();
    TopicName = TEXT("state");
    PublicationFrequencyHz = 100;
    QoS = UROS2QoS::DynamicBroadcaster;
    SetDefaultDelegates();    //use UpdateMessage as update delegate
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
    BodyState.Pose.Position = URRConversionUtils::ConvertHandedness(InPosition);
    BodyState.Pose.Orientation = URRConversionUtils::QuatUEToROS(InOrientation.Quaternion());
    BodyState.ReferenceFrame = InRefFrame;
    StatesToPublish.Emplace(MoveTemp(BodyState));
}
