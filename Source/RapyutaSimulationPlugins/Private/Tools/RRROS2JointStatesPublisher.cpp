// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2JointStatesPublisher.h"

// rclUE
#include "Msgs/ROS2JointStateMsg.h"

void URRROS2JointStatesPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    // (NOTE) Do not invoke [Super::], which configures its own MsgClass, TopicName, PublicationFrequencyHz
    UROS2Publisher::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2JointStateMsg::StaticClass();
    TopicName = TEXT("joint_states");
    PublicationFrequencyHz = 100;

    Init(UROS2QoS::SensorData);
}

void URRROS2JointStatesPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    // (NOTE) Robot could be reset when ROS AI Controller, which owns this publisher, unposses it.
    if (false == Robot.IsValid())
    {
        return;
    }
    CastChecked<UROS2JointStateMsg>(InMessage)->SetMsg(Robot->GetJointStates());
}
