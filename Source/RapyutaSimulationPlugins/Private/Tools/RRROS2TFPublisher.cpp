// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2TFPublisher.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "rclcUtilities.h"

URRROS2TFPublisher::URRROS2TFPublisher()
{
    PublicationFrequencyHz = 50;
    MsgClass = UROS2TFMsgMsg::StaticClass();
    SetDefaultDelegates();    //use UpdateMessage as update delegate
}

bool URRROS2TFPublisher::InitializeWithROS2(UROS2NodeComponent* InROS2Node)
{
    // (NOTE) [/tf, /tf_static] has its [tf_prefix] only for frame ids, not topics
    if (IsStatic)
    {
        TopicName = TEXT("/tf_static");
        QoS = UROS2QoS::StaticBroadcaster;
    }
    else
    {
        TopicName = TEXT("/tf");
        QoS = UROS2QoS::DynamicBroadcaster;
    }
    return Super::InitializeWithROS2(InROS2Node);
}

void URRROS2TFPublisher::SetTransform(const FVector& Translation, const FQuat& Rotation)
{
    TF.SetTranslation(Translation);
    TF.SetRotation(Rotation);
}

void URRROS2TFPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    FROSTFMsg tf;

    FROSTFStamped tfData;

    // time
    tfData.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));
    tfData.Header.FrameId = FrameId;
    tfData.ChildFrameId = ChildFrameId;

    tfData.Transform = URRConversionUtils::TransformUEToROS(TF);

    tf.Transforms.Emplace(MoveTemp(tfData));

    CastChecked<UROS2TFMsgMsg>(InMessage)->SetMsg(tf);
}
