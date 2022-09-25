// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2TFPublisher.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "rclcUtilities.h"

URRROS2TFPublisher::URRROS2TFPublisher()
{
    PublicationFrequencyHz = 50;
    MsgClass = UROS2TFMsgMsg::StaticClass();
}

void URRROS2TFPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    TEnumAsByte<UROS2QoS> QoS;
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
    Init(QoS);
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
    auto stamp = UROS2Utils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));
    tfData.Header.Stamp.Sec = stamp.sec;
    tfData.Header.Stamp.Nanosec = stamp.nanosec;

    tfData.Header.FrameId = FrameId;
    tfData.ChildFrameId = ChildFrameId;

    FTransform transfROS = URRConversionUtils::TransformUEToROS(TF);
    tfData.Transform.Translation = transfROS.GetTranslation();
    tfData.Transform.Rotation = transfROS.GetRotation();

    tf.Transforms.Add(tfData);

    CastChecked<UROS2TFMsgMsg>(InMessage)->SetMsg(tf);
}
