// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2TFPublisher.h"

// rclUE
#include "Msgs/ROS2TFMessageMsg.h"

URRROS2TFPublisher::URRROS2TFPublisher()
{
    PublicationFrequencyHz = 50;
    MsgClass = UROS2TFMessageMsg::StaticClass();
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
    TArray<FROSTFMessage> tfarray;

    FROSTFMessage tfdata;
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    tfdata.TransformsHeaderStampSec.Add((int32)TimeNow);
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    tfdata.TransformsHeaderStampNanosec.Add((uint32)(ns - (tfdata.TransformsHeaderStampSec.Last() * 1e+09)));

    tfdata.TransformsHeaderFrameId.Add(FrameId);
    tfdata.TransformsChildFrameId.Add(ChildFrameId);

    FTransform transfROS = URRConversionUtils::TransformUEToROS(TF);

    tfdata.TransformsTransformTranslation.Add(transfROS.GetTranslation());
    tfdata.TransformsTransformRotation.Add(transfROS.GetRotation());

    CastChecked<UROS2TFMessageMsg>(InMessage)->SetMsg(tfdata);
}
