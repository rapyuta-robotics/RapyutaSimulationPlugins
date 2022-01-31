// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2TFPublisher.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"

void URRROS2TFPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2TFMsg::StaticClass();
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
    PublicationFrequencyHz = 50;
    Init(QoS);
}

void URRROS2TFPublisher::SetTransform(const FVector& Translation, const FQuat& Rotation)
{
    TF.SetTranslation(Translation);
    TF.SetRotation(Rotation);
}

void URRROS2TFPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    TArray<FTFData> tfarray;

    FTFData tfdata;
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    tfdata.sec = (int32)TimeNow;
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    tfdata.nanosec = (uint32)(ns - (tfdata.sec * 1e+09));

    tfdata.frame_id = FrameId;
    tfdata.child_frame_id = ChildFrameId;

    FTransform TransROS = ConversionUtils::TransformUEToROS(TF);

    tfdata.translation = TransROS.GetTranslation();
    tfdata.rotation = TransROS.GetRotation();

    tfarray.Add(tfdata);
    CastChecked<UROS2TFMsg>(InMessage)->Update(tfarray);
}
