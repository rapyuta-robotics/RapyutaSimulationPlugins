// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2TFPublisher.h"

void UROS2TFPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    UpdateDelegate.BindDynamic(this, &UROS2TFPublisher::UpdateTFMsg);
    Super::InitializeWithROS2(InROS2Node);

    TEnumAsByte<UROS2QoS> QoS;
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
    MsgClass = UROS2TFMsg::StaticClass();
    Init(QoS);
}

void UROS2TFPublisher::SetTransform(const FVector& Translation, const FQuat& Rotation)
{
    TF.SetTranslation(Translation);
    TF.SetRotation(Rotation);
}

void UROS2TFPublisher::UpdateTFMsg(UROS2GenericMsg* Message)
{
    TArray<FTFData> tfarray;

    FTFData tfdata;
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    tfdata.sec = (int32)TimeNow;
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    tfdata.nanosec = (uint32)(ns - (tfdata.sec * 1e+09));

    tfdata.frame_id = FrameId;
    tfdata.child_frame_id = ChildFrameId;

    FTransform TransROS = TF;
    TransROS.SetTranslation(ConversionUtils::VectorUEToROS(TF.GetTranslation()));
    TransROS.SetRotation(ConversionUtils::QuatUEToROS(TF.GetRotation()));

    tfdata.translation = TransROS.GetTranslation();
    tfdata.rotation = TransROS.GetRotation();

    tfarray.Add(tfdata);

    UROS2TFMsg* TFMessage = Cast<UROS2TFMsg>(Message);
    TFMessage->Update(tfarray);
}
