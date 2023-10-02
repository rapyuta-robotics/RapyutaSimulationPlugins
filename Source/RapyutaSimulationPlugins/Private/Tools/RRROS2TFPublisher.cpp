// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2TFPublisher.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "rclcUtilities.h"

URRROS2TFPublisherBase::URRROS2TFPublisherBase()
{
    PublicationFrequencyHz = 10;
    MsgClass = UROS2TFMsgMsg::StaticClass();
    SetDefaultDelegates();    //use UpdateMessage as update delegate
}

bool URRROS2TFPublisherBase::InitializeWithROS2(UROS2NodeComponent* InROS2Node)
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

void URRROS2TFPublisherBase::AddTFtoMsg(FROSTFMsg& tf,
                                        const FString InFrameId,
                                        const FString InChildFrameId,
                                        const FTransform& InTF)
{
    FROSTFStamped tfData;

    // time
    tfData.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));
    tfData.Header.FrameId = InFrameId;
    tfData.ChildFrameId = InChildFrameId;

    tfData.Transform = URRConversionUtils::TransformUEToROS(InTF);

    tf.Transforms.Emplace(MoveTemp(tfData));
}

void URRROS2TFPublisherBase::UpdateMessage(UROS2GenericMsg* InMessage)
{
    FROSTFMsg tf;

    GetROS2Msg(tf);

    CastChecked<UROS2TFMsgMsg>(InMessage)->SetMsg(tf);
}

void URRROS2TFPublisher::SetTransform(const FVector& Translation, const FQuat& Rotation)
{
    TF.SetTranslation(Translation);
    TF.SetRotation(Rotation);
}

void URRROS2TFPublisher::GetROS2Msg(FROSTFMsg& tf)
{
    AddTFtoMsg(tf, FrameId, ChildFrameId, TF);
    Super::GetROS2Msg(tf);
}

void URRROS2TFsPublisher::GetROS2Msg(FROSTFMsg& tf)
{
    for (auto& tfc : TFComponents)
    {
        AddTFtoMsg(tf, tfc->FrameId, tfc->ChildFrameId, tfc->GetTF());
    }
    Super::GetROS2Msg(tf);
}
