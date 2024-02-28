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

void URRROS2TFPublisherBase::AddTFtoMsg(FROSTFMsg& OutROSTf,
                                        const FString& InFrameId,
                                        const FString& InChildFrameId,
                                        const FTransform& InTf)
{
    FROSTFStamped tfData;

    // time
    tfData.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));
    tfData.Header.FrameId = InFrameId;
    tfData.ChildFrameId = InChildFrameId;

    tfData.Transform = URRConversionUtils::TransformUEToROS(InTf);

    OutROSTf.Transforms.Emplace(MoveTemp(tfData));
}

void URRROS2TFPublisherBase::AddTFtoMsg(FROSTFMsg& OutROSTf, URRROS2TFComponent* InTfc)
{
    AddTFtoMsg(OutROSTf, InTfc->FrameId, InTfc->ChildFrameId, InTfc->GetTF());
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

void URRROS2TFPublisher::GetROS2Msg(FROSTFMsg& OutROSTf)
{
    AddTFtoMsg(OutROSTf, FrameId, ChildFrameId, TF);
    Super::GetROS2Msg(OutROSTf);
}

void URRROS2TFsPublisher::GetROS2Msg(FROSTFMsg& OutROSTf)
{
    for (auto& tfc : TFComponents)
    {
        AddTFtoMsg(OutROSTf, tfc);
    }
    Super::GetROS2Msg(OutROSTf);
}

FTransform URRROS2LinksTFComponent::GetTF()
{
    if (ParentLink != nullptr && ChildLink != nullptr)
    {
        TF = URRGeneralUtils::GetRelativeTransform(ParentLink->GetComponentTransform(), ChildLink->GetComponentTransform());
    }
    return Super::GetTF();
}

void URRROS2LinksTFComponent::AddLinks(UPrimitiveComponent* InParentLink,
                                       UPrimitiveComponent* InChildLink,
                                       const FString& InFrameId,
                                       const FString& InChildFrameId,
                                       URRROS2TFsPublisher* OutTFsPublisher)
{
    URRROS2LinksTFComponent* linksTF = NewObject<URRROS2LinksTFComponent>(OutTFsPublisher);
    linksTF->InitLinksTFComponent(InFrameId, InChildFrameId, InParentLink, InChildLink);
    OutTFsPublisher->TFComponents.Add(linksTF);
}

void URRROS2PhysicsConstraintTFComponent::AddConstraint(UPhysicsConstraintComponent* InConstraint,
                                                        const FString& InFrameId,
                                                        const FString& InChildFrameId,
                                                        URRROS2TFsPublisher* OutTFsPublisher)
{
    URRROS2PhysicsConstraintTFComponent* constraintTF = NewObject<URRROS2PhysicsConstraintTFComponent>(OutTFsPublisher);
    constraintTF->InitPhysicsConstraintTFComponent(InFrameId, InChildFrameId, InConstraint);
    OutTFsPublisher->TFComponents.Add(constraintTF);
}

FTransform URRROS2JointTFComponent::GetTF()
{
    if (Joint != nullptr)
    {
        TF.SetTranslation(Joint->Position);
        TF.SetRotation(Joint->Orientation.Quaternion());
        return Joint->GetJointToChildLink() * TF * Joint->GetParentLinkToJoint();
    }
    return TF;
}

void URRROS2JointTFComponent::AddJoint(URRJointComponent* InJoint,
                                       const FString& InFrameId,
                                       const FString& InChildFrameId,
                                       URRROS2TFsPublisher* OutTFsPublisher)
{
    URRROS2JointTFComponent* jointTF = NewObject<URRROS2JointTFComponent>(OutTFsPublisher);
    jointTF->InitJointTFComponent(InFrameId, InChildFrameId, InJoint);
    OutTFsPublisher->TFComponents.Add(jointTF);
}
