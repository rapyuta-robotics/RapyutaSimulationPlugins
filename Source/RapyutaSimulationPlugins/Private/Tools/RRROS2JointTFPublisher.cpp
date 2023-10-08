// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2JointTFPublisher.h"

FTransform URRROS2JointTFComponentBase::GetTF()
{
    return JointToChildLink * TF * ParentLinkToJoint;
}

FTransform URRROS2JointTFComponent::GetTF()
{
    if (Joint != nullptr)
    {
        TF.SetTranslation(Joint->Position);
        TF.SetRotation(Joint->Orientation.Quaternion());
    }
    return Super::GetTF();
}

void URRROS2JointsTFPublisher::AddJoint(URRJointComponent* InJoint, FString InFrameId, FString InChildFrameId)
{
    URRROS2JointTFComponent* jointTF = NewObject<URRROS2JointTFComponent>(this);
    jointTF->Init(InFrameId, InChildFrameId, InJoint->GetParentLinkToJoint(), InJoint->GetJointToChildLink(), InJoint);
    TFComponents.Add(jointTF);
}
