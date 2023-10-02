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
    // JointTFComponents.Emplace(InJoint->GetJointToChildLink(), InJoint->GetParentLinkToJoint());
    URRROS2JointTFComponent jointTF(InFrameId, InChildFrameId, InJoint->GetJointToChildLink(), InJoint->GetParentLinkToJoint());
    TFComponents.Add(&jointTF);
}
