// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2PhysicsConstraintTFPublisher.h"

bool URRROS2PhysicsConstraintTFPublisher::InitializeWithROS2(UROS2NodeComponent* InROS2Node)
{
    if (Constraint != nullptr)
    {
        UPrimitiveComponent* ParentLink = URRGeneralUtils::GetPhysicsConstraintComponent(Constraint, EConstraintFrame::Frame1);
        UPrimitiveComponent* ChildLink = URRGeneralUtils::GetPhysicsConstraintComponent(Constraint, EConstraintFrame::Frame2);
        if (ParentLink && ChildLink)
        {
            // set joints relations and save initial parent to joint transformation.
            JointToChildLink =
                URRGeneralUtils::GetRelativeTransform(Constraint->GetComponentTransform(), ChildLink->GetComponentTransform());
            ParentLinkToJoint =
                URRGeneralUtils::GetRelativeTransform(ParentLink->GetComponentTransform(), Constraint->GetComponentTransform());

            InitialJointTF = URRGeneralUtils::GetPhysicsConstraintTransform(Constraint, FTransform::Identity);
        }
    }
    else
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(LogRapyutaCore, Error, TEXT("Physics Constraint is not valid."));
    }

    return Super::InitializeWithROS2(InROS2Node);
}

void URRROS2PhysicsConstraintTFPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (Constraint == nullptr)
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(LogRapyutaCore, Error, TEXT("Physics Constraint is not valid, StopPublisher"));
        StopPublishTimer();
        return;
    }

    JointTF = URRGeneralUtils::GetPhysicsConstraintTransform(Constraint, InitialJointTF);

    Super::UpdateMessage(InMessage);
}
