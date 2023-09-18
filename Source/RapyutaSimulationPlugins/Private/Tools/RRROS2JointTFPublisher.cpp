// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2JointTFPublisher.h"

void URRROS2JointTFPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    TF = JointToChildLink * JointTF * ParentLinkToJoint;

    Super::UpdateMessage(InMessage);
}
