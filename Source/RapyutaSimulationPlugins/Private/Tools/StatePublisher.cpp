// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/StatePublisher.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/SimulationState.h"

void UStatePublisher::RegisterPublisher(AROS2Node* Node)
{
    for (auto i = 0; i < StatesToPublish.Num(); i++)
    {
        Node->AddPublisher(this);
    }
}

void UStatePublisher::PublishState(UROS2GenericMsg* Msg)
{
    UROS2EntityStateMsg* StateMsg = Cast<UROS2EntityStateMsg>(Msg);
    if ((StateMsg != nullptr) && StatesToPublish.IsValidIndex(Idx))
    {
        StateMsg->SetMsg(StatesToPublish[Idx]);
        Idx = (Idx + 1 == StatesToPublish.Num()) ? 0 : Idx + 1;    // ue4 does not seem to have a modulo operator for integers
                                                                   // (it does have one for floats: FGenericPlatformMath::FMod)
        check(Idx < StatesToPublish.Num());
    }
}

void UStatePublisher::Bind()
{
    UpdateDelegate.BindDynamic(this, &UStatePublisher::PublishState);
}

void UStatePublisher::AddEntityToPublish(const FString& InName,
                                         const FVector& InPosition,
                                         const FRotator& InOrientation,
                                         const FString& InRefFrame)
{
    FROSEntityState BodyState;
    BodyState.name = InName;
    BodyState.pose_position_x = InPosition.X;
    BodyState.pose_position_y = -InPosition.Y;
    BodyState.pose_position_z = InPosition.Z;
    BodyState.pose_orientation = InOrientation.Quaternion();
    BodyState.pose_orientation.X = -BodyState.pose_orientation.X;
    BodyState.pose_orientation.Z = -BodyState.pose_orientation.Z;
    BodyState.reference_frame = InRefFrame;
    StatesToPublish.Add(BodyState);
}
