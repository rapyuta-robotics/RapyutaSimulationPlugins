// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2SkeletalMeshStatePublisher.h"

// UE
#include "Components/SkeletalMeshComponent.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"

// rclUE
#include "ROS2Node.h"
//#include "DrawDebugHelpers.h"

void URRROS2SkeletalMeshStatePublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    TopicName = TEXT("ue_ros/model_state");
    PublicationFrequencyHz = 10;
    Super::InitializeWithROS2(InROS2Node);
}

void URRROS2SkeletalMeshStatePublisher::SetTargetRobot(ARobotVehicle* InRobot)
{
    Super::SetTargetRobot(InRobot);
    // Local transforms: GetBoneSpaceTransforms()
    // seems empty: GetCachedComponentSpaceTransforms()

    TInlineComponentArray<USkeletalMeshComponent*> skeletalMeshComponents;
    InRobot->GetComponents(skeletalMeshComponents, false);
    if (skeletalMeshComponents.Num() > 0)
    {
        SkeletalMeshComp = skeletalMeshComponents[0];
        const auto bonesNum = SkeletalMeshComp->GetBoneSpaceTransforms().Num();
        UE_LOG(LogRapyutaCore, Log, TEXT("[%s] has %d bones"), *SkeletalMeshComp->GetName(), bonesNum);
        check(bonesNum > 0);
    }
    else
    {
        SkeletalMeshComp = nullptr;
        UE_LOG(LogRapyutaCore, Fatal, TEXT("[%s] No child SkeletalMeshComp found!"), *SkeletalMeshComp->GetName());
    }
}

void URRROS2SkeletalMeshStatePublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    // (NOTE) Robot could be reset when ROS AI Controller, which owns this publisher, unposses it.
    if ((false == Robot.IsValid()) || (nullptr == SkeletalMeshComp))
    {
        return;
    }

    UROS2EntityStateMsg* stateMsg = CastChecked<UROS2EntityStateMsg>(InMessage);

#if 0    // To be enabled when bone transform is used in the message content
    const TArray<FTransform> boneTransforms = SkeletalMeshComp->GetBoneSpaceTransforms();
    if (boneTransforms.IsValidIndex(Idx))
    {
#endif
    const FTransform& robotTransform = Robot->GetTransform();

    FROSEntityState data;
    data.name = FrameId;

    const FVector& robotLocation = robotTransform.GetLocation();
    const FQuat& robotRotation = robotTransform.GetRotation();
    data.pose_position_x = 0.01f * robotLocation.X;
    data.pose_position_y = 0.01f * robotLocation.Y;
    data.pose_position_z = 0.01f * robotLocation.Z;
    data.pose_orientation = robotRotation;
    if (Robot->Map != nullptr)
    {
        const FQuat mapInverseQuat = Robot->Map->GetActorQuat().Inverse();
        const FVector mapPos = 0.01f * mapInverseQuat.RotateVector(robotLocation - Robot->Map->GetActorLocation());
        data.pose_position_x = mapPos.X;
        data.pose_position_y = mapPos.Y;
        data.pose_position_z = mapPos.Z;
        data.pose_orientation = robotRotation * mapInverseQuat;
    }
    data.pose_position_y = -data.pose_position_y;
    data.pose_orientation.X = -data.pose_orientation.X;
    data.pose_orientation.Z = -data.pose_orientation.Z;
    data.twist_linear = FVector::ZeroVector;
    data.twist_angular = FVector::ZeroVector;
    data.reference_frame = ReferenceFrameId;
    stateMsg->SetMsg(data);

#if 0
    if ((++Idx) >= boneTransforms.Num())
    {
        Idx = 0;
    }
}
#endif
    // DrawDebugDirectionalArrow(GetWorld(), data.position, data.position + data.orientation.GetForwardVector()*100, 100,
    // FColor(255, 0, 0, 255), false, 10, 1, 10);
}
