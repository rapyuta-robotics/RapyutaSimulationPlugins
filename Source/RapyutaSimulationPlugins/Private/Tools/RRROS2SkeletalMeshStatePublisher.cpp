// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2SkeletalMeshStatePublisher.h"

// UE
#include "Components/SkeletalMeshComponent.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"

// rclUE
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"
//#include "DrawDebugHelpers.h"

URRROS2SkeletalMeshStatePublisher::URRROS2SkeletalMeshStatePublisher()
{
    MsgClass = UROS2EntityStateMsg::StaticClass();
    TopicName = TEXT("ue_ros/model_state");
    PublicationFrequencyHz = 10;
    QoS = UROS2QoS::DynamicBroadcaster;
    SetDefaultDelegates();    //use UpdateMessage as update delegate
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
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("[%s] has %d bones"), *SkeletalMeshComp->GetName(), bonesNum);
    }
    else
    {
        SkeletalMeshComp = nullptr;
        UE_LOG_WITH_INFO(LogRapyutaCore, Fatal, TEXT("[%s] No child SkeletalMeshComp found!"), *SkeletalMeshComp->GetName());
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
    data.Name = FrameId;

    const FVector& robotLocation = robotTransform.GetLocation();
    const FQuat& robotRotation = robotTransform.GetRotation();
    data.Pose.Position = 0.01f * robotLocation;
    data.Pose.Orientation = robotRotation;
    if (Map != nullptr)
    {
        const FQuat mapInverseQuat = Map->GetActorQuat().Inverse();
        const FVector mapPos = 0.01f * mapInverseQuat.RotateVector(robotLocation - Map->GetActorLocation());
        data.Pose.Position = mapPos;
        data.Pose.Orientation = robotRotation * mapInverseQuat;
    }
    data.Pose.Position.Y = -data.Pose.Position.Y;
    data.Pose.Orientation.X = -data.Pose.Orientation.X;
    data.Pose.Orientation.Z = -data.Pose.Orientation.Z;
    data.Twist.Linear = FVector::ZeroVector;
    data.Twist.Angular = FVector::ZeroVector;
    data.ReferenceFrame = ReferenceFrameId;
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
