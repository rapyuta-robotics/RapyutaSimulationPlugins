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

void URRROS2SkeletalMeshStatePublisher::SetTargetRobot(AActor* InRobot)
{
    Super::SetTargetRobot(InRobot);
    // Local transforms: GetBoneSpaceTransforms()
    // seems empty: GetCachedComponentSpaceTransforms()

    TInlineComponentArray<USkeletalMeshComponent*> skeletalMeshComponents;
    InRobot->GetComponents(skeletalMeshComponents, false);
    if (skeletalMeshComponents.Num() > 0)
    {
        SkeletalMeshComp = skeletalMeshComponents[0];
        UE_LOG(LogRapyutaCore,
               Log,
               TEXT("[%s] has %d bones"),
               *SkeletalMeshComp->GetName(),
               SkeletalMeshComp->GetBoneSpaceTransforms().Num());
    }
}

void URRROS2SkeletalMeshStatePublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (nullptr == SkeletalMeshComp)
    {
        return;
    }

    UROS2EntityStateMsg* stateMsg = CastChecked<UROS2EntityStateMsg>(InMessage);
    ARobotVehicle* vehicle = Cast<ARobotVehicle>(Robot);
    const TArray<FTransform> boneTransforms = SkeletalMeshComp->GetBoneSpaceTransforms();
    if (boneTransforms.IsValidIndex(Idx))
    {
        FTransform elementTransform = boneTransforms[Idx];
        FTransform robotTransform = Robot->GetTransform();
        FROSEntityState data;

        data.name = RobotUniqueName + TEXT("/base_footprint");
        data.pose_position_x = 0.01f * robotTransform.GetLocation().X;
        data.pose_position_y = 0.01f * robotTransform.GetLocation().Y;
        data.pose_position_z = 0.01f * robotTransform.GetLocation().Z;
        data.pose_orientation = robotTransform.GetRotation();
        if ((vehicle != nullptr) && (vehicle->Map != nullptr))
        {
            const FQuat mapInverseQuat = vehicle->Map->GetActorQuat().Inverse();
            const FVector mapPos =
                0.01f * mapInverseQuat.RotateVector(robotTransform.GetLocation() - vehicle->Map->GetActorLocation());
            data.pose_position_x = mapPos.X;
            data.pose_position_y = mapPos.Y;
            data.pose_position_z = mapPos.Z;
            data.pose_orientation = robotTransform.GetRotation() * mapInverseQuat;
        }
        data.pose_position_y = -data.pose_position_y;
        data.pose_orientation.X = -data.pose_orientation.X;
        data.pose_orientation.Z = -data.pose_orientation.Z;
        data.twist_linear = FVector::ZeroVector;
        data.twist_angular = FVector::ZeroVector;
        data.reference_frame = RobotUniqueName + TEXT("/map");
        stateMsg->SetMsg(data);
        if ((++Idx) >= StatesToPublish.Num())
        {
            Idx = 0;
        }
        // DrawDebugDirectionalArrow(GetWorld(), data.position, data.position + data.orientation.GetForwardVector()*100, 100,
        // FColor(255, 0, 0, 255), false, 10, 1, 10);
    }
}
