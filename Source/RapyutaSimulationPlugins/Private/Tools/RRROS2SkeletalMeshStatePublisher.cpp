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
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2EntityStateMsg::StaticClass();
    TopicName = TEXT("ue_ros/model_state");
    PublicationFrequencyHz = 10;

    // The publisher itself must have been already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::DynamicBroadcaster);

    // Local transforms: GetBoneSpaceTransforms()
    // seems empty: GetCachedComponentSpaceTransforms()

    verify(IsValid(Robot));
    TInlineComponentArray<USkeletalMeshComponent*> skeletalMeshComponents;
    Robot->GetComponents(skeletalMeshComponents, false);
    check(skeletalMeshComponents.Num() == 1);
    SkeletalMeshComp = skeletalMeshComponents[0];

    UE_LOG(LogTemp, Warning, TEXT("Skeletal mesh has %d bones"), SkeletalMeshComp->GetBoneSpaceTransforms().Num());
    check(SkeletalMeshComp->GetBoneSpaceTransforms().Num() > 0);
}

void URRROS2SkeletalMeshStatePublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
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
