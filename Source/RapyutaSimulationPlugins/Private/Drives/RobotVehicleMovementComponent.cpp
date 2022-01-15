// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RobotVehicleMovementComponent.h"

// UE
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetMathLibrary.h"

void URobotVehicleMovementComponent::Initialize()
{    
    GaussianRNGPosition = std::normal_distribution<>{NoiseMeanPos, NoiseVariancePos};
    GaussianRNGRotation = std::normal_distribution<>{NoiseMeanRot, NoiseVarianceRot};

    InitOdom();
}

void URobotVehicleMovementComponent::UpdateMovement(float InDeltaTime)
{
    const FQuat OldRotation = UpdatedComponent->GetComponentQuat();

    FVector position = UpdatedComponent->ComponentVelocity * InDeltaTime;
    FQuat DeltaRotation(FVector::ZAxisVector, InversionFactor * AngularVelocity.Z * InDeltaTime);

    DesiredRotation = OldRotation * DeltaRotation;
    DesiredMovement = (OldRotation * position);

    FHitResult Hit;
    SafeMoveUpdatedComponent(DesiredMovement, DesiredRotation, true, Hit);

    // If we bumped into something, try to slide along it
    if (Hit.IsValidBlockingHit())
    {
        SlideAlongSurface(DesiredMovement, 1.0f - Hit.Time, Hit.Normal, Hit);
    }
}

void URobotVehicleMovementComponent::SetFrameIds(const FString& InFrameId, const FString& InChildFrameId)
{
    OdomData.header_frame_id = FrameId = InFrameId;
    OdomData.child_frame_id = ChildFrameId = InChildFrameId;
}

//todo separate ROS
void URobotVehicleMovementComponent::InitOdom()
{
    AActor* owner = GetOwner();
    OdomData.header_frame_id = FrameId;
    OdomData.child_frame_id = ChildFrameId;

    if (OdomSource == EOdomSource::ENCODER) { //odom source = encoder. Odom frame start from robot initial pose
        InitialTransform.SetTranslation(owner->GetActorLocation());
        InitialTransform.SetRotation(owner->GetActorQuat()); 
    }
    else { //odom source = world. Odom frame start from world origin
        InitialTransform.SetTranslation(FVector::ZeroVector);
        InitialTransform.SetRotation(FQuat::Identity);          
    }

    OdomData.pose_pose_position_x = InitialTransform.GetTranslation().X;
    OdomData.pose_pose_position_y = InitialTransform.GetTranslation().Y;
    OdomData.pose_pose_position_z = InitialTransform.GetTranslation().Z;
    OdomData.pose_pose_orientation = InitialTransform.GetRotation();

    PreviousTransform = InitialTransform;

    // todo temporary hardcoded
    OdomData.pose_covariance.Init(0, 36);
    OdomData.pose_covariance[0] = 1e-05f;
    OdomData.pose_covariance[7] = 1e-05f;
    OdomData.pose_covariance[14] = 1e+12;
    OdomData.pose_covariance[21] = 1e+12;
    OdomData.pose_covariance[28] = 1e+12;
    OdomData.pose_covariance[35] = 1e-03f;

    OdomData.twist_covariance.Init(0, 36);
    OdomData.twist_covariance[0] = 1e-05f;
    OdomData.twist_covariance[7] = 1e-05f;
    OdomData.twist_covariance[14] = 1e+12;
    OdomData.twist_covariance[21] = 1e+12;
    OdomData.twist_covariance[28] = 1e+12;
    OdomData.twist_covariance[35] = 1e-03f;

    IsOdomInitialized = true;

}

void URobotVehicleMovementComponent::UpdateOdom(float InDeltaTime)
{
    if (!IsOdomInitialized)
    {
        InitOdom();
    }

    // time
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    OdomData.header_stamp_sec = static_cast<int32>(TimeNow);
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    OdomData.header_stamp_nanosec = static_cast<uint32>(ns - (OdomData.header_stamp_sec * 1e+09));

    // previous estimated data
    FVector PreviousEstimatedPos =
        FVector(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z);
    FQuat PreviousEstimatedRot = OdomData.pose_pose_orientation;

    AActor* owner = GetOwner();

    // position
    FVector Pos = InitialTransform.GetRotation().UnrotateVector(owner->GetActorLocation() - InitialTransform.GetTranslation());
    FVector PreviousPos = PreviousTransform.GetTranslation(); //prev pos without noise
    PreviousTransform.SetTranslation(Pos);
    Pos += PreviousEstimatedPos - PreviousPos + WithNoise * FVector(GaussianRNGPosition(Gen), GaussianRNGPosition(Gen), 0);

    FRotator NoiseRot = FRotator(0, 0, WithNoise * GaussianRNGRotation(Gen));
    FQuat Rot =  owner->GetActorQuat() * InitialTransform.GetRotation().Inverse();
    FQuat PreviousRot = PreviousTransform.GetRotation();
    PreviousTransform.SetRotation(Rot);
    Rot = NoiseRot.Quaternion() * PreviousEstimatedRot * PreviousRot.Inverse() * Rot;
    Rot.Normalize();

    OdomData.pose_pose_position_x = Pos.X;
    OdomData.pose_pose_position_y = Pos.Y;
    OdomData.pose_pose_position_z = Pos.Z;
    OdomData.pose_pose_orientation = Rot;

    OdomData.twist_twist_linear = OdomData.pose_pose_orientation.UnrotateVector(Pos - PreviousEstimatedPos) / InDeltaTime;
    OdomData.twist_twist_angular =
        FMath::DegreesToRadians((Rot * PreviousEstimatedRot.Inverse()).GetNormalized().Euler()) / InDeltaTime;
}

void URobotVehicleMovementComponent::TickComponent(float InDeltaTime,
                                                   enum ELevelTick TickType,
                                                   FActorComponentTickFunction* ThisTickFunction)
{
    if (!ShouldSkipUpdate(InDeltaTime))
    {
        Super::TickComponent(InDeltaTime, TickType, ThisTickFunction);

        // Make sure that everything is still valid, and that we are allowed to move.
        if (IsValid(UpdatedComponent))
        {
            UpdateMovement(InDeltaTime);
            UpdateOdom(InDeltaTime);
        }

        UpdateComponentVelocity();
    }
}

FTransform URobotVehicleMovementComponent::GetOdomTF() const
{
    return FTransform(OdomData.pose_pose_orientation,
                      FVector(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z));
}
