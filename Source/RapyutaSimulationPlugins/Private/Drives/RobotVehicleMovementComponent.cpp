// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RobotVehicleMovementComponent.h"

#include "Kismet/KismetMathLibrary.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

URobotVehicleMovementComponent::URobotVehicleMovementComponent()
{
    Gen = std::mt19937{Rng()};
    GaussianRNGPosition = std::normal_distribution<>{NoiseMeanPos, NoiseVariancePos};
    GaussianRNGRotation = std::normal_distribution<>{NoiseMeanRot, NoiseVarianceRot};
}

void URobotVehicleMovementComponent::UpdateMovement(float DeltaTime)
{
    const FQuat OldRotation = UpdatedComponent->GetComponentQuat();

    FVector position = UpdatedComponent->ComponentVelocity * DeltaTime;
    FQuat DeltaRotation(FVector::ZAxisVector, AngularVelocity.Z * DeltaTime);

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

void URobotVehicleMovementComponent::InitOdom()
{
    OdomData.header_frame_id = FrameId;
    OdomData.child_frame_id = ChildFrameId;

    InitialTransform.SetTranslation(PawnOwner->GetActorLocation());
    InitialTransform.SetRotation(FQuat(PawnOwner->GetActorRotation()));

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

void URobotVehicleMovementComponent::UpdateOdom(float DeltaTime)
{
    if (!IsOdomInitialized)
    {
        InitOdom();
    }

    // noise is cumulative and gaussian
    // need to track previous real and estimated position
    // currPos - previousRealPos + previousEstimatedPos
    // same for rot
    // 2 location: DifferentialDriveComponent, RobotVehicleMovementComponent

    // time
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    OdomData.header_stamp_sec = static_cast<int32>(TimeNow);
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    OdomData.header_stamp_nanosec = static_cast<uint32>(ns - (OdomData.header_stamp_sec * 1e+09));

    // previous estimated data
    FVector PreviousEstimatedPos = FVector(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z);
    FQuat PreviousEstimatedRot = OdomData.pose_pose_orientation;

    // position
    FVector Pos = PawnOwner->GetActorLocation() + FVector(GaussianRNGPosition(Gen), GaussianRNGPosition(Gen), GaussianRNGPosition(Gen));
    PreviousTransform.SetTranslation(Pos);
    Pos += PreviousEstimatedPos - PreviousTransform.GetTranslation() - InitialTransform.GetTranslation();

    FQuat Rot = FQuat(UKismetMathLibrary::ComposeRotators(PawnOwner->GetActorRotation(), FRotator(GaussianRNGRotation(Gen), GaussianRNGRotation(Gen), GaussianRNGRotation(Gen))));
    PreviousTransform.SetRotation(Rot);
    Rot = InitialTransform.GetRotation().Inverse() * PreviousEstimatedRot * PreviousTransform.GetRotation().Inverse() * Rot;
    Rot.Normalize();


    OdomData.pose_pose_position_x = Pos.X;
    OdomData.pose_pose_position_y = Pos.Y;
    OdomData.pose_pose_position_z = Pos.Z;
    OdomData.pose_pose_orientation = Rot;

    // velocity
    OdomData.twist_twist_linear = (Pos - PreviousEstimatedPos)/DeltaTime;
    OdomData.twist_twist_angular = FMath::DegreesToRadians((OdomData.pose_pose_orientation * PreviousEstimatedRot.Inverse()).GetNormalized().Euler())/DeltaTime;
}

void URobotVehicleMovementComponent::TickComponent(float DeltaTime,
                                                   enum ELevelTick TickType,
                                                   FActorComponentTickFunction* ThisTickFunction)
{
    if (!ShouldSkipUpdate(DeltaTime))
    {
        Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

        // Make sure that everything is still valid, and that we are allowed to move.
        if (IsValid(UpdatedComponent))
        {
            UpdateMovement(DeltaTime);
            UpdateOdom(DeltaTime);
        }

        UpdateComponentVelocity();
    }
}

FTransform URobotVehicleMovementComponent::GetOdomTF()
{
    FTransform TF;
    FVector Pos(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z);
    TF.SetTranslation(Pos);
    TF.SetRotation(OdomData.pose_pose_orientation);
    return TF;
}

void URobotVehicleMovementComponent::InitMovementComponent()
{
    InitOdom();
}