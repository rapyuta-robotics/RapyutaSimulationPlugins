// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotVehicleMovementComponent.h"

#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

void URobotVehicleMovementComponent::UpdateMovement(float DeltaTime)
{
    const FQuat OldRotation = UpdatedComponent->GetComponentQuat();

    FVector position = UpdatedComponent->ComponentVelocity * DeltaTime;
    FQuat DeltaRotation(FVector::ZAxisVector, AngularVelocity.Z * DeltaTime);

    DesiredRotation = OldRotation * DeltaRotation;
    DesiredMovement = (OldRotation * position);
}

void URobotVehicleMovementComponent::InitOdom()
{
    OdomData.header_frame_id = FrameId;
    OdomData.child_frame_id = ChildFrameId;

    InitialTransform.SetTranslation(PawnOwner->GetActorLocation());
    InitialTransform.SetRotation(FQuat(PawnOwner->GetActorRotation()));

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

void URobotVehicleMovementComponent::UpdateOdom()
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

    // position
    FVector Pos = PawnOwner->GetActorLocation() - InitialTransform.GetTranslation();
    OdomData.pose_pose_position_x = Pos.X;
    OdomData.pose_pose_position_y = Pos.Y;
    OdomData.pose_pose_position_z = Pos.Z;
    OdomData.pose_pose_orientation = FQuat(PawnOwner->GetActorRotation() - InitialTransform.GetRotation().Rotator());

    // velocity
    OdomData.twist_twist_linear = Velocity;
    OdomData.twist_twist_angular = FMath::DegreesToRadians(AngularVelocity);
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
            UpdateOdom();

            FHitResult Hit;
            SafeMoveUpdatedComponent(DesiredMovement, DesiredRotation, true, Hit);
            // UE_LOG(LogTemp, Error, TEXT("INNNN movementtickcomponent %ds"), Hit.IsValidBlockingHit());
            // If we bumped into something, try to slide along it
            if (Hit.IsValidBlockingHit())
            {
                SlideAlongSurface(DesiredMovement, 1.0f - Hit.Time, Hit.Normal, Hit);
            }
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
