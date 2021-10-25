// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RobotVehicleMovementComponent.h"

#include "Kismet/KismetMathLibrary.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

URobotVehicleMovementComponent::URobotVehicleMovementComponent()
{
    Gen = std::mt19937{Rng()};
}

void URobotVehicleMovementComponent::BeginPlay()
{
    Super::BeginPlay();
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

    PreviousTransform = FTransform(FQuat(0,0,0,1), FVector(0,0,0), FVector(1,1,1));

    OdomData.pose_pose_position_x = 0;
    OdomData.pose_pose_position_y = 0;
    OdomData.pose_pose_position_z = 0;
    OdomData.pose_pose_orientation = FQuat(0,0,0,1);

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

    // time
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    OdomData.header_stamp_sec = static_cast<int32>(TimeNow);
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    OdomData.header_stamp_nanosec = static_cast<uint32>(ns - (OdomData.header_stamp_sec * 1e+09));

    // previous estimated data
    FVector PreviousEstimatedPos = FVector(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z);
    FQuat PreviousEstimatedRot = OdomData.pose_pose_orientation;

    // position
    FVector Pos = PawnOwner->GetActorLocation() - InitialTransform.GetTranslation();
    FVector PreviousPos = PreviousTransform.GetTranslation();
    PreviousTransform.SetTranslation(Pos);
    Pos += PreviousEstimatedPos - PreviousPos + WithNoise * FVector(GaussianRNGPosition(Gen), GaussianRNGPosition(Gen), 0);

    FRotator NoiseRot = FRotator(0, 0, WithNoise * GaussianRNGRotation(Gen));
    FQuat Rot = InitialTransform.GetRotation().Inverse() * PawnOwner->GetActorRotation().Quaternion();
    FQuat PreviousRot = PreviousTransform.GetRotation();
    PreviousTransform.SetRotation(Rot);
    Rot =  NoiseRot.Quaternion() * PreviousEstimatedRot * PreviousRot.Inverse() * Rot;
    Rot.Normalize();

    OdomData.pose_pose_position_x = Pos.X;
    OdomData.pose_pose_position_y = Pos.Y;
    OdomData.pose_pose_position_z = Pos.Z;
    OdomData.pose_pose_orientation = Rot;

    OdomData.twist_twist_linear = OdomData.pose_pose_orientation.UnrotateVector( Pos-PreviousEstimatedPos ) / DeltaTime;
    OdomData.twist_twist_angular = FMath::DegreesToRadians((PreviousEstimatedRot * Rot.Inverse()).GetNormalized().Euler())/DeltaTime;

    // UE_LOG(LogTemp, Warning, TEXT("Odometry:"));
    // UE_LOG(LogTemp, Warning, TEXT("\tCurrent Positon:\t\t\t%s"), *PawnOwner->GetActorLocation().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tInitial Positon:\t\t\t%s"), *InitialTransform.GetTranslation().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tPrevious Positon:\t\t\t%s"), *PreviousTransform.GetTranslation().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tPrevious Estimated Positon:\t%s"), *PreviousEstimatedPos.ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom Positon:\t\t\t\t%s"), *Pos.ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tCurrent Orientation:\t\t%s"), *PawnOwner->GetActorRotation().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tInitial Orientation:\t\t%s"), *InitialTransform.GetRotation().Rotator().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tPrevious Orientation:\t\t%s"), *PreviousTransform.GetRotation().Rotator().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tPrevious Estimated Orientation:\t%s"), *PreviousEstimatedRot.Rotator().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom Orientation:\t\t\t%s"), *Rot.Rotator().ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom Orientation:\t\t\t%s"), *Rot.ToString());
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistLin:\t\t\t\t%s - %f"), *OdomData.twist_twist_linear.ToString(), OdomData.twist_twist_linear.Size());
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistAng:\t\t\t\t%s"), *OdomData.twist_twist_angular.ToString());
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

void URobotVehicleMovementComponent::Initialize()
{   
    InitOdom();
}