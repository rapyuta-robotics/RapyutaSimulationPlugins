// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRJointComponent.h"

// Sets default values for this component's properties
URRJointComponent::URRJointComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void URRJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    LinearVelocity = InLinearVelocity.BoundToBox(LinearVelMin, LinearVelMax);
    AngularVelocity = InAngularVelocity.BoundToBox(AngularVelMin, AngularVelMax);
};

void URRJointComponent::SetVelocityWithArray(const TArray<float>& InVelocity)
{
    if (InVelocity.Num() != LinearDOF + RotationalDOF)
    {
        UE_LOG_WITH_INFO(LogTemp,
                         Warning,
                         TEXT("Given joint command num is not much with joint DOF. Linear DOF %i and Rotational DOF %i"),
                         LinearDOF,
                         RotationalDOF);
        return;
    }

    uint8 i;
    FVector LinearInput = FVector::ZeroVector;
    for (i = 0; i < LinearDOF; i++)
    {
        LinearInput[i] = InVelocity[i];
    }

    FVector AngularInput = FVector::ZeroVector;
    for (i = 0; i < RotationalDOF; i++)
    {
        AngularInput[i] = InVelocity[LinearDOF + i];
    }

    SetVelocity(LinearInput, AngularInput);
};

void URRJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    PositionTarget = InPosition;
    OrientationTarget = InOrientation;
};

bool URRJointComponent::HasReachedPoseTarget(const float InTolerance)
{
    return PositionTarget.Equals(Position, InTolerance) && OrientationTarget.Equals(Orientation, InTolerance);
};

void URRJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
    Position = InPosition.BoundToBox(PositionMin, PositionMax);
    Orientation =
        FRotator(IsLimitPitch ? FMath::Clamp(InOrientation.Pitch, OrientationMin.Pitch, OrientationMax.Pitch) : InOrientation.Pitch,
                 IsLimitYaw ? FMath::Clamp(InOrientation.Yaw, OrientationMin.Yaw, OrientationMax.Yaw) : InOrientation.Yaw,
                 IsLimitRoll ? FMath::Clamp(InOrientation.Roll, OrientationMin.Roll, OrientationMax.Roll) : InOrientation.Roll);
};

void URRJointComponent::PoseFromArray(const TArray<float>& InPose, FVector& OutPosition, FRotator& OutOrientation)
{
    if (InPose.Num() != LinearDOF + RotationalDOF)
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore,
            Warning,
            TEXT("Given joint pose values num (%u) does not match joint total DOF (Linear DOF %i & Rotational DOF %i)"),
            InPose.Num(),
            LinearDOF,
            RotationalDOF);
        return;
    }

    uint8 i;
    FVector LinearInput = FVector(0, 0, 0);
    for (i = 0; i < LinearDOF; i++)
    {
        LinearInput[i] = InPose[i];
    }

    FVector RotationalInput = FVector(0, 0, 0);
    for (i = 0; i < RotationalDOF; i++)
    {
        RotationalInput[i] = InPose[LinearDOF + i];
    }

    OutPosition = LinearInput;
    OutOrientation = FRotator::MakeFromEuler(RotationalInput);
};

void URRJointComponent::SetPoseTargetWithArray(const TArray<float>& InPose)
{
    FVector OutPosition;
    FRotator OutOrientation;
    PoseFromArray(InPose, OutPosition, OutOrientation);
    SetPoseTarget(OutPosition, OutOrientation);
}

void URRJointComponent::SetPoseWithArray(const TArray<float>& InPose)
{
    FVector OutPosition;
    FRotator OutOrientation;
    PoseFromArray(InPose, OutPosition, OutOrientation);
    SetPose(OutPosition, OutOrientation);
}
