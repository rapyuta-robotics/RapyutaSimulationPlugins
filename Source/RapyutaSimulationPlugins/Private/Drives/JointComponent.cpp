// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/JointComponent.h"

#include <algorithm>

// Sets default values for this component's properties
UJointComponent::UJointComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    LinearVelocity = InLinearVelocity.BoundToBox(LinearVelMin, LinearVelMax);
    AngularVelocity = InAngularVelocity.BoundToBox(AngularVelMin, AngularVelMax);
};

void UJointComponent::SetVelocityWithArray(const TArray<float>& InVelocity)
{
    if (InVelocity.Num() != LinearDOF + RotationalDOF)
    {
        UE_LOG(LogTemp,
               Warning,
               TEXT("Given joint command num is not much with joint DOF. Linear DOF %i and Rotational DOF %i"),
               LinearDOF,
               RotationalDOF);
        return;
    }

    uint8 i;
    FVector LinearInput = FVector(0, 0, 0);
    for (i = 0; i < LinearDOF; i++)
    {
        LinearInput[i] = InVelocity[i];
    }

    FVector AngularInput = FVector(0, 0, 0);
    for (i = 0; i < RotationalDOF; i++)
    {
        AngularInput[i] = InVelocity[LinearDOF + i];
    }

    SetVelocity(LinearInput, AngularInput);
};

void UJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    PositionTarget = InPosition;
    OrientationTarget = InOrientation;
};

void UJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
    Position = InPosition.BoundToBox(PositionMin, PositionMax);
    Orientation = FRotator(FMath::Clamp(InOrientation.Pitch, OrientationMin.Pitch, OrientationMax.Pitch),
                           FMath::Clamp(InOrientation.Yaw, OrientationMin.Yaw, OrientationMax.Yaw),
                           FMath::Clamp(InOrientation.Roll, OrientationMin.Roll, OrientationMax.Roll));
};

void UJointComponent::PoseFromArray(const TArray<float>& InPose, FVector& OutPosition, FRotator& OutOrientation)
{
    if (InPose.Num() != LinearDOF + RotationalDOF)
    {
        UE_LOG(LogRapyutaCore,
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

void UJointComponent::SetPoseTargetWithArray(const TArray<float>& InPose)
{
    FVector OutPosition;
    FRotator OutOrientation;
    PoseFromArray(InPose, OutPosition, OutOrientation);
    SetPoseTarget(OutPosition, OutOrientation);
}

void UJointComponent::SetPoseWithArray(const TArray<float>& InPose)
{
    FVector OutPosition;
    FRotator OutOrientation;
    PoseFromArray(InPose, OutPosition, OutOrientation);
    SetPose(OutPosition, OutOrientation);
}
