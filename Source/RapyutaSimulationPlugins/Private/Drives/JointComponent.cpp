// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/JointComponent.h"
#include <algorithm>

// Sets default values for this component's properties
UJointComponent::UJointComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

// Called when the game starts
void UJointComponent::BeginPlay()
{
    Super::BeginPlay();
}

void UJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    LinearVelocity = FVector(
        std::clamp(InLinearVelocity.X, LinearVelMin.X, LinearVelMax.X),
        std::clamp(InLinearVelocity.Y, LinearVelMin.Y, LinearVelMax.Y),
        std::clamp(InLinearVelocity.Z, LinearVelMin.Z, LinearVelMax.Z)
    );
    AngularVelocity = FVector(
        std::clamp(InAngularVelocity.X, AngularVelMin.X, AngularVelMax.X),
        std::clamp(InAngularVelocity.Y, AngularVelMin.Y, AngularVelMax.Y),
        std::clamp(InAngularVelocity.Z, AngularVelMin.Z, AngularVelMax.Z)
    );
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
    float LinearInput[3] = {0, 0, 0};
    for (i = 0; i < LinearDOF; i++)
    {
        LinearInput[i] = InVelocity[i];
    }

    float AngularInput[3] = {0, 0, 0};
    for (i = 0; i < RotationalDOF; i++)
    {
        AngularInput[i] = InVelocity[LinearDOF + i];
    }

    SetVelocity(FVector(LinearInput[0], LinearInput[1], LinearInput[2]),
                FVector(AngularInput[0], AngularInput[1], AngularInput[2]));
};

void UJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    PositionTarget = FVector(
        std::clamp(InPosition.X, PositionMin.X, PositionMax.X),
        std::clamp(InPosition.Y, PositionMin.Y, PositionMax.Y),
        std::clamp(InPosition.Z, PositionMin.Z, PositionMax.Z)
    );
    OrientationTarget = FRotator(
        std::clamp(InOrientation.Pitch, OrientationMin.Pitch, OrientationMax.Pitch),
        std::clamp(InOrientation.Yaw,   OrientationMin.Yaw,   OrientationMax.Yaw),
        std::clamp(InOrientation.Roll,  OrientationMin.Roll,  OrientationMax.Roll)
    );
};

void UJointComponent::SetPoseTargetWithArray(const TArray<float>& InPose)
{
    if (InPose.Num() != LinearDOF + RotationalDOF)
    {
        UE_LOG(LogTemp,
               Warning,
               TEXT("Given joint command num is not much with joint DOF. Linear DOF %i and Rotational DOF %i"),
               LinearDOF,
               RotationalDOF);
        return;
    }

    uint8 i;
    float LinearInput[3] = {0, 0, 0};
    for (i = 0; i < LinearDOF; i++)
    {
        LinearInput[i] = InPose[i];
    }

    float RotationalInput[3] = {0, 0, 0};
    for (i = 0; i < RotationalDOF; i++)
    {
        RotationalInput[i] = InPose[LinearDOF + i];
    }

    SetPoseTarget(FVector(LinearInput[0], LinearInput[1], LinearInput[2]),
                  FRotator(RotationalInput[1], RotationalInput[2], RotationalInput[0]));    // pitch yaw roll
};
