// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/JointComponent.h"

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
    //! todo add limitation
    LinearVelocity = InLinearVelocity;
    AngularVelocity = InAngularVelocity;
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
    //! todo add limitation
    PositionTarget = InPosition;
    OrientationTarget = InOrientation;
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
