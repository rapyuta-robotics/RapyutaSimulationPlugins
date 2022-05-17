// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/PhysicsJointComponent.h"

// Sets default values for this component's properties
UPhysicsJointComponent::UPhysicsJointComponent()
{
    // todo add initialization
    Constraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_LidarSensor"));
}

// Called when the game starts
void UPhysicsJointComponent::BeginPlay()
{
    SetJoint();
    Super::BeginPlay();
    // todo add initialization
}

void UPhysicsJointComponent::SetJoint()
{
    // add param to constraints, pose
}

void UPhysicsJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    Super::SetVelocity(InLinearVelocity, InAngularVelocity);
    Constraint->SetLinearVelocityTarget(InLinearVelocity);
    Constraint->SetAngularVelocityTarget(InAngularVelocity);
}

void UPhysicsJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPose(InPosition, InOrientation);
    Constraint->SetLinearPositionTarget(InPosition);
    Constraint->SetAngularOrientationTarget(InOrientation);
}
