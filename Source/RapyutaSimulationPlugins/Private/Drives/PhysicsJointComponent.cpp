// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/PhysicsJointComponent.h"

// Sets default values for this component's properties
UPhysicsJointComponent::UPhysicsJointComponent()
{
    // todo add initialization
    Constraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(FString::Printf(TEXT("%sPhysicsConstraint"), *GetName()));
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
    Constraint->ComponentName2.ComponentName = *ChildLink->GetName();
    Constraint->ComponentName1.ComponentName = *ParentLink->GetName();

    // todo add param to constraints, pose
}

void UPhysicsJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    Super::SetVelocity(InLinearVelocity, InAngularVelocity);
    Constraint->SetLinearVelocityTarget(InLinearVelocity);
    Constraint->SetAngularVelocityTarget(InAngularVelocity);
}

void UPhysicsJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPoseTarget(InPosition, InOrientation);
    Constraint->SetLinearPositionTarget(InPosition);
    Constraint->SetAngularOrientationTarget(InOrientation);
}
