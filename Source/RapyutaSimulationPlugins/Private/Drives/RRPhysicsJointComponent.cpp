// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRPhysicsJointComponent.h"

// Sets default values for this component's properties
URRPhysicsJointComponent::URRPhysicsJointComponent()
{
    // todo add initialization
    Constraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("%sPhysicsConstraint"), *GetName());
}

// Called when the game starts
void URRPhysicsJointComponent::BeginPlay()
{
    SetJoint();
    Super::BeginPlay();
    // todo add initialization
}

void URRPhysicsJointComponent::SetJoint()
{
    Constraint->ComponentName2.ComponentName = *ChildLink->GetName();
    Constraint->ComponentName1.ComponentName = *ParentLink->GetName();

    // todo add param to constraints, pose
}

void URRPhysicsJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    Super::SetVelocity(InLinearVelocity, InAngularVelocity);
    Constraint->SetLinearVelocityTarget(InLinearVelocity);
    Constraint->SetAngularVelocityTarget(InAngularVelocity);
}

void URRPhysicsJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPoseTarget(InPosition, InOrientation);
    Constraint->SetLinearPositionTarget(InPosition);
    Constraint->SetAngularOrientationTarget(InOrientation);
}
