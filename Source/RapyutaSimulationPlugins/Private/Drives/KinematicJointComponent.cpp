// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/KinematicJointComponent.h"

// Sets default values for this component's properties
UKinematicJointComponent::UKinematicJointComponent()
{
    // todo add initialization
}

// Called when the game starts
void UKinematicJointComponent::BeginPlay()
{
    Super::BeginPlay();
    // todo add initialization
}

// Called every frame
void UKinematicJointComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    Position += LinearVelocity * DeltaTime;
    Orientation += AngularVelocity.Rotation() * DeltaTime;

    UpdatePose();
}

void UKinematicJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPose(InPosition, InOrientation);
    UpdatePose();
}

void UKinematicJointComponent::UpdatePose()
{
    FHitResult SweepHitResult;
    ChildLink->MeshComponent->K2_SetWorldTransform(
        ChildLink->GetRelativeTransfomToParent() *                 // joint to child link center
            FTransform(Orientation, Position) *                    // joint changes
            FTransform(RotationOffset, PositionOffset) *           // joint offset
            ParentLink->GetRelativeTransfomToParent() *            // parent link center to joint
            ParentLink->MeshComponent->GetComponentTransform(),    // world orogin to parent link center
        true,                                                      // bSweep
        SweepHitResult,
        true    // bTeleport
    );
}
