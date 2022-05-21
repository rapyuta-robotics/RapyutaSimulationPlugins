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
    // set joints relations and save initial parent to joint transformation.
    ParentLinkToJoint = GetRelativeTransform();
    this->SetupAttachment(ParentLink);
    ChildLink->SetupAttachment(this);

    Super::BeginPlay();
}

// Called every frame
void UKinematicJointComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    SetPoseTarget(
        PositionTarget + LinearVelocity * DeltaTime,
        OrientationTarget + FRotator(AngularVelocity[1], AngularVelocity[2], AngularVelocity[0]) * DeltaTime
    );
}

void UKinematicJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPoseTarget(InPosition, InOrientation);
    UpdatePose();
}

void UKinematicJointComponent::UpdatePose()
{
    FHitResult SweepHitResult;
    K2_SetWorldTransform(FTransform(OrientationTarget, PositionTarget) *    // joint changes
                             ParentLinkToJoint *                            // initial transform parentlink to joint
                             ParentLink->GetComponentTransform(),           // world orogin to parent
                         true,                                              // bSweep
                         SweepHitResult,
                         true    // bTeleport
    );
}
