// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/RRKinematicJointComponent.h"

// Sets default values for this component's properties
URRKinematicJointComponent::URRKinematicJointComponent()
{
    // todo add initialization
}

void URRKinematicJointComponent::Initialize()
{
    if (IsValid())
    {
        // set joints relations and save initial parent to joint transformation.
        JointToChildLink = ChildLink->GetRelativeTransform();
        ParentLinkToJoint = GetRelativeTransform();
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogTemp, Error, TEXT("JointComponent must have ChildLink and ParentLink."));
    }
}

void URRKinematicJointComponent::UpdateControl(const float DeltaTime)
{
}

void URRKinematicJointComponent::UpdateState(const float DeltaTime)
{
    if (!LinearVelocity.IsZero() || !AngularVelocity.IsZero())
    {
        FVector dPos = LinearVelocity * DeltaTime;
        FVector dRot = AngularVelocity * DeltaTime;

        // Check reach goal in this step.
        if (ControlType == ERRJointControlType::POSITION)
        {
            uint8 i;
            for (i = 0; i < 3; i++)
            {
                if (FMath::Abs(Position[i] - PositionTarget[i]) < FMath::Abs(dPos[i]))
                {
                    Position[i] = PositionTarget[i];
                    LinearVelocity[i] = 0;
                }
                else
                {
                    Position[i] += dPos[i];
                }
            }

            FVector orientationVec = Orientation.Euler();
            FVector orientationTargetVec = OrientationTarget.Euler();
            for (i = 0; i < 3; i++)
            {
                if (FMath::Abs(orientationVec[i] - orientationTargetVec[i]) < FMath::Abs(dRot[i]))
                {
                    orientationVec[i] = orientationTargetVec[i];
                    AngularVelocity[i] = 0;
                }
                else
                {
                    orientationVec[i] += dRot[i];
                }
            }
            Orientation = FRotator::MakeFromEuler(orientationVec);
        }
        else
        {
            Position += dPos;
            Orientation += FRotator::MakeFromEuler(dRot);
        }

        SetPose(Position, Orientation);
    }
}

void URRKinematicJointComponent::SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    Super::SetVelocityTarget(InLinearVelocity, InAngularVelocity);
    SetVelocity(InLinearVelocity, InAngularVelocity);
};

void URRKinematicJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPose(InPosition, InOrientation);
    UpdatePose();
}

void URRKinematicJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPoseTarget(InPosition, InOrientation);

    FVector poseDiff = PositionTarget - Position;
    FVector orientDiff = OrientationTarget.Euler() - Orientation.Euler();
    uint8 i;
    for (i = 0; i < 3; i++)
    {
        LinearVelocity[i] = FMath::IsNearlyZero(poseDiff[i]) ? 0 : poseDiff[i] < 0 ? -LinearVelMax[i] : LinearVelMax[i];
    }
    for (i = 0; i < 3; i++)
    {
        AngularVelocity[i] = FMath::IsNearlyZero(orientDiff[i]) ? 0 : orientDiff[i] < 0 ? -AngularVelMax[i] : AngularVelMax[i];
    }
}

void URRKinematicJointComponent::UpdatePose()
{
    if (!IsValid())
    {
        return;
    }
    FHitResult SweepHitResult;
    K2_SetWorldTransform(FTransform(Orientation, Position) *         // joint changes
                             ParentLinkToJoint *                     // joint to child l
                             ParentLink->GetComponentTransform(),    // world orogin to parent
                         true,                                       // bSweep
                         SweepHitResult,
                         false    // bTeleport
    );
}

void URRKinematicJointComponent::Teleport(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::Teleport(InPosition, InOrientation);
    SetPose(InPosition, InOrientation);
};


void URRKinematicJointComponent::MoveToInitPose()
{
    Teleport(InitialPosition, InitialOrientation);
    SetPoseTarget(InitialPosition, InitialOrientation);
}
