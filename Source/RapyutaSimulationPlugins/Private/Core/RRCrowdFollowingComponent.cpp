// Copyright 2020-2022 Rapyuta Robotics Co., Ltd

#include "Core/RRCrowdFollowingComponent.h"

//RapyutaSimulationPlugins
#include "Core/RRMathUtils.h"
#include "Drives/RRFloatingMovementComponent.h"

URRCrowdFollowingComponent::URRCrowdFollowingComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    AvoidanceQuality = ECrowdAvoidanceQuality::Good;
}

bool URRCrowdFollowingComponent::Is2DMovement() const
{
    return FloatMovementComp && FloatMovementComp->Is2DMovement();
}

void URRCrowdFollowingComponent::SetMovementComponent(UNavMovementComponent* InMoveComp)
{
    Super::SetMovementComponent(InMoveComp);
    FloatMovementComp = Cast<URRFloatingMovementComponent>(InMoveComp);
    if (FloatMovementComp)
    {
        bEnableSlowdownAtGoal = FloatMovementComp->UseDecelerationForPathFollowing();
    }
}

void URRCrowdFollowingComponent::FollowPathSegment(float InDeltaTime)
{
    if (IsCrowdSimulationEnabled() || (MovementComp && MovementComp->UseAccelerationForPathFollowing()))
    {
        Super::FollowPathSegment(InDeltaTime);
    }
    else
    {
        if (!Path.IsValid() || MovementComp == nullptr)
        {
            return;
        }

        // set to false by default, we will set set this back to true if appropriate
        bIsDecelerating = false;

        // New instantaneous vel
        const float maxSpeed = GetCrowdAgentMaxSpeed();
        FVector newVelocity = (GetCurrentTargetLocation() - MovementComp->GetActorFeetLocation()) / InDeltaTime;
        if (FloatMovementComp && (false == FloatMovementComp->UseDecelerationForPathFollowing()))
        {
            // NON-DECELERATION movement: Always keep the vel's magnitude as [maxSpeed]
            URRMathUtils::SetVectorClampedToMaxMagnitude(newVelocity, maxSpeed, Is2DMovement());
        }
        else
        {
            URRMathUtils::ClampVectorToMaxMagnitude(newVelocity, maxSpeed, Is2DMovement());
        }

        const int32 lastSegmentStartIndex = Path->GetPathPoints().Num() - 2;
        const bool bNotFollowingLastSegment = (MoveSegmentStartIndex < lastSegmentStartIndex);

        PostProcessMove.ExecuteIfBound(this, newVelocity);
        MovementComp->RequestDirectMove(newVelocity, bNotFollowingLastSegment);
    }
}

void URRCrowdFollowingComponent::ApplyCrowdAgentVelocity(const FVector& InNewVelocity,
                                                         const FVector& InDestPathCorner,
                                                         bool bInTraversingLink,
                                                         bool bInNearEndOfPath)
{
    FVector newVel = InNewVelocity;
    URRMathUtils::ClampVectorToMaxMagnitude(newVel, GetCrowdAgentMaxSpeed(), Is2DMovement());
    Super::ApplyCrowdAgentVelocity(InNewVelocity, InDestPathCorner, bInTraversingLink, bInNearEndOfPath);
}

void URRCrowdFollowingComponent::OnPathFinished(const FPathFollowingResult& InResult)
{
    // IF NOT using deceleration: Don't stop movement on finish
    if (FloatMovementComp && (false == FloatMovementComp->UseDecelerationForPathFollowing()))
    {
        bStopMovementOnFinish = false;
    }
    Super::OnPathFinished(InResult);
}
