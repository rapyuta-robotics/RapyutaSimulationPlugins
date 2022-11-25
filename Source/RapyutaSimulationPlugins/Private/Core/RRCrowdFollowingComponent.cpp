// Copyright 2020-2022 Rapyuta Robotics Co., Ltd

#include "Core/RRCrowdFollowingComponent.h"

#include "AITypes.h"
#include "BrainComponent.h"
#include "NavLinkCustomInterface.h"
#include "Navigation/MetaNavMeshPath.h"

//RapyutaSimulationPlugins
#include "Drives/RRFloatingMovementComponent.h"

URRCrowdFollowingComponent::URRCrowdFollowingComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SimulationState = ECrowdSimulationState::ObstacleOnly;
}

void URRCrowdFollowingComponent::SetMovementComponent(UNavMovementComponent* InMoveComp)
{
    Super::SetMovementComponent(InMoveComp);
    FloatMovementComp = Cast<URRFloatingMovementComponent>(InMoveComp);
}

void URRCrowdFollowingComponent::FollowPathSegment(float InDeltaTime)
{
    UE_LOG(
        LogTemp, Error, TEXT("%s Crowd:FollowPathSegment Before : %s"), *GetOwner()->GetName(), *MovementComp->Velocity.ToString());
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

        // Keeps the previous Velocity in case of constant movement
        FVector newExpectedVelocity = (GetCurrentTargetLocation() - MovementComp->GetActorFeetLocation()) / InDeltaTime;
        FVector newVelocity;
        if (FloatMovementComp && FloatMovementComp->UseConstantVelocity())
        {
            newVelocity = newExpectedVelocity.GetUnsafeNormal() * MovementComp->Velocity.Size();
        }
        else
        {
            newVelocity = newExpectedVelocity;
        }
        //if ((MaxCrowdSpeed > 0.f) && (newVelocity.SizeSquared2D() > 1.01f * FMath::Square(MaxCrowdSpeed)))
        //{
        //    newVelocity = newVelocity.GetUnsafeNormal() * MaxCrowdSpeed;
        //}

        const int32 lastSegmentStartIndex = Path->GetPathPoints().Num() - 2;
        const bool bNotFollowingLastSegment = (MoveSegmentStartIndex < lastSegmentStartIndex);

        PostProcessMove.ExecuteIfBound(this, newVelocity);
        MovementComp->RequestDirectMove(newVelocity, bNotFollowingLastSegment);
    }
    UE_LOG(
        LogTemp, Error, TEXT("%s Crowd:FollowPathSegment After : %s"), *GetOwner()->GetName(), *MovementComp->Velocity.ToString());
}

void URRCrowdFollowingComponent::ApplyCrowdAgentVelocity(const FVector& InNewVelocity,
                                                         const FVector& InDestPathCorner,
                                                         bool bInTraversingLink,
                                                         bool bInNearEndOfPath)
{
    const FVector newVel = (MaxCrowdSpeed > 0.f) && (InNewVelocity.SizeSquared2D() > 1.01f * FMath::Square(MaxCrowdSpeed))
                               ? (InNewVelocity.GetUnsafeNormal() * MaxCrowdSpeed)
                               : InNewVelocity;
    Super::ApplyCrowdAgentVelocity(newVel, InDestPathCorner, bInTraversingLink, bInNearEndOfPath);
    UE_LOG(
        LogTemp, Error, TEXT("%s Crowd:ApplyCrowdAgentVelocity : %s"), *GetOwner()->GetName(), *MovementComp->Velocity.ToString());
}

void URRCrowdFollowingComponent::OnPathFinished(const FPathFollowingResult& InResult)
{
    if (FloatMovementComp && (false == FloatMovementComp->UseDecelerationForPathFollowing()))
    {
        bStopMovementOnFinish = false;
    }
    Super::OnPathFinished(InResult);
}
