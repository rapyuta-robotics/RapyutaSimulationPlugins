// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRFloatingMovementComponent.h"

void URRFloatingMovementComponent::TickComponent(float InDeltaTime,
                                                 enum ELevelTick InTickType,
                                                 FActorComponentTickFunction* InTickFunction)
{
    if (ShouldSkipUpdate(InDeltaTime))
    {
        return;
    }

    // NOTE: Here we implement custom movement, Don't call Super::, which already auto updates Owner actor's velocity & pose
    UMovementComponent::TickComponent(InDeltaTime, InTickType, InTickFunction);

    if (!PawnOwner || !UpdatedComponent)
    {
        return;
    }

    const AController* controller = PawnOwner->GetController();
    if (false == (controller && controller->IsLocalController()))
    {
        return;
    }

    // Apply input for local players but also for AI that's not following a navigation path at the moment
    if (controller->IsLocalPlayerController() || (false == controller->IsFollowingAPath()) || bUseAccelerationForPaths)
    {
        ApplyControlInputToVelocity(InDeltaTime);
    }
    // Limit speed in case of non-path-following AI controller
    else if (IsExceedingMaxSpeed(MaxSpeed) == true)
    {
        Velocity = Velocity.GetUnsafeNormal() * MaxSpeed;
    }

    LimitWorldBounds();
    bPositionCorrected = false;

    // Move [UpdatedComponent], updating [bPositionCorrected] here-in
    FVector deltaLoc = Velocity * InDeltaTime;
    if (!deltaLoc.IsNearlyZero(1e-6f))
    {
        // Save prevLocation
        const FVector prevLocation = UpdatedComponent->GetComponentLocation();
        FHitResult hit(1.f);
        SafeMoveUpdatedComponent(deltaLoc, UpdatedComponent->GetComponentQuat(), bSweepEnabled, hit);

        if (hit.IsValidBlockingHit())
        {
            HandleImpact(hit, InDeltaTime, deltaLoc);
            // Slide the remaining distance along the hit surface
            SlideAlongSurface(deltaLoc, 1.f - hit.Time, hit.Normal, hit, bSweepEnabled);
        }

        // Update [Velocity], only if not already [bPositionCorrected] possibly due to penetration fixup
        if (false == bPositionCorrected)
        {
            Velocity = ((UpdatedComponent->GetComponentLocation() - prevLocation) / InDeltaTime);
        }
    }

    // Update [UpdatedComponent]'s Velocity by [Velocity]
    UpdateComponentVelocity();
}
