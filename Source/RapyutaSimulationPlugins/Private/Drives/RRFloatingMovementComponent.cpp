// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRFloatingMovementComponent.h"

URRFloatingMovementComponent::URRFloatingMovementComponent(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer), bSweepEnabled(true), b2DMovement(false), bUseDecelerationForPaths(true)
{
}

bool URRFloatingMovementComponent::IsExceedingMaxSpeed(float InMaxSpeed) const
{
    if (b2DMovement)
    {
        const float maxSpeedSquared = FMath::Square(FMath::Max(0.f, InMaxSpeed));

        // Allow 1% error tolerance, to account for numeric imprecision.
        static constexpr float OVER_VEL_PERCENT = 1.01f;
        bool bResult = (Velocity.SizeSquared2D() > OVER_VEL_PERCENT * FMath::Square(FMath::Max(0.f, InMaxSpeed)));
        if (bResult && (Velocity != FVector::ZeroVector))
            UE_LOG(LogTemp,
                   Error,
                   TEXT("%s Is ExceedingMaxSpeed %s : %f # %f"),
                   *GetOwner()->GetName(),
                   *Velocity.ToString(),
                   Velocity.SizeSquared2D(),
                   maxSpeedSquared);
        return bResult;
    }
    else
    {
        return Super::IsExceedingMaxSpeed(InMaxSpeed);
    }
}

void URRFloatingMovementComponent::TickComponent(float InDeltaTime,
                                                 enum ELevelTick InTickType,
                                                 FActorComponentTickFunction* InTickFunction)
{
    if (ShouldSkipUpdate(InDeltaTime))
    {
        return;
    }

    // NOTE: Here we implement custom movement, Don't call UFloatingPawnMovement::, which already auto updates Owner actor's velocity & pose
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

    if (Velocity != FVector::ZeroVector)
        UE_LOG(LogTemp, Error, TEXT("%s Cur Vel : %s"), *GetOwner()->GetName(), *Velocity.ToString());

    // Apply input for local players but also for AI that's not following a navigation path at the moment
    if (controller->IsLocalPlayerController() || (false == controller->IsFollowingAPath()) || bUseAccelerationForPaths)
    {
        ApplyControlInputToVelocity(InDeltaTime);
        if (Velocity != FVector::ZeroVector)
            UE_LOG(LogTemp, Error, TEXT("%s Vel 1 : %s"), *GetOwner()->GetName(), *Velocity.ToString());
    }
    // Limit speed in case of non-path-following AI controller
    else if (IsExceedingMaxSpeed(MaxSpeed))
    {
        if (b2DMovement)
        {
            Velocity = FVector(FVector2D(Velocity.X, Velocity.Y).GetSafeNormal() * MaxSpeed, 0.f);
        }
        else
        {
            Velocity = Velocity.GetUnsafeNormal() * MaxSpeed;
        }
        if (Velocity != FVector::ZeroVector)
        {
            UE_LOG(LogTemp, Error, TEXT("%s Vel 2 : %s"), *GetOwner()->GetName(), *Velocity.ToString());
        }
    }

    if (false == b2DMovement)
    {
        LimitWorldBounds();
    }
    bPositionCorrected = false;

    // Move [UpdatedComponent], updating [bPositionCorrected] here-in
    FVector deltaLoc = Velocity * InDeltaTime;
    if (!deltaLoc.IsNearlyZero(1e-6f))
    {
        // Save prevLocation
        const FVector prevLocation = UpdatedComponent->GetComponentLocation();
        // Scope for setting move flags & restoring it after movement
        {
            // NOTE: [UpdatedComponent] should have been attached to the target base, of which overlapping is ignored
            TGuardValue<EMoveComponentFlags> ScopedFlagRestore(MoveComponentFlags, MoveComponentFlags | MOVECOMP_IgnoreBases);

            FHitResult hit(1.f);
            SafeMoveUpdatedComponent(deltaLoc, UpdatedComponent->GetComponentQuat(), bSweepEnabled, hit);

#if RAPYUTA_SIM_DEBUG
            if (hit.bBlockingHit)
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("ResolvePenetration: %s.%s at location %s inside %s.%s at location %s by %.3f (netmode: %d)\n"
                            "TraceStart %s TraceEnd %s Bone %s"),
                       *UpdatedComponent->GetOwner()->GetName(),
                       *UpdatedComponent->GetName(),
                       *UpdatedComponent->GetComponentLocation().ToString(),
                       *GetNameSafe(hit.GetActor()),
                       *GetNameSafe(hit.GetComponent()),
                       hit.Component.IsValid() ? *hit.GetComponent()->GetComponentLocation().ToString() : TEXT("<unknown>"),
                       hit.PenetrationDepth,
                       static_cast<uint32>(GetNetMode()),
                       *hit.TraceStart.ToString(),
                       *hit.TraceEnd.ToString(),
                       *hit.BoneName.ToString());
            }
#endif

            if (hit.IsValidBlockingHit())
            {
                HandleImpact(hit, InDeltaTime, deltaLoc);
                // Slide the remaining distance along the hit surface
                SlideAlongSurface(deltaLoc, 1.f - hit.Time, hit.Normal, hit, bSweepEnabled);
            }
        }

        // Update current-moment [Velocity], only if using acceleration/deceleration + not already [bPositionCorrected] possibly due to penetration fixup
        if (bUseAccelerationForPaths && bUseDecelerationForPaths && (false == bPositionCorrected))
        {
            Velocity = ((UpdatedComponent->GetComponentLocation() - prevLocation) / InDeltaTime);
            if (Velocity != FVector::ZeroVector)
                UE_LOG(LogTemp, Error, TEXT("%s Vel 3 : %s"), *GetOwner()->GetName(), *Velocity.ToString());
        }
    }

    if (Velocity != FVector::ZeroVector)
        UE_LOG(LogTemp, Error, TEXT("%s Vel final : %s"), *GetOwner()->GetName(), *Velocity.ToString());

    // Update [UpdatedComponent]'s Velocity by [Velocity]
    UpdateComponentVelocity();
}

#if RAPYUTA_SIM_DEBUG
bool URRFloatingMovementComponent::ResolvePenetrationImpl(const FVector& InProposedAdjustment,
                                                          const FHitResult& InHit,
                                                          const FQuat& InNewRotationQuat)
{
    // NOTE: THIS IS REIMPLEMENTATION, DON'T CALL SUPER::
    // SceneComponent can't be in penetration, so this function really only applies to PrimitiveComponent.
    const FVector constrainedAdjustment = ConstrainDirectionToPlane(InProposedAdjustment);
    if (!constrainedAdjustment.IsZero() && UpdatedPrimitive)
    {
        QUICK_SCOPE_CYCLE_COUNTER(STAT_MovementComponent_ResolvePenetration);
        // See if we can fit at the adjusted location without overlapping anything.
        AActor* ownerActor = UpdatedComponent->GetOwner();
        if (!ownerActor)
        {
            return false;
        }

        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("ResolvePenetration: %s.%s at location %s inside %s.%s at location %s by %.3f (netmode: %d)\n"
                    "TraceStart %s TraceEnd %s Bone %s"),
               *ownerActor->GetName(),
               *UpdatedComponent->GetName(),
               *UpdatedComponent->GetComponentLocation().ToString(),
               *GetNameSafe(InHit.GetActor()),
               *GetNameSafe(InHit.GetComponent()),
               InHit.Component.IsValid() ? *InHit.GetComponent()->GetComponentLocation().ToString() : TEXT("<unknown>"),
               InHit.PenetrationDepth,
               static_cast<uint32>(GetNetMode()),
               *InHit.TraceStart.ToString(),
               *InHit.TraceEnd.ToString(),
               *InHit.BoneName.ToString());

        // We really want to make sure that precision differences or differences between the overlap test and sweep tests don't put us into another overlap,
        // so make the overlap test a bit more restrictive.
        const float overlapInflation = 0.1f;    // MovementComponentCVars::PenetrationOverlapCheckInflation;
        bool bEncroached = OverlapTest(InHit.TraceStart + constrainedAdjustment,
                                       InNewRotationQuat,
                                       UpdatedPrimitive->GetCollisionObjectType(),
                                       UpdatedPrimitive->GetCollisionShape(overlapInflation),
                                       ownerActor);
        if (!bEncroached)
        {
            // Move without sweeping.
            MoveUpdatedComponent(constrainedAdjustment, InNewRotationQuat, false, nullptr, ETeleportType::TeleportPhysics);
            UE_LOG(LogRapyutaCore, Error, TEXT("ResolvePenetration: teleport by %s"), *constrainedAdjustment.ToString());
            return true;
        }
        else
        {
            // Disable MOVECOMP_NeverIgnoreBlockingOverlaps if it is enabled, otherwise we wouldn't be able to sweep out of the object to fix the penetration.
            TGuardValue<EMoveComponentFlags> ScopedFlagRestore(
                MoveComponentFlags, EMoveComponentFlags(MoveComponentFlags & (~MOVECOMP_NeverIgnoreBlockingOverlaps)));

            // Try sweeping as far as possible...
            FHitResult outSweepHit(1.f);
            bool bMoved =
                MoveUpdatedComponent(constrainedAdjustment, InNewRotationQuat, true, &outSweepHit, ETeleportType::TeleportPhysics);

            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("ResolvePenetration: sweep by %s (success = %d)"),
                   *constrainedAdjustment.ToString(),
                   bMoved);

            // Try sweep again - 1st
            if (!bMoved && outSweepHit.bStartPenetrating)
            {
                // Combine two MTD results to get a new direction that gets out of multiple surfaces.
                const FVector secondMTD = GetPenetrationAdjustment(outSweepHit);
                const FVector combinedMTD = constrainedAdjustment + secondMTD;
                if (secondMTD != constrainedAdjustment && !combinedMTD.IsZero())
                {
                    bMoved = MoveUpdatedComponent(combinedMTD, InNewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
                    UE_LOG(LogRapyutaCore,
                           Error,
                           TEXT("ResolvePenetration: sweep by %s (MTD combo success = %d)"),
                           *combinedMTD.ToString(),
                           bMoved);
                }
            }

            // Try sweep again - 2nd
            if (!bMoved)
            {
                // Try moving the proposed adjustment plus the attempted move direction. This can sometimes get out of penetrations with multiple objects
                const FVector moveDelta = ConstrainDirectionToPlane(InHit.TraceEnd - InHit.TraceStart);
                if (!moveDelta.IsZero())
                {
                    bMoved = MoveUpdatedComponent(
                        constrainedAdjustment + moveDelta, InNewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
                    UE_LOG(LogRapyutaCore,
                           Error,
                           TEXT("ResolvePenetration: sweep by %s (adjusted attempt success = %d)"),
                           *(constrainedAdjustment + moveDelta).ToString(),
                           bMoved);

                    // Finally, try the original move without MTD adjustments, but allowing depenetration along the MTD normal.
                    // This was blocked because MOVECOMP_NeverIgnoreBlockingOverlaps was true for the original move to try a better depenetration normal, but we might be running in to other geometry in the attempt.
                    // This won't necessarily get us all the way out of penetration, but can in some cases and does make progress in exiting the penetration.
                    if (!bMoved && FVector::DotProduct(moveDelta, constrainedAdjustment) > 0.f)
                    {
                        bMoved = MoveUpdatedComponent(moveDelta, InNewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
                        UE_LOG(LogRapyutaCore,
                               Error,
                               TEXT("ResolvePenetration:   sweep by %s (Original move, attempt success = %d)"),
                               *(moveDelta).ToString(),
                               bMoved);
                    }
                }
            }

            return bMoved;
        }
    }

    return false;
}
#endif
