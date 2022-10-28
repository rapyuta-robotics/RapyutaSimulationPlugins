// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRFloatingMovementComponent.h"

URRFloatingMovementComponent::URRFloatingMovementComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    // NOTE: Let itself be manually activated & tick then to wait for [ExemptedCollidingCompList] to be confirmed
    bAutoActivate = false;
    bAutoUpdateTickRegistration = false;
    bTickBeforeOwner = false;
}

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

    //LimitWorldBounds();
    bPositionCorrected = false;

    // Move [UpdatedComponent], updating [bPositionCorrected] here-in
    FVector deltaLoc = Velocity * InDeltaTime;
    if (!deltaLoc.IsNearlyZero(1e-6f))
    {
        // Save prevLocation
        const FVector prevLocation = UpdatedComponent->GetComponentLocation();
        FHitResult hit(1.f);
        SafeMoveTargetWithCollisionExemption(deltaLoc, UpdatedComponent->GetComponentQuat(), bSweepEnabled, hit);

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

#if 1    // RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("ResolvePenetration: %s.%s at location %s inside %s.%s at location %s by %.3f (netmode: %d)"),
               *ownerActor->GetName(),
               *UpdatedComponent->GetName(),
               *UpdatedComponent->GetComponentLocation().ToString(),
               *GetNameSafe(InHit.GetActor()),
               *GetNameSafe(InHit.GetComponent()),
               InHit.Component.IsValid() ? *InHit.GetComponent()->GetComponentLocation().ToString() : TEXT("<unknown>"),
               InHit.PenetrationDepth,
               static_cast<uint32>(GetNetMode()));
#endif

        if (ExemptedCollidingCompList.Contains(InHit.GetComponent()))
        {
#if 1    // RAPYUTA_SIM_DEBUG
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("%s ResolvePenetrationImpl - EXEMPTED hit comp %s"),
                   *GetName(),
                   *InHit.GetComponent()->GetName());
#endif
            // Retry original move WITHOUT Sweep
            MoveUpdatedComponent(constrainedAdjustment, InNewRotationQuat, false, nullptr, ETeleportType::TeleportPhysics);
            return true;
        }

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
#if 1    // RAPYUTA_SIM_DEBUG
            UE_LOG(LogRapyutaCore, Error, TEXT("ResolvePenetration: teleport by %s"), *constrainedAdjustment.ToString());
#endif
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

#if 1    // RAPYUTA_SIM_DEBUG
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("ResolvePenetration: sweep by %s (success = %d)"),
                   *constrainedAdjustment.ToString(),
                   bMoved);
#endif

            // Try sweep again - 1st
            if (!bMoved && outSweepHit.bStartPenetrating)
            {
                // Combine two MTD results to get a new direction that gets out of multiple surfaces.
                const FVector secondMTD = GetPenetrationAdjustment(outSweepHit);
                const FVector combinedMTD = constrainedAdjustment + secondMTD;
                if (secondMTD != constrainedAdjustment && !combinedMTD.IsZero())
                {
                    bMoved = MoveUpdatedComponent(combinedMTD, InNewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
#if 1    // RAPYUTA_SIM_DEBUG
                    UE_LOG(LogRapyutaCore,
                           Error,
                           TEXT("ResolvePenetration: sweep by %s (MTD combo success = %d)"),
                           *combinedMTD.ToString(),
                           bMoved);
#endif
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
#if 1    // RAPYUTA_SIM_DEBUG
                    UE_LOG(LogRapyutaCore,
                           Error,
                           TEXT("ResolvePenetration: sweep by %s (adjusted attempt success = %d)"),
                           *(constrainedAdjustment + moveDelta).ToString(),
                           bMoved);
#endif

                    // Finally, try the original move without MTD adjustments, but allowing depenetration along the MTD normal.
                    // This was blocked because MOVECOMP_NeverIgnoreBlockingOverlaps was true for the original move to try a better depenetration normal, but we might be running in to other geometry in the attempt.
                    // This won't necessarily get us all the way out of penetration, but can in some cases and does make progress in exiting the penetration.
                    if (!bMoved && FVector::DotProduct(moveDelta, constrainedAdjustment) > 0.f)
                    {
                        bMoved = MoveUpdatedComponent(moveDelta, InNewRotationQuat, true, nullptr, ETeleportType::TeleportPhysics);
#if 1    // RAPYUTA_SIM_DEBUG
                        UE_LOG(LogRapyutaCore,
                               Error,
                               TEXT("ResolvePenetration:   sweep by %s (Original move, attempt success = %d)"),
                               *(moveDelta).ToString(),
                               bMoved);
#endif
                    }
                }
            }

            return bMoved;
        }
    }

    return false;
}

bool URRFloatingMovementComponent::SafeMoveTargetWithCollisionExemption(const FVector& InDeltaLoc,
                                                                        const FQuat& InNewRotation,
                                                                        bool bSweep,
                                                                        FHitResult& OutHit,
                                                                        const ETeleportType InTeleportType)
{
    if (UpdatedComponent == NULL)
    {
        OutHit.Reset(1.f);
        return false;
    }

    bool bMoveResult = false;

    // Scope for move flags
    {
        // NOT IGNORE blocking overlaps
        const EMoveComponentFlags IncludeBlockingOverlapsWithoutEvents =
            (MOVECOMP_NeverIgnoreBlockingOverlaps | MOVECOMP_DisableBlockingOverlapDispatch);
        TGuardValue<EMoveComponentFlags> ScopedFlagRestore(
            MoveComponentFlags,
            //MovementComponentCVars::MoveIgnoreFirstBlockingOverlap
            false ? MoveComponentFlags : (MoveComponentFlags | IncludeBlockingOverlapsWithoutEvents));
        bMoveResult = MoveUpdatedComponent(InDeltaLoc, InNewRotation, bSweep, &OutHit, InTeleportType);
    }

    // Handle initial penetration
    if (OutHit.bStartPenetrating && UpdatedComponent)
    {
        if (ExemptedCollidingCompList.Contains(OutHit.GetComponent()))
        {
#if 1    // RAPYUTA_SIM_DEBUG
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("%s Handle initial penetration - EXEMPTED hit comp %s"),
                   *GetName(),
                   *OutHit.GetComponent()->GetName());
#endif
            // Retry original move WITHOUT Sweep
            bMoveResult = MoveUpdatedComponent(InDeltaLoc, InNewRotation, false, &OutHit, InTeleportType);
        }
        else
        {
            if (ResolvePenetration(GetPenetrationAdjustment(OutHit), OutHit, InNewRotation))
            {
                // Retry original move
                bMoveResult = MoveUpdatedComponent(InDeltaLoc, InNewRotation, bSweep, &OutHit, InTeleportType);
            }
        }
    }

    return bMoveResult;
}
