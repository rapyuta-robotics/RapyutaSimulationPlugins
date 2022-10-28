/**
 * @file RRFloatingMovementComponent.h
 * @brief Base Robot floating movement class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "GameFramework/FloatingPawnMovement.h"

#include "RRFloatingMovementComponent.generated.h"

/**
 * @brief Base Robot floating movement class
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRFloatingMovementComponent : public UFloatingPawnMovement
{
    GENERATED_BODY()

public:
    URRFloatingMovementComponent()
    {
    }
    URRFloatingMovementComponent(const FObjectInitializer& ObjectInitializer);
    UPROPERTY()
    bool bSweepEnabled = true;

    // Colliding comps to be moved through without sweep, which prevents movement upon collision
    UPROPERTY()
    TArray<USceneComponent*> ExemptedCollidingCompList;

protected:
    virtual void TickComponent(float InDeltaTime, enum ELevelTick InTickType, FActorComponentTickFunction* InTickFunction) override;
    virtual bool ResolvePenetrationImpl(const FVector& InProposedAdjustment,
                                        const FHitResult& InHit,
                                        const FQuat& InNewRotationQuat) override;
    bool SafeMoveTargetWithCollisionExemption(const FVector& InDeltaLoc,
                                              const FQuat& InNewRotation,
                                              bool bSweep,
                                              FHitResult& OutHit,
                                              const ETeleportType InTeleportType = ETeleportType::None);
};
