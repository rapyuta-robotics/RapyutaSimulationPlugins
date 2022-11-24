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
    URRFloatingMovementComponent();
    URRFloatingMovementComponent(const FObjectInitializer& ObjectInitializer);
    UPROPERTY()
    bool bSweepEnabled = true;

protected:
    virtual void TickComponent(float InDeltaTime, enum ELevelTick InTickType, FActorComponentTickFunction* InTickFunction) override;
#if RAPYUTA_SIM_DEBUG
    virtual bool ResolvePenetrationImpl(const FVector& InProposedAdjustment,
                                        const FHitResult& InHit,
                                        const FQuat& InNewRotationQuat) override;
#endif
};
