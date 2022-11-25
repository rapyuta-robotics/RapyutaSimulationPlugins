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
    URRFloatingMovementComponent(const FObjectInitializer& ObjectInitializer);
    void Set2DMovementEnabled(bool bEnabled)
    {
        b2DMovement = bEnabled;
    }

    void SetUseDecelerationForPaths(bool bEnabled)
    {
        bUseDecelerationForPaths = bEnabled;
    }
    FORCEINLINE bool UseDecelerationForPathFollowing() const
    {
        return bUseDecelerationForPaths;
    }

    void SetUseAccelerationForPaths(bool bEnabled)
    {
        bUseAccelerationForPaths = bEnabled;
    }

    void SetUseConstantVelocity(bool bConstantVelocity)
    {
        bUseConstantVelocity = bConstantVelocity;
        if (bConstantVelocity)
        {
            bUseAccelerationForPaths = false;
            bUseDecelerationForPaths = false;
        }
    }

    FORCEINLINE bool UseConstantVelocity() const
    {
        return bUseConstantVelocity;
    }

protected:
    virtual void TickComponent(float InDeltaTime, enum ELevelTick InTickType, FActorComponentTickFunction* InTickFunction) override;
    virtual bool IsExceedingMaxSpeed(float InMaxSpeed) const override;
#if RAPYUTA_SIM_DEBUG
    virtual bool ResolvePenetrationImpl(const FVector& InProposedAdjustment,
                                        const FHitResult& InHit,
                                        const FQuat& InNewRotationQuat) override;
#endif
private:
    UPROPERTY(EditAnywhere)
    uint8 bSweepEnabled : 1;

    UPROPERTY(EditAnywhere)
    uint8 b2DMovement : 1;

    UPROPERTY(EditAnywhere)
    uint8 bUseDecelerationForPaths : 1;

    UPROPERTY(EditAnywhere)
    uint8 bUseConstantVelocity : 1;
};
