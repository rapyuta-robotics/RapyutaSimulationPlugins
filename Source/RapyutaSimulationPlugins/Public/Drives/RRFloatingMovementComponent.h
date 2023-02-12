/**
 * @file RRFloatingMovementComponent.h
 * @brief Base Robot floating movement class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "GameFramework/FloatingPawnMovement.h"

#include "RRFloatingMovementComponent.generated.h"

#define RAPYUTA_FLOAT_MOVEMENT_DEBUG (0)
/**
 * @brief Base Robot floating movement class
 * Support 2D movement with stick pawn to the floor.
 * @sa [UFloatingPawnMovement](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UFloatingPawnMovement/)
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRFloatingMovementComponent : public UFloatingPawnMovement
{
    GENERATED_BODY()

public:
    URRFloatingMovementComponent(const FObjectInitializer& ObjectInitializer);
    void Set2DMovement(bool bEnabled)
    {
        b2DMovement = bEnabled;
    }
    FORCEINLINE bool Is2DMovement() const
    {
        return b2DMovement;
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

    void SetPenetrationPullbackDistance(float PullbackDistance)
    {
        PenetrationPullbackDistance = PullbackDistance;
    }

protected:
    virtual void TickComponent(float InDeltaTime, enum ELevelTick InTickType, FActorComponentTickFunction* InTickFunction) override;
    virtual bool IsExceedingMaxSpeed(float InMaxSpeed) const override;
    virtual FVector GetPenetrationAdjustment(const FHitResult& Hit) const;
#if RAPYUTA_FLOAT_MOVEMENT_DEBUG
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
    float PenetrationPullbackDistance = 0.f;
};
