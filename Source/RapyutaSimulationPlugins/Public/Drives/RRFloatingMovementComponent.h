/**
 * @file RRFloatingMovementComponent.h
 * @brief Base Robot floating movement class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "AI/Navigation/AvoidanceManager.h"
#include "GameFramework/FloatingPawnMovement.h"

#include "RRFloatingMovementComponent.generated.h"

#define RAPYUTA_FLOAT_MOVEMENT_DEBUG (0)
/**
 * @brief Base Robot floating movement class
 * This is MovementComponent to move Robot with UE's AIController.
 * This is useful to develop/test higher level logics such as multi robot coordination without emulate low level navigation.
 * Support 2D movement with stick pawn to the floor.
 * @sa [UFloatingPawnMovement](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UFloatingPawnMovement/)
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRFloatingMovementComponent : public UFloatingPawnMovement
{
    GENERATED_BODY()

public:
    URRFloatingMovementComponent();
    URRFloatingMovementComponent(const FObjectInitializer& ObjectInitializer);
    void SetupDefault();
    virtual void SetUpdatedComponent(USceneComponent* InNewUpdatedComponent) override;
    void Set2DMovement(bool bEnabled)
    {
        b2DMovement = bEnabled;
    }
    FORCEINLINE bool Is2DMovement() const
    {
        return b2DMovement;
    }

    //! [deg/s] Current AngularVelocity of #UpdatedComponent [X:Roll - Y:Pitch - Z: Yaw]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelocity = FVector::ZeroVector;

    //! Maximum angular speed magnitude allowed for #UpdatedComponent
    //! [deg/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxAngularSpeed = 360.f;

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

    virtual void StopMovementImmediately() override;

    // RVO Avoidance
    // Ref: UCharacterMovementComponent
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    int32 RVOAvoidanceUID = 0;
    //! Whether to use RVO avoidance (instead of crowd avoidance). This only runs on the server
    UPROPERTY(VisibleAnywhere)
    uint8 bUseRVOAvoidance : 1;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float RVOAvoidanceWeight = 0.5f;
    UPROPERTY(Transient)
    uint8 bRVOAvoidanceRecentlyUpdated : 1;
    UPROPERTY()
    TObjectPtr<UAvoidanceManager> RVOAvoidanceManager = nullptr;
    void SetRVOAvoidanceEnabled(bool bEnabled);
    void SetRVOAvoidanceVelocityLock(float InDuration);
    //! Forced avoidance velocity if AvoidanceLockTimer > 0
    UPROPERTY()
    FVector RVOAvoidanceLockVelocity = FVector::ZeroVector;
    UPROPERTY()
    float RVOAvoidanceLockTimeout = 0.f;

protected:
    virtual void OnRegister() override;
    virtual void TickComponent(float InDeltaTime, enum ELevelTick InTickType, FActorComponentTickFunction* InTickFunction) override;
    virtual bool IsExceedingMaxSpeed(float InMaxSpeed) const override;
    virtual FVector GetPenetrationAdjustment(const FHitResult& Hit) const;
#if RAPYUTA_FLOAT_MOVEMENT_DEBUG
    virtual bool ResolvePenetrationImpl(const FVector& InProposedAdjustment,
                                        const FHitResult& InHit,
                                        const FQuat& InNewRotationQuat) override;
#endif
    void UpdateDefaultAvoidance();
    void CalculateRVOAvoidanceVelocity(float InDeltaTime);
    virtual void PostProcessRVOAvoidanceVelocity(FVector& InNewVelocity)
    {
    }

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
