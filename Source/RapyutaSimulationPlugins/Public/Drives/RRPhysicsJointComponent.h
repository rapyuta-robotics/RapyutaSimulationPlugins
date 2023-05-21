/**
 * @file PhysicsJointComponent.h
 * @brief Physics Joint component class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "Drives/RRJointComponent.h"

#include "RRPhysicsJointComponent.generated.h"

/**
 * @brief Physics Joint component. 
 * PhysicsConstraintsComponent needs to be defined outside of this class and passed to #Constraint in construction.
 * @sa[PhysicsConstraints](https://docs.unrealengine.com/4.26/en-US/InteractiveExperiences/Physics/Constraints/ConstraintsBlueprints/)
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRPhysicsJointComponent : public URRJointComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    URRPhysicsJointComponent();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;

public:
    /**
     * @brief 
     *
     * @param DeltaTime
     * @param TickType
     * @param ThisTickFunction
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


    /**
     * @brief Do nothing since can't set velocity directly to physics joint.
     * @param InLinearVelocity
     * @param InAngularVelocity
     */
    virtual void SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity) override;


    /**
     * @brief Call SetLinearVelocityTarget and SetAngularVelocityTarget
     * @sa[SetLinearVelocityTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetLinearVelocit-_1/)
     * @sa[SetAngularVelocityTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularVelocit-_1/)
     */
    virtual void SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity) override;

    /**
     * @brief Do nothing since can't set pose directly to physics joint.
     * @param InPosition
     * @param InOrientation
     */
    virtual void SetPose(const FVector& InPosition, const FRotator& InOrientation) override;


    /**
     * @brief Call SetLinearPositionTarget and SetAngularOrientationTarget
     * @sa[SetLinearPositionTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetLinearPositio-_1/)
     * @sa[SetAngularVelocityTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularOrient-_1/)
     */
    virtual void SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation) override;

    /**
     * Set joints parameter and etc
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetJoint();

    //! Physics Constraints
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Constraint = nullptr;

    //! Smoothing TargetPose to #Constraint.
    //! If this is false, step pose target are used by #SetPoseTarget
    //! If this is true, pose target changes linearly with max vel in #TickComponent
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bSmoothing = false;

    //! Acceleration[cm/ss] used by velocity smoothing if #bVelocitySmoothing = true.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearVelocitySmoothingAcc = 10;

    //! Acceleration[deg/ss] used by velocity smoothing if #bVelocitySmoothing = true.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularVelocitySmoothingAcc = 10;

    //! [cm] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float PositionTolerance = 1;

    //! [degree] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OrientationTolerance = 1;

    //! [cm/s] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearVelocityTolerance = 10;
    
    //! [degree/s] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularVelocityTolerance = 10;

protected:

    //! LinearVelocityTarget for smoothing
    FVector MidLinearVelocityTarget = FVector::ZeroVector;

    //! AngularVelocityTarget for smoothing
    FVector MidAngularVelocityTarget = FVector::ZeroVector;

    //! PositionTarget for smoothing
    FVector MidPositionTarget = FVector::ZeroVector;

    //! OrientationTarget for smoothing
    FRotator MidOrientationTarget = FRotator::ZeroRotator;

    //todo move to general utils?

    /**
     * @brief update current value with step to reach target within tolerance
     * 
     * @param current 
     * @param target 
     * @param step 
     * @param tolerance 
     */
    virtual void StepUpdateFloat(double& current, const double target, const double step, const double tolerance);

    /**
     * @brief update current value with step to reach target within tolerance
     * 
     * @param current 
     * @param target 
     * @param step 
     * @param tolerance 
     */
    virtual void StepUpdateVector(FVector& current, const FVector target, const FVector step, const double tolerance);

    /**
     * @brief update current value with step to reach target within tolerance
     * 
     * @param current 
     * @param target 
     * @param step 
     * @param tolerance 
     */
    virtual void StepUpdateRotator(FRotator& current, const FRotator target, const FVector step, const double tolerance);

};
