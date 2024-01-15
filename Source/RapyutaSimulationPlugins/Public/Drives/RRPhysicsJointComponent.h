/**
 * @file PhysicsJointComponent.h
 * @brief Physics Joint component class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

//UE
#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "Kismet/KismetMathLibrary.h"
// #include "Containers/EnumAsByte.h"
#include <Containers/EnumAsByte.h>

//RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"
#include "Drives/RRJointComponent.h"

//Thirdparty
#include "two_points_interpolation_constant_acc.hpp"

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

public:
    virtual bool IsValid() override;

    /**
     * @brief Initialize #Constraint #JointToChildLink and #ParentLinkToJoint
     *
     */
    virtual void InitializeComponent() override;

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
     * @brief Disable collision and set target pose to given pose.
     * @note temporary implementation. it should teleported target pose instantly.
     *
     * @param InPosition
     * @param InOrientation
     */
    virtual void Teleport(const FVector& InPosition, const FRotator& InOrientation) override;

    virtual void MoveToInitPose();

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

    //! Position Velocity Gain used with #bSmoothing = false
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Kvp = 10.f;

    //! Angular Velocity Gain used with #bSmoothing = false.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Kva = 10.f;

    //! Position Integral Gain used with #bSmoothing = false
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Kip = 0.f;

    //! Angular Integral Gain used with #bSmoothing = false
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Kia = 0.f;

    //! Acceleration[cm/ss] used by velocity smoothing if #bVelocitySmoothing = true.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearVelocitySmoothingAcc = 10.f;

    //! Acceleration[deg/ss] used by velocity smoothing if #bVelocitySmoothing = true.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularVelocitySmoothingAcc = 10.f;

    //! [kg/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearSpring = 1000;

    //! [kg/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearDamper = 10;

    //! [kg.cm/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearForceLimit = UE_BIG_NUMBER;

    //! [kg.cm/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularSpring = 100000;

    //! [kg.cm/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularDamper = 10000;

    //! [kg.deg.deg/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularForceLimit = UE_BIG_NUMBER;

    //! if false, physics constraints parameters are set from #SetJoint .
    //! Plese set true if you want to set physics constraints parameter manually.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bManualConstraintSetting = false;

protected:
    //! LinearVelocityTarget for smoothing
    UPROPERTY(VisibleAnywhere)
    FVector MidLinearVelocityTarget = FVector::ZeroVector;

    //! AngularVelocityTarget for smoothing
    UPROPERTY(VisibleAnywhere)
    FVector MidAngularVelocityTarget = FVector::ZeroVector;

    //! PositionTarget for smoothing
    UPROPERTY(VisibleAnywhere)
    FVector MidPositionTarget = FVector::ZeroVector;

    //! OrientationTarget for smoothing
    UPROPERTY(VisibleAnywhere)
    FRotator MidOrientationTarget = FRotator::ZeroRotator;

    virtual void UpdateState(const float DeltaTime) override;

    virtual void UpdateControl(const float DeltaTime) override;

    UFUNCTION()
    /**
     * @brief Update #LinearVelocityTarget from InPositionDiff and #Kvp.
     * And update #AngularVelocityTarget from InOrientationDiff and #Kva.
     * @param InPositionDiff
     * @param InPositionDiff
     * @param InOrientationDiff
     * @param DeltaTime
     */
    virtual void UpdateIntegral(const FVector& InPositionDiff, const FVector& InOrientationDiff, const float DeltaTime);

    /**
     * @brief Update #LinearVelocityTarget from InPositionDiff and #Kvp.
     * And update #AngularVelocityTarget from InOrientationDiff and #Kva.
     * @param InPositionDiff
     * @param InOrientationDiff
     */
    UFUNCTION()
    virtual void UpdateVelocityTargetFromPose(const FVector InPositionDiff, const FVector InOrientationDiff);

    /**
     * @brief Get the Orientation Target to SetAngularOrientationTarget from Euler angle
     *
     * @param InOrientationTarget
     * @return FRotator
     */
    UFUNCTION()
    virtual FRotator GetOrientationTargetFromEuler(const FVector& InOrientationTarget);

    //! Position Two Point Interpolation used with #bSmoothing = true
    TStaticArray<TwoPointInterpolation, 3> PositionTPI;

    //! Angular Two Point Interpolation used with #bSmoothing = true
    TStaticArray<TwoAngleInterpolation, 3> OrientationTPI;

    //! Position Error Integral used with #bSmoothing = false
    UPROPERTY(VisibleAnywhere)
    FVector PErrInt = FVector::ZeroVector;

    //! Angular Error Integral used with #bSmoothing = false
    UPROPERTY(VisibleAnywhere)
    FVector AErrInt = FVector::ZeroVector;
};
