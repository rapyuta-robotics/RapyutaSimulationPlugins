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
     * @brief Initialize #JointToChildLink and #ParentLinkToJoint
     * 
     */
    virtual void Initialize() override;

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

    //! [kg/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearSpring = 1000;

    //! [kg/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearBamper = 10;

    //! [kg.cm/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearForceLimit = UE_BIG_NUMBER;

    //! [kg.cm/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularSpring = 100000;

    //! [kg.cm/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularBamper = 10000;

    //! [kg.deg.deg/ss]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularForceLimit = UE_BIG_NUMBER;

    //! if false, physics constraints parameters are set from #SetJoint .
    //! Plese set true if you want to set physics constraints parameter manually.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bManualConstraintSetting = false;

protected:

    //! LinearVelocityTarget for smoothing
    FVector MidLinearVelocityTarget = FVector::ZeroVector;

    //! AngularVelocityTarget for smoothing
    FVector MidAngularVelocityTarget = FVector::ZeroVector;

    //! PositionTarget for smoothing
    FVector MidPositionTarget = FVector::ZeroVector;

    //! OrientationTarget for smoothing
    FRotator MidOrientationTarget = FRotator::ZeroRotator;

    virtual void UpdateState(const float DeltaTime);

    virtual void UpdateControl(const float DeltaTime);

    TStaticArray<TwoPointInterpolation, 3> PositionTPI;
    TStaticArray<TwoAngleInterpolation, 3> OrientationTPI;
};
