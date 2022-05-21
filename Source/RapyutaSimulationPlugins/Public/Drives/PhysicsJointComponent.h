/**
 * @file PhysicsJointComponent.h
 * @brief Physics Joint component class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "Drives/JointComponent.h"

#include "PhysicsJointComponent.generated.h"

/**
 * @brief Physics Joint component. Uses PhysicsConstraints.
 * Expects Joint(this) is child component of #ParentLink and #ChildLink is child component of Joint(this), which relations are set in #BeginPlay.
 * @sa [PhysicsConstraints](https://docs.unrealengine.com/4.26/en-US/InteractiveExperiences/Physics/Constraints/ConstraintsBlueprints/)
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UPhysicsJointComponent : public UJointComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    UPhysicsJointComponent();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;

public:
    /**
     * @brief Call SetLinearVelocityTarget and SetAngularVelocityTarget
     * @sa [SetLinearVelocityTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularVelocityTarget/)
     * @sa [SetAngularVelocityTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetLinearVelocityTarget/)
     */
    virtual void SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity) override;

    /**
     * @brief Call SetLinearPositionTarget and SetAngularOrientationTarget
     * @sa [SetLinearPositionTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetLinearPositio-_1/)
     * @sa [SetAngularVelocityTarget](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularOrientationTarget/)
     */
    virtual void SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation) override;

    /**
     * Set joints parameter and etc
     */ 
    UFUNCTION(BlueprintCallable)
    virtual void SetJoint();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Constraint = nullptr;
};
