/**
 * @file KinematicJointComponent.h
 * @brief Kinematic Joint component class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Drives/RRJointComponent.h"

#include "RRKinematicJointComponent.generated.h"

/**
 * @brief Kinematic Joint component. Uses K2_SetWorldTransform to move joint.
 * Expects Joint(this) is child component of #ParentLink and #ChildLink is child component of Joint(this). Use initial pose
 * relations among parent link, joint and child link to calculate joint movement.
 *
 * #Controltype:
 * - Joints moves with Max, Min velocity to target with #ERRJointControlType::POSITION
 * - Joints moves with given velocity with #ERRJointControlType::VELOCITY
 * @sa[K2_SetWorldTransform](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/Components/USceneComponent/K2_SetWorldTransform/)
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRKinematicJointComponent : public URRJointComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    URRKinematicJointComponent();

    /**
     * @brief Initialize #JointToChildLink and #ParentLinkToJoint
     *
     */
    virtual void InitializeComponent() override;

public:
    /**
     * @brief Set velocity target
     * Velocity become same value as VelocityTarget with Kinematic Mode.
     * @param InLinearVelocity
     * @param InAngularVelocity
     */
    virtual void SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity) override;

    /**
     * @brief Super::SetPose + #UpdatePose
    */
    virtual void SetPose(const FVector& InPosition, const FRotator& InOrientation) override;

    /**
     * @brief Set the Pose Target. Set #LinearVelocity and AngularVelocity.
     * Velocity become Max, Min or Zero velocity.
     * @param InPosition
     * @param InOrientation
     */
    virtual void SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation) override;

    /**
     * @brief Update joints and #ChildLink pose by K2_SetWorldTransform with #PositionTarget and #OrientationTarget
     * @sa[K2_SetWorldTransform](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/Components/USceneComponent/K2_SetWorldTransform/)
     */
    virtual void UpdatePose();

    /**
     * @brief Teleport robot to given pose. Just call #SetPose.
     * @param InPosition
     * @param InOrientation
     */
    virtual void Teleport(const FVector& InPosition, const FRotator& InOrientation) override;

    virtual void MoveToInitPose();

protected:
    virtual void UpdateState(const float DeltaTime) override;

    virtual void UpdateControl(const float DeltaTime) override;
};
