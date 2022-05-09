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
 * - Joints moves with Max, Min velocity to target with #EJointControlType::POSITION
 * - Joints moves with given velocity with #EJointControlType::VELOCITY
 * @sa[K2_SetWorldTransform](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/Components/USceneComponent/K2_SetWorldTransform/)
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRKinematicJointComponent : public URRJointComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    URRKinematicJointComponent();

protected:
    virtual void BeginPlay() override;

    FTransform ParentLinkToJoint;

public:
    /**
     * @brief Call #UpdatePose after update #PositionTarget and #OrientationTarget with #LinearVelocity and AngularVelocity
     *
     * @param DeltaTime
     * @param TickType
     * @param ThisTickFunction
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

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
};
