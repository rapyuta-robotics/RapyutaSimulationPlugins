/**
 * @file KinematicTricycleDriveComponent.h
 * @brief Differential Drive component class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Drives/RRJointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"

#include "KinematicTricycleDriveComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogKinematicTricycleDriveComponent, Log, All);

/**
 * @brief Differential Drive component class.
 * Simulate differential drive by using 2 UPhysicsConstraintComponent.
 * Calculate wheel rotation from given Velocity(member of UMovementComponent) and #AngularVelocity and set by calling
 * SetAngularVelocityTarget Publish odometry from Velocity and #AngularVelocity.
 *
 * @sa
 * [UPhysicsConstraintComponent](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/)
 * @sa
 * [SetAngularVelocityTarget](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularVeloci-_3/)
 *
 * @todo Calculate odom from wheel rotation.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UKinematicTricycleDriveComponent : public URobotVehicleMovementComponent
{
    GENERATED_BODY()

public:
    void BeginPlay();

    void TickComponent(float InDeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction);

    UFUNCTION(BlueprintCallable)
    void SetDriveJoints(URRJointComponent* InSteeringJoint, URRJointComponent* InDriveJoint);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRJointComponent* SteeringJoint;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRJointComponent* DriveJoint;

    UPROPERTY(BlueprintReadWrite)
    float WheelRadius = 12.7;

    //! Center of rotation to RearWheel
    UPROPERTY(BlueprintReadWrite)
    float WheelBase = 110;

protected:
    UPROPERTY()
    float PrevWheeAngle;
};
