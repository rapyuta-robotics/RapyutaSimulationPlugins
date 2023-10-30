/**
 * @file DifferentialDriveComponent.h
 * @brief Differential Drive component class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Drives/DifferentialDriveComponentBase.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include <random>

#include "DifferentialDriveComponent.generated.h"

/**
 * @brief Differential Drive component class.
 * Simulate differential drive by using 2 UPhysicsConstraintComponent.
 * Calculate wheel rotation from given Velocity(member of UMovementComponent) and #AngularVelocity and set by calling SetAngularVelocityTarget
 * Publish odometry from Velocity and #AngularVelocity.
 *
 * @sa [UPhysicsConstraintComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/)
 * @sa [SetAngularVelocityTarget](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularVeloci-_3/)
 *
 * @todo Calculate odom from wheel rotation.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UDifferentialDriveComponent : public UDifferentialDriveComponentBase
{
    GENERATED_BODY()

public:
    /**
     * @brief Calculate wheel velocity from Velocity(member of UMovementComponent) and #AngularVelocity, and set by calling SetAngularVelocityTarget
     * SetAngularDriveParams as well.
     * @param DeltaTime
     * @sa [UPhysicsConstraintComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/)
     * @sa [SetAngularVelocityTarget](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularVeloci-_3/)
     * @sa [SetAngularDriveParams](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/PhysicsEngine/UPhysicsConstraintComponent/SetAngularDriveP-/)
     */
    virtual void UpdateMovement(float DeltaTime) override;

    /**
     * @brief Set left and right wheels.
     *
     * @param InWheelLeft
     * @param InWheelRight
     */
    UFUNCTION(BlueprintCallable)
    void SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight);

    /**
     * @brief Get the Wheel Velocity [cm/s]
     *
     * @param index index of wheels
     */
    virtual float GetWheelVelocity(const EDiffDriveWheel WheelIndex) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* WheelRight = nullptr;
};
