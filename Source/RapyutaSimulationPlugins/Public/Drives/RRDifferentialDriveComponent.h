/**
 * @file RRDifferentialDriveComponent.h
 * @brief RRDifferential Drive component class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Drives/DifferentialDriveComponentBase.h"
#include "Drives/RRPhysicsJointComponent.h"

#include <random>

#include "RRDifferentialDriveComponent.generated.h"

/**
 * @brief Differential Drive component class.
 * Simulate differential drive by using 2 #URRPhysicsJointComponent.
 * Calculate wheel rotation from given Velocity(member of UMovementComponent) and #AngularVelocity and set by calling SetAngularVelocityTarget
 * Publish odometry from wheel rotation.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRDifferentialDriveComponent : public UDifferentialDriveComponentBase
{
    GENERATED_BODY()

public:
    /**
     * @brief Calculate wheel velocity from Velocity(member of UMovementComponent) and #AngularVelocity, and set by calling SetAngularVelocityTarget
     * @param DeltaTime
     */
    virtual void UpdateMovement(float DeltaTime) override;

    /**
     * @brief Set left and right wheels.
     *
     * @param InWheelLeft
     * @param InWheelRight
     */
    UFUNCTION(BlueprintCallable)
    void SetWheels(URRPhysicsJointComponent* InWheelLeft, URRPhysicsJointComponent* InWheelRight);

    /**
     * @brief Get the Wheel Velocity [cm/s]
     *
     * @param index index of wheels
     */
    virtual float GetWheelVelocity(const EDiffDriveWheel WheelIndex) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRPhysicsJointComponent* WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRPhysicsJointComponent* WheelRight = nullptr;
};
