/**
 * @file DifferentialDriveComponent.h
 * @brief Differential Drive component class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include <random>

#include "DifferentialDriveComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogDifferentialDriveComponent, Log, All);

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
class RAPYUTASIMULATIONPLUGINS_API UDifferentialDriveComponent : public URobotVehicleMovementComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Call #UpdateOdom in addition to update movement
     *
     * @param DeltaTime
     * @param TickType
     * @param ThisTickFunction
     *
     * @sa
     * [UpdateComponentVelocity](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/UpdateComponentVelocity/)
     */
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

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
     * @brief Calculate odometry from Velocity and #AngularVelocity.
     *
     * @param DeltaTime
     *
     * @todo Calculate odom from wheel rotation.
     */
    virtual void UpdateOdom(float DeltaTime);

    /**
     * @brief Set left and right wheels.
     *
     * @param InWheelLeft
     * @param InWheelRight
     */
    UFUNCTION(BlueprintCallable)
    void SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight);

    /**
     * @brief Call Super::Initialize() and #SetPerimeter.
     *
     */
    virtual void Initialize() override;

    /**
     * @brief SetPerimeter from #WheelRadius * 2.f * M_PI
     *
     */
    UFUNCTION(BlueprintCallable)
    void SetPerimeter();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* WheelRight = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelRadius = 1.f;

    //! @todo get data from links
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelSeparationHalf = 1.f;

    //! @todo get data from physics constraints
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxForce = 1000.f;

protected:
    UPROPERTY()
    float WheelPerimeter = 6.28f;

    UPROPERTY()
    float PoseEncoderX = 0.f;
    UPROPERTY()
    float PoseEncoderY = 0.f;
    UPROPERTY()
    float PoseEncoderTheta = 0.f;
};
