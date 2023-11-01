/**
 * @file DifferentialDriveComponentBase.h
 * @brief Differential Drive component base class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include <random>

#include "DifferentialDriveComponentBase.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogDifferentialDriveComponent, Log, All);

/**
 * @brief Wheel type of differential drive
 */
UENUM(BlueprintType)
enum class EDiffDriveWheel : uint8
{
    RIGHT UMETA(DisplayName = "RIGHT"),
    LEFT UMETA(DisplayName = "LEFT")
};

/**
 * @brief Differential Drive component base class.
 * Please check #UDifferentialDriveComponent and #URRDifferentialDriveComponent as a example child component.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UDifferentialDriveComponentBase : public URobotVehicleMovementComponent
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
     * @brief Get the Wheel Velocity [cm/s]
     *
     * @param index index of wheels
     */
    UFUNCTION(BlueprintCallable)
    virtual float GetWheelVelocity(const EDiffDriveWheel WheelIndex);

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

    //! [cm]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelRadius = 1.f;

    //! [cm] @todo get data from links
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelSeparationHalf = 1.f;

    //! @todo get data from physics constraints
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxForce = 1000.f;

protected:
    //! [cm]
    UPROPERTY()
    float WheelPerimeter = 6.28f;

    //! [cm]
    UPROPERTY()
    float PoseEncoderX = 0.f;
    //! [cm]
    UPROPERTY()
    float PoseEncoderY = 0.f;
    //! [rad]
    UPROPERTY()
    float PoseEncoderThetaRad = 0.f;
};
