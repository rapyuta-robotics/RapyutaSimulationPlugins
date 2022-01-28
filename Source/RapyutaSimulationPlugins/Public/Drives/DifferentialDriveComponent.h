// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include <random>

#include "DifferentialDriveComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogDifferentialDriveComponent, Log, All);

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UDifferentialDriveComponent : public URobotVehicleMovementComponent
{
    GENERATED_BODY()

public:
    virtual void UpdateMovement(float DeltaTime) override;
    virtual void UpdateOdom(float DeltaTime) override;

    UFUNCTION(BlueprintCallable)
    void SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight);

    virtual void Initialize() override;

    UFUNCTION(BlueprintCallable)
    void SetPerimeter();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* WheelRight = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelRadius = 1.f;

    // todo get data from links
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelSeparationHalf = 1.f;

    // todo get data from physics constraints
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxForce = 1000.f;

    UPROPERTY()
    float WheelPerimeter = 2 * PI;

protected:
    UPROPERTY()
    double PoseEncoderX = 0.f;
    UPROPERTY()
    double PoseEncoderY = 0.f;
    UPROPERTY()
    double PoseEncoderTheta = 0.f;

    // From controller
    FORCEINLINE virtual float GetDesiredWheelSpeed(bool bIsLeftWheel) const
    {
        return GetDesiredForwardReverseVelocity() +
               (bIsLeftWheel ? 1.f : -1.f) * GetDesiredSteeringVelocity() * WheelSeparationHalf;
    }

    // From encoder (revolution/sec)
    virtual double GetLeftWheelSpeed() const
    {
        return GetDesiredWheelSpeed(true);
    }
    virtual double GetRightWheelSpeed() const
    {
        return GetDesiredWheelSpeed(false);
    }
};
