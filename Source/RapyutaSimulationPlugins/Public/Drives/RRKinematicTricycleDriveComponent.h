/**
 * @file RRKinematicTricycleDriveComponent.h
 * @brief Differential Drive component class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Drives/RRJointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"

#include "RRKinematicTricycleDriveComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogRRKinematicTricycleDriveComponent, Log, All);

/**
 * @brief Kinematic Tricycle Drive component class.
 * Simulate kinematic tricycle drive by using 2 URRJointComponent SteeringJoint + DriveJoint.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRKinematicTricycleDriveComponent : public URobotVehicleMovementComponent
{
    GENERATED_BODY()

public:
    void BeginPlay();

    void TickComponent(float InDeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction);

    UFUNCTION(BlueprintCallable)
    void SetDriveJoints(URRJointComponent* InSteeringJoint, URRJointComponent* InDriveJoint);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRJointComponent* SteeringJoint = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRJointComponent* DriveJoint = nullptr;

    UPROPERTY(BlueprintReadWrite)
    float WheelRadius = 12.7f;

    //! Center of rotation to RearWheel
    UPROPERTY(BlueprintReadWrite)
    float WheelBase = 110.f;

protected:
    //! [rad]
    UPROPERTY()
    float PrevWheeAngleRad = 0.f;
};
