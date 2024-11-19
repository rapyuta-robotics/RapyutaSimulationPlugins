/**
 * @file RRROS2IMUComponent.h
 * @brief IMU sensor components
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "EngineUtils.h"
#include "Math/UnitConversion.h"

// rclUE
#include "RRROS2BaseSensorComponent.h"

#include <Msgs/ROS2Imu.h>

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"

#include "RRROS2IMUComponent.generated.h"

/**
 * @brief IMU sensor components which publish Orienation, AngularVelocity, LinearAcceleration.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2IMUComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRROS2IMUComponent object
     *
     */
    URRROS2IMUComponent();

    void BeginPlay() override;

    /**
     * @brief Calculate IMU data, i.e. Orienation, AngularVelocity, LinearAcceleration.
     */
    virtual void SensorUpdate() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform InitialTransform = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FVector LinearAcceleration = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FQuat Orientation = FQuat::Identity;

    /**
     * @brief Initialize sensor data
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void Reset();

    UPROPERTY(EditAnywhere, Category = "Noise")
    TObjectPtr<URRGaussianNoise> LinearAccelerationNoise;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseMeanLinearAcceleration = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseVarianceLinearAcceleration = 0.05f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    TObjectPtr<URRGaussianNoise> OrientationNoise;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseMeanOrientation = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseVarianceOrientation = 0.01f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float OrientationNoiseDriftCoefficient = 0.0f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    TObjectPtr<URRGaussianNoise> AngularVelocityNoise;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseMeanAngularVelocity = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseVarianceAngularVelocity = 0.01f;

    // ROS
    /**
     * @brief return #Data
     *
     * @return FROSImu
     */
    UFUNCTION(BlueprintCallable)
    virtual FROSImu GetROS2Data();

    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UPROPERTY(BlueprintReadWrite)
    FROSImu Data;

protected:
    UPROPERTY(BlueprintReadWrite)
    FVector OffsetOrientation = FVector::ZeroVector;

    // sensor drift
    FVector OrientationNoiseSum = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTransform LastTransform = FTransform::Identity;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector LastLinearVel = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTransform LastdT = FTransform::Identity;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float LastSensorUpdateTime;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bAddGravity = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AccGain = 1.0;
};
