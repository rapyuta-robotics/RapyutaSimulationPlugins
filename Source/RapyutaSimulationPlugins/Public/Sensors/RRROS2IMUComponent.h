/**
 * @file RRROS2IMUComponent.h
 * @brief IMU sensor components
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "EngineUtils.h"

// rclUE
#include "RRROS2BaseSensorComponent.h"

#include <Msgs/ROS2Imu.h>

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"

#include "RRROS2IMUComponent.generated.h"

/**
 * @brief EntityState sensor components which publish entitystate relative to a specific actor.
 * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
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
     * @brief Calculate relative pose with #URRGeneralUtils and update #Data
     * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
     */
    virtual void SensorUpdate() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform OffsetTransform = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform InitialTransform = FTransform::Identity;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTransform LastTransform = FTransform::Identity;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTransform LastdT = FTransform::Identity;

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
     * @return FROSEntityState
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

    FVector OrientationNoiseSum = FVector::ZeroVector;

    float LastSensorUpdateTime;
};
