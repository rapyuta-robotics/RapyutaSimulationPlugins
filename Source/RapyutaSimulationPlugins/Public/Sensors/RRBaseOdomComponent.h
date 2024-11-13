/**
 * @file RRBaseOdomComponent.h
 * @brief Base Odom Component which provides actor pose changes.
 * @copyright Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Sensors/RRROS2EntityStateSensorComponent.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/SimulationState.h"

#include "RRBaseOdomComponent.generated.h"

class ARRBaseRobot;

/**
 * @brief Type of odometry frame origin.
 * World provide odometry from world origin and Encoder provide odometry from initial pose.
 */
UENUM(BlueprintType)
enum class EOdomSource : uint8
{
    WORLD UMETA(DisplayName = "World"),
    ENCODER UMETA(DisplayName = "Encoder")
};

/**
 * @brief Base Odom Component which provide actor pose changes.
 * Default odom calculation is done by differentiate current pose and last pose.
 * You can create child odom source class from this class or update odom data directly
 * with bManualUpdate=true to avoid updating data by this class.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRBaseOdomComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    URRBaseOdomComponent();

    /**
     * @brief Calculate relative pose with #URRGeneralUtils and update #Data
     */
    virtual void SensorUpdate() override;

    virtual void PreInitializePublisher(UROS2NodeComponent* InROS2Node, const FString& InTopicName) override;

    UPROPERTY(BlueprintReadWrite)
    TWeakObjectPtr<ARRBaseRobot> RobotVehicle = nullptr;

    //! If this is true, SensorUpdate do nothing.
    //! Since odometry calculation is depends on movement component,
    //! you can make this true to and manually update Odomdata by movement component
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    bool bManualUpdate = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSOdom OdomData;

    //! Child frame id of odometry
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId = TEXT("base_footprint");

    /**
     * @brief Set the Frame Id and child frame id of odometry
     *
     * @param InFrameId
     * @param InChildFrameId
     */
    void SetFrameIds(const FString& InFrameId, const FString& InChildFrameId);

    UPROPERTY(EditAnywhere)
    FTransform InitialTransform = FTransform::Identity;

    UFUNCTION(BlueprintCallable)
    FTransform GetOdomTF() const;

    UFUNCTION(BlueprintCallable)
    virtual void InitOdom();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    EOdomSource OdomSource = EOdomSource::WORLD;

    /**
     * @brief Update odom.
     * Noise is integral of gaussian noise.
     * @param InDeltaTime
     */
    virtual void UpdateOdom(float InDeltaTime);

    //! Publish tf or not
    //! @todo move this to publisher
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bPublishOdomTf = false;

    UPROPERTY(VisibleAnywhere)
    bool bIsOdomInitialized = false;

    //! Offset transform between the Owner Actor root component and the pose that will be published in /odom topic
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform RootOffset = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    TObjectPtr<URRGaussianNoise> PositionNoise;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    TObjectPtr<URRGaussianNoise> RotNoise;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    float NoiseMeanPosition = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    float NoiseVariancePosition = 0.001f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    float NoiseMeanRot = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    float NoiseVarianceRot = 0.005f;

    //! Add noise or not
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    bool bWithNoise = true;

protected:
    float LastUpdatedTime = 0.f;

    UPROPERTY()
    FTransform PreviousTransform = FTransform::Identity;

    UPROPERTY()
    FTransform PreviousNoisyTransform = FTransform::Identity;
};
