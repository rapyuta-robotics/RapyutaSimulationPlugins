/**
 * @file RRPoseOdomComponent.h
 * @brief Robot pose sensor manager. If #RefActorSelectMode=AUTO, RefActor is automatically updated with nearlest actor with Tag
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Sensors/RRROS2EntityStateSensorComponent.h"
#include "Tools/SimulationState.h"

#include "RRPoseOdomComponent.generated.h"

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
 * @brief 
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRPoseOdomComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    URRPoseOdomComponent();

    virtual void BeginPlay() override;

    /**
     * @brief Calculate relative pose with #URRGeneralUtils and update #Data
     * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
     */
    virtual void SensorUpdate() override;

    virtual void PreInitializePublisher(UROS2NodeComponent* InROS2Node, const FString& InTopicName) override;

    UPROPERTY(BlueprintReadWrite)
    TWeakObjectPtr<ARRBaseRobot> RobotVehicle = nullptr;

    //! If this is true, SensorUpdate do nothing.
    //! Since odometry calculation is depends on movement component, 
    //! you can make this true to and manually update Odomdata by movement component
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    bool ManualUpdate = false;

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

    UPROPERTY(EditAnywhere)
    EOdomSource OdomSource = EOdomSource::WORLD;

    /**
     * @brief Update odom.
     * Noise is integral of gaussian noise.
     * @param InDeltaTime
     */
    virtual void UpdateOdom(float InDeltaTime);

    //! Publish tf or not
    //! @todo move this to publisher
    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdomTf = false;

    UPROPERTY(VisibleAnywhere)
    bool bIsOdomInitialized = false;

    //! Offset transform between the Owner Actor root component and the pose that will be published in /odom topic
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform RootOffset = FTransform::Identity;

protected:

    float LastUpdatedTime = 0.f;

    UPROPERTY()
    FTransform PreviousTransform = FTransform::Identity;

    UPROPERTY()
    FTransform PreviousNoisyTransform = FTransform::Identity;

    //! C++11 RNG for odometry noise
    std::random_device Rng;

    //! C++11 RNG for odometry noise
    std::mt19937 Gen = std::mt19937{Rng()};

    std::normal_distribution<> GaussianRNGPosition;
    std::normal_distribution<> GaussianRNGRotation;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseMeanPos = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseVariancePos = 0.01f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseMeanRot = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseVarianceRot = 0.05f;

    //! Add noise or not
    UPROPERTY(EditAnywhere, Category = "Noise")
    bool WithNoise = true;
};
