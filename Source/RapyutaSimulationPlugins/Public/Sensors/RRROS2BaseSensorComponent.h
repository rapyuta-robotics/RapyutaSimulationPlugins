/**
 * @file RRROS2BaseSensorComponent.h
 * @brief Base ROS 2 Sensor Component class. Other sensors class should inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GenericPlatform/GenericPlatformMath.h"

// rclUE
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2BaseSensorComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogROS2Sensor, Log, All);

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRGaussianNoise : public UObject
{
    GENERATED_BODY()

public:
    URRGaussianNoise()
    {
        Init();
    }

    URRGaussianNoise(const float InMean, const float InCov)
    {
        Init(InMean, InCov);
    }

    UFUNCTION(BlueprintCallable)
    virtual void Init()
    {
        GaussianRNG = std::normal_distribution<>(Mean, std::sqrt(Cov));
    }

    virtual void Init(const float InMean, const float InCov)
    {
        Mean = InMean;
        Cov = InCov;
        Init();
    }

    UFUNCTION(BlueprintCallable)
    virtual float Get()
    {
        return GaussianRNG(Gen);
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    float Mean = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Noise")
    float Cov = 0.01f;

protected:
    std::random_device Rng;

    std::mt19937 Gen = std::mt19937(Rng());

    std::normal_distribution<> GaussianRNG;
};

/**
 * @brief Base ROS 2 Sensor Component class. Other sensors class should inherit from this class.
 * Provide features to initialize with [UROS2NodeComponent](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d1/d79/_r_o_s2_node_component_8h.html)
 * and initialize #URRROS2BaseSensorPublisher.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2BaseSensorComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URRROS2BaseSensorComponent object
    *
    */
    URRROS2BaseSensorComponent();

    /**
     * @brief Destroy after stop timers and destroy publisher
     *
     * @param bPromoteChildren
     */
    virtual void DestroyComponent(bool bPromoteChildren = false) override;

    /**
     * @brief Create and initialize publisher and start sensor update by calling
     * #CreatePublisher, #PreInitializePublisher, #InitializePublisher and #Run.
     *
     * @param InROS2Node ROS2Node which this publisher belongs to
     * @param InPublisherName Publisher component name
     * @param InTopicName Topic name
     * @param InQoS Topic QoS
     *
     * @sa [UROS2NodeComponent](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d1/d79/_r_o_s2_node_component_8h.html)
     * @sa [ROS 2 QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitalizeWithROS2(UROS2NodeComponent* InROS2Node,
                                   const FString& InPublisherName = TEXT(""),
                                   const FString& InTopicName = TEXT(""),
                                   const UROS2QoS InQoS = UROS2QoS::SensorData);

    /**
     * @brief Create a Publisher with #SensorPublisherClass.
     *
     * @param InPublisherName If this is empty, publisher name become this component name + SensorPublisher.
     */
    UFUNCTION(BlueprintCallable)
    virtual void CreatePublisher(const FString& InPublisherName = TEXT(""));

    /**
     * @brief Set publish frequency, topic name and #FrameId.
     * if #bAppendNodeNamespace == true, FrameId become ROS2Node's namespace + #FramId
     * @param InROS2Node  ROS2Node which this publisher belongs to
     * @param InTopicName If this is empty, topic name become #TopicName.
     */
    UFUNCTION(BlueprintCallable)
    virtual void PreInitializePublisher(UROS2NodeComponent* InROS2Node, const FString& InTopicName = TEXT(""));

    /**
     * @brief Initialize Sensorpublisher by using [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)'s methods.
     *
     * @param InROS2Node ROS2Node which this publisher belongs to
     * @param InQoS

     * @sa [ROS 2 QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitializePublisher(UROS2NodeComponent* InROS2Node, const UROS2QoS InQoS = UROS2QoS::SensorData);

    /**
     * @brief Start timer to update and publish sensor data by using SetTimer
     * @sa [SetTimer](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/FTimerManager/SetTimer/4/)
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void Run();

    /**
     * @brief Stop timer to update and publish sensor data by using ClearTimer
     * @sa [ClearTimer](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/FTimerManager/ClearTimer)
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void Stop();

    /**
     * @brief Update Sensor data. This method should be overwritten by child class.
     */
    UFUNCTION(BlueprintCallable)
    virtual void SensorUpdate()
    {
    }

    /**
     * @brief Set sensor data to ROS 2 msg. This method should be overwritten by child class.
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage)
    {
    }

    UPROPERTY()
    TSubclassOf<UROS2Publisher> SensorPublisherClass = URRROS2BaseSensorPublisher::StaticClass();

    UPROPERTY(BlueprintReadWrite)
    URRROS2BaseSensorPublisher* SensorPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<UROS2GenericMsg> MsgClass = UROS2GenericMsg::StaticClass();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TopicName = TEXT("sensor_data");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("sensor_frame");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 PublicationFrequencyHz = 30;

    //! Append namespace to #FrameId or not.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bAppendNodeNamespace = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool bIsValid = true;

protected:
    UPROPERTY()
    FTimerHandle TimerHandle;
};
