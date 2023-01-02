/**
 * @file RRROS2BaseSensorComponent.h
 * @brief Base ROS2 Sensor Component class. Other sensors class should inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2BaseSensorComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogROS2Sensor, Log, All);

#define TRACE_ASYNC 1

/**
 * @brief Base ROS2 Sensor Component class. Other sensors class should inherit from this class.
 * Provide features to initialize with [AROS2NodeActor](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html)
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
     * @brief Create and initialize publisher and start sensor update by calling
     * #CreatePublisher, #PreInitializePublisher, #InitializePublisher and #Run.
     *
     * @param InROS2Node ROS2Node which this publisher belongs to
     * @param InPublisherName Publisher component name
     * @param InTopicName Topic name
     * @param InQoS Topic QoS
     *
     * @sa [AROS2NodeActor](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html)
     * @sa [ROS2 QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitalizeWithROS2(UROS2Node* InROS2Node,
                                   const FString& InPublisherName = TEXT(""),
                                   const FString& InTopicName = TEXT(""),
                                   const TEnumAsByte<UROS2QoS> InQoS = UROS2QoS::SensorData);

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
    virtual void PreInitializePublisher(UROS2Node* InROS2Node, const FString& InTopicName = TEXT(""));

    /**
     * @brief Initialize Sensorpublisher by using [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)'s methods.
     *
     * @param InROS2Node ROS2Node which this publisher belongs to
     * @param InQoS

     * @sa [ROS2 QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitializePublisher(UROS2Node* InROS2Node, const TEnumAsByte<UROS2QoS> InQoS = UROS2QoS::SensorData);

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
        checkNoEntry();
    }

    /**
     * @brief Set sensor data to ROS2 msg. This method should be overwritten by child class.
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage)
    {
        checkNoEntry();
    }

    UPROPERTY()
    TSubclassOf<UROS2Publisher> SensorPublisherClass = URRROS2BaseSensorPublisher::StaticClass();

    UPROPERTY(Transient)
    URRROS2BaseSensorPublisher* SensorPublisher = nullptr;

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
