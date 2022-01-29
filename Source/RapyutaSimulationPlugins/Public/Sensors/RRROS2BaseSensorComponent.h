// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"
#include "ROS2Publisher.h"
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2BaseSensorComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogROS2Sensor, Log, All);

#define TRACE_ASYNC 1

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2BaseSensorComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    URRROS2BaseSensorComponent();

    UFUNCTION(BlueprintCallable)
    virtual void InitalizeWithROS2(AROS2Node* InROS2Node, const FString& InPublisherName = TEXT(""), const FString& InTopicName = TEXT(""), const TEnumAsByte<UROS2QoS> InQoS = UROS2QoS::SensorData);

    UFUNCTION(BlueprintCallable)
    virtual void CreatePublisher(const FString& InPublisherName = TEXT(""));

    UFUNCTION(BlueprintCallable)
    virtual void PreInitializePublisher(AROS2Node* InROS2Node, const FString& InTopicName = TEXT(""));

    UFUNCTION(BlueprintCallable)
    virtual void InitializePublisher(AROS2Node* InROS2Node, const TEnumAsByte<UROS2QoS> InQoS = UROS2QoS::SensorData);

    UFUNCTION(BlueprintCallable)
    virtual void Run()
    {
        checkNoEntry();
    }
    
    UFUNCTION(BlueprintCallable)
	virtual void SetROS2Msg(UROS2GenericMsg* InMessage)
    {
        checkNoEntry();
    }

    UPROPERTY()
    TSubclassOf<UROS2Publisher> SensorPublisherClass;

    UPROPERTY(Transient)
    URRROS2BaseSensorPublisher* SensorPublisher = nullptr;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TopicName = TEXT("sensor_data");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("sensor_frame");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 PublicationFrequencyHz = 30;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bAppendNodeNamespace = true;

};
