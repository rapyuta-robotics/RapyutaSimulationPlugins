/**
 * @file RRROS2BaseSensorPublisher.h
 * @brief Base Sensor Publisher class. Other sensor publisher class should inherit from this class.
 * Publishes data from #URRROS2BaseSensorComponent
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */


#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "RRROS2BaseSensorPublisher.generated.h"

class URRROS2BaseSensorComponent;

/**
 * @brief Base Sensor Publisher class. Other sensor publisher class should inherit from this class.
 * Publishes sensor data from #URRROS2BaseSensorComponent
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2BaseSensorPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    URRROS2BaseSensorPublisher();

    UPROPERTY()
    URRROS2BaseSensorComponent* DataSourceComponent = nullptr;

    virtual void UpdateMessage(UROS2GenericMsg* InMessage);
};
