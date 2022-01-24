// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "ROS2BaseSensorPublisher.generated.h"

class UROS2BaseSensorComponent;
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UROS2BaseSensorPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    UROS2BaseSensorPublisher();
    
    UPROPERTY()
    UROS2BaseSensorComponent* DataSourceComponent = nullptr;

    virtual void UpdateMessage(UROS2GenericMsg* InMessage);
};
