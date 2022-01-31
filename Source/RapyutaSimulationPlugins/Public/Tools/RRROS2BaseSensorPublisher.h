// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "RRROS2BaseSensorPublisher.generated.h"

class URRROS2BaseSensorComponent;
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
