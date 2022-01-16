// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"
#include "ROS2Publisher.h"

#include "RRROS2LidarPublisher.generated.h"

class URRBaseLidarComponent;
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2LidarPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    URRROS2LidarPublisher();

    UPROPERTY()
    URRBaseLidarComponent* LidarComponent = nullptr;

    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
