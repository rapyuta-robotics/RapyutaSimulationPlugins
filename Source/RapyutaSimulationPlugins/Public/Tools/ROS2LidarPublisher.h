// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"
#include "ROS2Publisher.h"

#include "ROS2LidarPublisher.generated.h"

class URRBaseLidarComponent;
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UROS2LidarPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    UROS2LidarPublisher();

    UPROPERTY()
    URRBaseLidarComponent* LidarComponent = nullptr;

    void InitializeWithROS2(AROS2Node* InROS2Node) override;

    UFUNCTION()
    void UpdateMessage(UROS2GenericMsg* InMessage);
};
