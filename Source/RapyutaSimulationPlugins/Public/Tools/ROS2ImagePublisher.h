// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "ROS2ImagePublisher.generated.h"

class UROS2CameraComponent;
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UROS2ImagePublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    UROS2ImagePublisher();

    UPROPERTY()
    UROS2CameraComponent* DataSourceComponent = nullptr;

    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
