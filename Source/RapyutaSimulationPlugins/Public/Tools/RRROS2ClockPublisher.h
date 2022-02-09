// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "RRROS2ClockPublisher.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2ClockPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
