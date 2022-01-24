// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Tools/ROS2BaseSensorPublisher.h"

#include "ROS2ImagePublisher.generated.h"

class UROS2CameraComponent;
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UROS2ImagePublisher : public UROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    UROS2ImagePublisher();

    // void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
