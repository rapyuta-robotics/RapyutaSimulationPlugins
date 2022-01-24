// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/ROS2BaseSensorPublisher.h"

#include "RRROS2LidarPublisher.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2LaserScanPublisher : public UROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2LaserScanPublisher();

    // UPROPERTY()
    // URRBaseLidarComponent* LidarComponent = nullptr;

    // void UpdateMessage(UROS2GenericMsg* InMessage) override;
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2PointCloud2Publisher : public UROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2PointCloud2Publisher();

    // UPROPERTY()
    // URRBaseLidarComponent* LidarComponent = nullptr;

    // void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
