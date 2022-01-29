// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2LidarPublisher.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2LaserScanPublisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2LaserScanPublisher();
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2PointCloud2Publisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2PointCloud2Publisher();
};
