/**
 * @file RRROS2LidarPublisher.h
 * @brief Publisher classes for 2D and 3D lidars
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2NodeComponent.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2LidarPublisher.generated.h"

/**
 * @brief Publisher class of 2D scan
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2LaserScanPublisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2LaserScanPublisher();
};

/**
 * @brief Publisher class of Point cloud such as 3D lidar scan
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2PointCloud2Publisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2PointCloud2Publisher();
};
