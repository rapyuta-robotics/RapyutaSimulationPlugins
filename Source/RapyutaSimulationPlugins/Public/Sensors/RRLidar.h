/**
 * @file RRLidar.h
 * @brief Standalone lidar sensor actor.
 * @todo Follow the same imple as #ARRROS2CameraActor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "ROS2NodeComponent.h"
#include "ROS2Publisher.h"

#include "RRLidar.generated.h"

/**
 * @brief Standalone lidar sensor actor which can be placed in the level.
 * @todo Follow the same impl as #ARRROS2CameraActor.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRLidar : public AStaticMeshActor
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRBaseLidarComponent* LidarComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URRBaseLidarComponent> LidarComponentClass;

    virtual void PostInitializeComponents() override;
};
