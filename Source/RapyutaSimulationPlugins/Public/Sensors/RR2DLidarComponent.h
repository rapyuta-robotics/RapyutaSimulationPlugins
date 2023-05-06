/**
 * @file RR2DLidarComponent.h
 * @brief ROS 2 2D lidar components.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/LineBatchComponent.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

// rclUE
#include "Msgs/ROS2LaserScan.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"
#include "Tools/RRROS2LidarPublisher.h"

// Others
#include <limits>

#include "RR2DLidarComponent.generated.h"

/**
 * @brief ROS 2 2D lidar components.
 * This class has 2 types of implementation, async and sync which can be switched by define TRACE_ASYNC.
 * sync uses LineTraceSingleByChannel and async uses AsyncLineTraceByChannel.
 *
 * @sa [LineTraceSingleByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/LineTraceSingleByChannel/)
 * @sa [AsyncLineTraceByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/AsyncLineTraceSingleByChannel/)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URR2DLidarComponent : public URRBaseLidarComponent
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URR2DLidarComponent object.
    *
    */
    URR2DLidarComponent();

    /**
     * @brief
     * async: Update #RecordedHits from #TraceHandles which is update in #SensorUpdate.
     *
     * @param DeltaTime
     * @param TickType
     * @param ThisTickFunction
     */
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief
     * sync:  Initialize #FHitResult
     * async: Initialize #FTraceHandle
     */
    void Run() override;

    /**
     * @brief Get Lidar data, add noise, draw lidar rays.
     * sync  : Uses LineTraceSingleByChannel to get lidar data.
     * async : Uses AsyncLineTraceByChannel to get lidar data.
     *
     * @sa [LineTraceSingleByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/LineTraceSingleByChannel/)
     * @sa [AsyncLineTraceByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/AsyncLineTraceSingleByChannel/)
     */
    void SensorUpdate() override;

    /**
     * @brief Return true if laser hits the target actor.
     * @param TargetActor
     * @return true
     * @return false
     */
    bool Visible(AActor* TargetActor) override;

    UFUNCTION(BlueprintCallable)
    /**
     * @brief Create ROS 2 Msg structure from #RecordedHits
     * This should probably be removed so that the sensor can be decoupled from the message types
     *
     * @return FROSLaserScan
     */
    FROSLaserScan GetROS2Data();

    /**
     * @brief Set result of #GetROS2Data to
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UFUNCTION(BlueprintCallable)
    float GetMinAngleRadians() const;

    UFUNCTION(BlueprintCallable)
    float GetMaxAngleRadians() const;
};
