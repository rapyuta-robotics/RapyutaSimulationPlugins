/**
 * @file RR3DLidarComponent.h
 * @brief ROS 2 3D lidar components.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "Msgs/ROS2PointCloud2.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

#include "RR3DLidarComponent.generated.h"

/**
 * @brief ROS 2 3D lidar components.
 * This class has 2 types of implementation, async and sync which can be switched by define TRACE_ASYNC.
 * sync uses LineTraceSingleByChannel and async uses AsyncLineTraceByChannel.
 *
 * @sa [LineTraceSingleByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/LineTraceSingleByChannel/)
 * @sa [AsyncLineTraceByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/AsyncLineTraceSingleByChannel/)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URR3DLidarComponent : public URRBaseLidarComponent
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URR3DLidarComponent object
    *
    */
    URR3DLidarComponent();

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
    virtual void Run() override;

    /**
     * @brief Get Lidar data, add noise, draw lidar rays.
     * sync  : Uses LineTraceSingleByChannel to get lidar data.
     * async : Uses AsyncLineTraceByChannel to get lidar data.
     *
     * @sa [LineTraceSingleByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/LineTraceSingleByChannel/)
     * @sa [AsyncLineTraceByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/AsyncLineTraceSingleByChannel/)
     */
    virtual void SensorUpdate() override;

    /**
     * @brief Return true if laser hits the target actor.
     * @param TargetActor
     * @return true
     * @return false
     */
    virtual bool Visible(AActor* TargetActor) override;

    /**
     * @brief Create ROS 2 Msg structure from #RecordedHits
     * This should probably be removed so that the sensor can be decoupled from the message types
     * @return FROSPointCloud2
     */
    UFUNCTION(BlueprintCallable)
    virtual FROSPointCloud2 GetROS2Data();

    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    // vertical samples
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NChannelsPerScan = 32;

    /**
     * @brief Get the Total Scan object
     *
     * @return uint64 #NSamplesPerScan * #NChannelsPerScan
     */
    virtual uint64 GetTotalScan() const
    {
        return NSamplesPerScan * NChannelsPerScan;
    }

    //! [degrees]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartVerticalAngle = -45.f;

    //! [degrees]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float FOVVertical = 90.f;

    //! Data is organized or not.
    //! false: height=1, width=data size
    //! true:  height=#NChannelsPerScan, width=#NSamplesPerScan
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bOrganizedCloud = false;

    //! [degrees]
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float DVAngle = 0.f;
};
