/**
 * @file RRBaseSensorComponent.h
 * @brief Base Sensor Component class. Other sensors class should inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/SceneComponent.h"

#include "RRBaseSensorComponent.generated.h"

#define TRACE_ASYNC 1

/**
 * @brief Base Sensor Component class. Other sensors class should inherit from this class.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRBaseSensorComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URRBaseSensorComponent object
    * 
    */
    URRBaseSensorComponent();

    /**
     * @brief Initialize sensor specifics & Start scanning
     */
    UFUNCTION(BlueprintCallable)
    virtual void Initialize();

    /**
     * @brief Update Sensor data. This method should be overwritten by child class.
     */
    UFUNCTION(BlueprintCallable)
    virtual void SensorUpdate()
    {
        checkNoEntry();
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 ScanFrequencyHz = 30;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool bLastScanDataValid = true;

protected:
    UPROPERTY()
    FTimerHandle TimerHandle;

    /**
     * @brief Start timer to update and publish sensor data by using SetTimer
     * @sa [SetTimer](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/FTimerManager/SetTimer/4/)
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void Run();
};
