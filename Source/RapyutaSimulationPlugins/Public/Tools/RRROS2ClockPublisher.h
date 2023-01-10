/**
 * @file RRROS2ClockPublisher.h
 * @brief Clock publisher class. Get elapsed time by [UGameplayStatics::GetTimeSeconds](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Kismet/UGameplayStatics/GetTimeSeconds/)
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Containers/Ticker.h"
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "RRROS2ClockPublisher.generated.h"

/**
 * @brief Clock publisher class. Get elapsed time by [UGameplayStatics::GetTimeSeconds](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Kismet/UGameplayStatics/GetTimeSeconds/)
 * @todo get publish frequency from project setting.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2ClockPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    URRROS2ClockPublisher();

    /**
     * @brief Initialize tickdelegate
     *
     */
    virtual bool Init() override;

protected:
    /** Delegate for callbacks to Tick */
    FTickerDelegate TickDelegate;

    /** Handle to various registered delegates */
    FTSTicker::FDelegateHandle TickDelegateHandle;

    /**
     * @brief
     * Called with every simulation step. Publishing clock msg with simulation step.
     *
     * @param DeltaSeconds
     */
    bool Tick(float DeltaSeconds);
};
