/**
 * @file RRROS2ClockPublisher.h
 * @brief Clock publisher class. Get elapsed time by [UGameplayStatics::GetTimeSeconds](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Kismet/UGameplayStatics/GetTimeSeconds/)
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */


#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "RRROS2ClockPublisher.generated.h"

/**
 * @brief Clock publisher class. Get elapsed time by [UGameplayStatics::GetTimeSeconds](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Kismet/UGameplayStatics/GetTimeSeconds/)
 * @todo get publish frequency from project setting.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2ClockPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    void InitializeWithROS2(AROS2Node* InROS2Node) override;

    /**
     * @brief Update messsage with [UGameplayStatics::GetTimeSeconds](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Kismet/UGameplayStatics/GetTimeSeconds/)
     * 
     * @param InMessage 
     */
    void UpdateMessage(UROS2GenericMsg* InMessage) override;

        /**
     * @brief 
     * Called with every simulation step. Publishing clock msg with simulation step.
     * 
     * @param DeltaTime 
     * @param TickType 
     * @param ThisTickFunction 
     */
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

};
