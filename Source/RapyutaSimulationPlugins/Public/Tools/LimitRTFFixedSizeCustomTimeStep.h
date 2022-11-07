/**
 * @file LimitRTFFixedSizeCustomTimeStep.h
 * @brief CustomTimeStep class which uses fixed time step and and limit RTF(Real Time Factor)
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
#include "Engine/EngineCustomTimeStep.h"

#include "LimitRTFFixedSizeCustomTimeStep.generated.h"

class UEngine;

/**
 * @brief Control the Engine TimeStep via a fixed time step and limit RTF(Real Time Factor).
 * Main logic is copied from UGenlockedFixedRateCustomTimeStep and UEngineCustomTimeStep.
 * @sa [UEngineCustomTimeStep](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Engine/UEngineCustomTimeStep/)
 * @sa [UGenlockedFixedRateCustomTimeStep](https://docs.unrealengine.com/4.27/en-US/API/Runtime/TimeManagement/UGenlockedFixedRateCustomTimeSte-/)
 * 
 */
UCLASS(Blueprintable, editinlinenew, meta = (DisplayName = "Limit RTF Fixed Rate"))
class RAPYUTASIMULATIONPLUGINS_API URRLimitRTFFixedSizeCustomTimeStep : public UEngineCustomTimeStep
{
    GENERATED_UCLASS_BODY()

public:
    //~ UFixedFrameRateCustomTimeStep interface
    /**
     * @brief Overriden function from UEngineCustomTimeStep
     * @sa [UEngineCustomTimeStep](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Engine/UEngineCustomTimeStep/)
     * @param InEngine 
     * @return true 
     * @return false 
     */
    virtual bool Initialize(UEngine* InEngine) override;

    /**
     * @brief Overriden function from UEngineCustomTimeStep
     * @sa [UEngineCustomTimeStep](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Engine/UEngineCustomTimeStep/)
     * 
     * @param InEngine 
     */
    virtual void Shutdown(UEngine* InEngine) override;

    /**
     * @brief Overriden function from UEngineCustomTimeStep
     * @sa [UEngineCustomTimeStep](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Engine/UEngineCustomTimeStep/)
     * 
     * @param InEngine 
     * @return true 
     * @return false 
     */
    virtual bool UpdateTimeStep(UEngine* InEngine) override;

    /**
     * @brief Overriden function from UEngineCustomTimeStep
     * @sa [UEngineCustomTimeStep](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/Engine/UEngineCustomTimeStep/)
     * 
     * @return ECustomTimeStepSynchronizationState 
     */
    virtual ECustomTimeStepSynchronizationState GetSynchronizationState() const override;

    /**
     * @brief Get the Step Size object
     * 
     * @return float 
     */
    virtual float GetStepSize() const;

    /**
     * @brief Set the Step Size object
     * 
     * @param InStepSize 
     */
    virtual void SetStepSize(const float InStepSize);

    /**
     * @brief Get the Target R T F object
     * 
     * @return float 
     */
    virtual float GetTargetRTF() const;

    /**
     * @brief Set the Target R T F object
     * 
     * @param InTargetRTF 
     */
    virtual void SetTargetRTF(const float InTargetRTF);

    /**
     * @brief Main logic to update simulation time.
     * Simulation time += #StepSize and wait not to over #TargetRTF.
     * 
     * @return true 
     * @return false 
     */
    virtual bool WaitForSync();

public:
    /** Desired step size */
    UPROPERTY(EditAnywhere, Category = "Timing")
    float StepSize;

    /** Desired RTF(Real Time Factor).
     * No guarantee to meet this RTF but do not over this value
     */
    UPROPERTY(EditAnywhere, Category = "Timing")
    float TargetRTF = 1.f;

    UPROPERTY()
    double LastPlatformTime = 0;
};
