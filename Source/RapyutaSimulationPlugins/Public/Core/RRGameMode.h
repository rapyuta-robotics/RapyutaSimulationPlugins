/**
 * @file RRGameMode.h
 * @brief GameMode with specific setting and asset loading.
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Engine/EngineTypes.h"

// RapyutaSimulationPlugins
#include "Core/RRROS2GameMode.h"
#include "Core/RRTypeUtils.h"

#include "RRGameMode.generated.h"

class URRGameInstance;

/**
 * @brief todo
 * @todo add documentation
 */
UENUM()
enum class ERRSimType : uint8
{
    NONE = 0x00,
    DATA_SYNTH = 0x01,
    ROBOT_SIM = 0x02
};

/**
 * @brief GameMode with specific setting, asset loading and ROS2 interface via #ClockPublisher and #ASimulationState.
 * 
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API ARRGameMode : public ARRROS2GameMode
{
    GENERATED_BODY()

public:
    ARRGameMode();
    static constexpr int32 SIM_START_TIMEOUT_MINS = 2;
    static constexpr int32 SIM_START_TIMEOUT_SECS = 60 * SIM_START_TIMEOUT_MINS;

    UPROPERTY()
    ERRSimType SimType = ERRSimType::ROBOT_SIM;

    FString GetSimTypeName() const
    {
        return URRTypeUtils::GetEnumValueAsString(TEXT("ERRSimType"), SimType);
    }
    bool IsDataSynthSimType() const
    {
        return (static_cast<uint8>(ERRSimType::DATA_SYNTH) & static_cast<uint8>(SimType));
    }

    UPROPERTY()
    URRGameInstance* GameInstance = nullptr;

    virtual void PreInitializeComponents() override;
    virtual void InitGameState() override;

    /**
     * @brief Set benchmark, fixed time step via FApp if running without editor. 
     * Call #ConfigureSimInPlay and #StartSim
     * @sa [FApp](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Core/Misc/FApp/)
     */
    virtual void StartPlay() override;

    virtual void PrintSimConfig() const;
    virtual void PrintUEPreprocessors() const;

    virtual void ConfigureSimInPlay();

    /**
     * @brief 
     * 1. LOAD [ImageWrapperModule].  This must be loaded this early for possible external image-based texture loading at Sim initialization!
     * 2. LOAD SIM STATIC GLOBAL RESOURCES
     * 3. START SIM ONCE RESOURCES ARE LOADED
     *  The reason for this scheduled delegate is some essential operation, which facilitates sim startup activities like
     *  asynchronous resource loading, could only run after this [ARRGameState::BeginPlay()] ends!
     */ 
    void StartSim();

    UPROPERTY(config)
    bool bBenchmark = true;

private:
    FTimerHandle OwnTimerHandle;

    /**
     * @brief This method was scheduled to be run by BeginPlay()
     * 1. WAIT FOR RESOURCE LOADING, in prep for Sim mode initialization
     * 2. START PARENT'S PLAY, WHICH TRIGGER OTHERS PLAY FROM GAME STATE, PLAYER CONTROLLER, ETC.
     */
    UFUNCTION()
    bool TryStartingSim();
};
