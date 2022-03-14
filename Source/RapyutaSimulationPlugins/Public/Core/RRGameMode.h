// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "Engine/EngineTypes.h"

// RapyutaSimulationPlugins
#include "Core/RRROS2GameMode.h"
#include "Core/RRTypeUtils.h"

#include "RRGameMode.generated.h"

class URRGameInstance;

UENUM()
enum class ERRSimType : uint8
{
    DEFAULT = 0x00,
    DATA_SYNTH = 0x01,
    ROBOT_SIM = 0x02
};

UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API ARRGameMode : public ARRROS2GameMode
{
    GENERATED_BODY()

public:
    ARRGameMode();
    static constexpr int32 SIM_START_TIMEOUT_MINS = 2;
    static constexpr int32 SIM_START_TIMEOUT_SECS = 60 * SIM_START_TIMEOUT_MINS;

    UPROPERTY(config)
    ERRSimType SimType = ERRSimType::ROBOT_SIM;

    UFUNCTION()
    FString GetSimTypeName() const
    {
        return URRTypeUtils::GetEnumValueAsString(TEXT("ERRSimType"), SimType);
    }

    UPROPERTY()
    URRGameInstance* GameInstance = nullptr;

    virtual void PreInitializeComponents() override;
    virtual void InitGameState() override;
    virtual void StartPlay() override;

    UFUNCTION()
    void PrintSimConfig() const;
    UFUNCTION()
    void PrintUEPreprocessors();

    virtual void ConfigureSimInPlay();
    void StartSim();

    UPROPERTY(Config)
    FIntPoint CaptureResolution = FIntPoint(1024, 1024);

    UPROPERTY(config)
    bool bBenchmark = true;

private:
    FTimerHandle OwnTimerHandle;

    UFUNCTION()
    bool TryStartingSim();
};
