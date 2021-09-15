// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "Engine/EngineTypes.h"
#include "GameFramework/GameModeBase.h"

#include "RRGameMode.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    ARRGameMode();
    static constexpr int32 SIM_START_TIMEOUT_MINS = 2;
    static constexpr int32 SIM_START_TIMEOUT_SECS = 60 * SIM_START_TIMEOUT_MINS;

protected:
    virtual void StartPlay() override;

private:
    FTimerHandle OwnTimerHandle;

    UFUNCTION()
    void TryStartingSim();
};
