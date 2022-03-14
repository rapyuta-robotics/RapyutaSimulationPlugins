// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "Engine/GameInstance.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGameMode.h"

#include "RRGameInstance.generated.h"

// This is a globally accessible instanced UObject that can store run-time data to be commonly accessed between levels and Scene
// instances. (Not to keep data persistent out of PIE)
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API URRGameInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FString InitialMapName;

    virtual void PreloadContentForURL(FURL InURL) override;
    virtual class AGameModeBase* CreateGameModeForURL(FURL InURL, UWorld* InWorld) override;

    virtual void StartGameInstance() override;
    virtual void Init() override;

    UFUNCTION()
    virtual void LoadComplete(const float InLoadTime, const FString& InMapName) override;

protected:
    void OnStart() override;
};
