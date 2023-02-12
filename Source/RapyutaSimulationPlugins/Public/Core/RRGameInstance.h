/**
 * @file RRGameInstance.h
 * @brief GameInstance
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "Engine/GameInstance.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGameMode.h"

#include "RRGameInstance.generated.h"

/**
 * @brief
 * This is a globally accessible instanced UObject that can store run-time data to be commonly accessed between levels and Scene
 * instances(Not to keep data persistent out of PIE). 
 * This class methods call UGameInstance method with logs.
 * @sa [UGameInstance](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UGameInstance/)
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API URRGameInstance : public UGameInstance
{
    GENERATED_BODY()

public:
    //! Store the World name.
    static FString SMapName;

    virtual void PreloadContentForURL(FURL InURL) override;

    /**
     * @brief Save InWorld name to #SMapName and print logs.
     * 
     * @param InURL 
     * @param InWorld 
     * @return AGameModeBase* 
     */
    virtual class AGameModeBase* CreateGameModeForURL(FURL InURL, UWorld* InWorld) override;

    virtual void StartGameInstance() override;

    /**
     * @brief Initialize Random seed.
     * 
     */
    virtual void Init() override;

    UFUNCTION()
    virtual void LoadComplete(const float InLoadTime, const FString& InMapName) override;

protected:
    void OnStart() override;
};
