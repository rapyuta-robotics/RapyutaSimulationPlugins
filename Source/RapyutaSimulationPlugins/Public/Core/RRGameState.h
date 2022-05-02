/**
 * @file RRGameState.h
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "GameFramework/GameState.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"

#include "RRGameState.generated.h"

class ARRGameMode;
class URRGameInstance;
class ARRPlayerController;

/**
 * @brief Game state
 *
 * @sa [AGameState](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AGameState/)
 *
 * @todo add documentation
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API ARRGameState : public AGameState
{
    GENERATED_BODY()

public:
    ARRGameState();
    virtual void StartSim();

    UPROPERTY(config)
    int8 SCENE_INSTANCES_NUM = 1;

    UPROPERTY()
    ARRGameMode* GameMode = nullptr;

    UPROPERTY()
    URRGameInstance* GameInstance = nullptr;

    static constexpr const float SCENE_INSTANCES_DISTANCE_INTERVAL = 2000.f;
    UPROPERTY()
    TArray<URRSceneInstance*> SceneInstanceList;

    template<typename TScenceInstanceClass>
    TScenceInstanceClass* GetSceneInstance(int8 InSceneInstanceId) const
    {
        TScenceInstanceClass* instance = SceneInstanceList.IsValidIndex(InSceneInstanceId)
                                           ? Cast<TScenceInstanceClass>(SceneInstanceList[InSceneInstanceId])
                                           : nullptr;
        verify(instance);
        return instance;
    }

    UFUNCTION()
    bool HasSceneInstance(int8 InSceneInstanceId)
    {
        return SceneInstanceList.IsValidIndex(InSceneInstanceId) && SceneInstanceList[InSceneInstanceId];
    }

    UFUNCTION()
    virtual bool HasSceneInstanceListBeenCreated(bool bIsLogged = false) const;

    UFUNCTION()
    virtual bool HasInitialized(bool bIsLogged = false) const;

protected:
    virtual void CreateSceneInstance(int8 InSceneInstanceId);
    virtual void InitializeSim(int8 InSceneInstanceId);
    virtual void StartSubSim(int8 InSceneInstanceId);
    virtual void CreateServiceObjects(int8 InSceneInstanceId);

    virtual void FinalizeSim();
    virtual void PrintSimConfig() const;
    virtual void BeginPlay() override;
    virtual void BeginSubPlay();
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    virtual void Tick(float DeltaTime) override;
    virtual void OnTick(float DeltaTime);

protected:
    UPROPERTY()
    TSubclassOf<URRSceneInstance> SceneInstanceClass;
};
