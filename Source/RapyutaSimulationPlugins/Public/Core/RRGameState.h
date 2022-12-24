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
class ARRMeshActor;

/**
 * @brief Game state
 *
 * @sa [AGameState](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/AGameState/)
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

    UPROPERTY(config)
    int32 OPERATION_BATCHES_NUM = 5;

    UPROPERTY()
    ARRGameMode* GameMode = nullptr;

    UPROPERTY()
    URRGameInstance* GameInstance = nullptr;

    UPROPERTY(config)
    float SCENE_INSTANCES_DISTANCE_INTERVAL = 200.f;

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

    bool HasSceneInstance(int8 InSceneInstanceId)
    {
        return SceneInstanceList.IsValidIndex(InSceneInstanceId) && SceneInstanceList[InSceneInstanceId];
    }

    virtual bool HasSceneInstanceListBeenCreated(bool bIsLogged = false) const;
    virtual bool HasInitialized(bool bIsLogged = false) const;

    virtual bool HaveAllSceneInstancesCompleted() const;

    // SIM OUTPUTS
    UPROPERTY(config)
    FString SIM_OUTPUTS_BASE_FOLDER_NAME = TEXT("OutputData");

    // To faciliate testing on CI, Outputs base folder need to be cleared during the test.
    // Thus, it would be clearer as using [ProjectSavedDir()] as the CI default output folder.
    FString GetSimOutputsBaseFolderPath() const
    {
        return FPaths::IsRelative(SIM_OUTPUTS_BASE_FOLDER_NAME)
                   ? FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / SIM_OUTPUTS_BASE_FOLDER_NAME)
                   : SIM_OUTPUTS_BASE_FOLDER_NAME;
    }

    // ENTITIES
    /* The vertex normals
      7 -- 4      Z
     /|   /|      |
    2-1--5 6      | Y
    |/   |/       |/
    0 -- 3         -----X

    4 -- 7         Z
    |\   |\     Y  |
    6 5--1 2     \ |
     \|   \|      \|
      3 -- 0  X----
     */
    UPROPERTY(config)
    TArray<FVector> ENTITY_BOUNDING_BOX_VERTEX_NORMALS = {
        {0.f, 0.f, 0.f},    //[0]
        {0.f, 1.f, 0.f},    //[1]
        {0.f, 0.f, 1.f},    //[2]
        {1.f, 0.f, 0.f},    //[3]
        {1.f, 1.f, 1.f},    //[4]
        {1.f, 0.f, 1.f},    //[5]
        {1.f, 1.f, 0.f},    //[6]
        {0.f, 1.f, 1.f},    //[7]
    };
    ARRMeshActor* FindEntityByModel(const FString& InEntityModelName, bool bToActivate, bool bToTakeAway);
    void SetAllEntitiesActivated(bool bIsActivated);
    FORCEINLINE void AddEntity(ARRMeshActor* InEntity)
    {
        AllDynamicMeshEntities.AddUnique(InEntity);
    }

    template<typename T>
    void AddEntities(const TArray<T*> InEntityList)
    {
        for (const auto& entity : InEntityList)
        {
            AddEntity(entity);
        }
    }

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

    // Pool of all entities having been spawned
    UPROPERTY()
    TArray<ARRMeshActor*> AllDynamicMeshEntities;

private:
    // To avoid early GC, this exists only to keep ones temporarily taken away from [AllDynamicMeshEntities] & recycled later
    UPROPERTY()
    TArray<AActor*> OrphanEntities;
};
