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
#include "Core/RRConversionUtils.h"

#include "RRGameState.generated.h"

class ARRGameMode;
class URRGameInstance;
class ARRPlayerController;
class ARRMeshActor;

/**
 * @brief Game state which handles multiple #URRSceneInstance which spit game in scenes for data gen, large world and etc.
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

    /**
     * @brief Create Scene Instances for #SCENE_INSTANCES_NUM by calling 
     * #CreateSceneInstance, #CreateServiceObjects, #StartSubSim, and #InitializeSim;
     * 
     */
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
    float SCENE_INSTANCES_DISTANCE_INTERVAL = 5000.f;
    UPROPERTY()
    int8 LastSceneInstanceId = URRActorCommon::DEFAULT_SCENE_INSTANCE_ID;

    UPROPERTY()
    TArray<URRSceneInstance*> SceneInstanceList;

    /**
     * @brief Get the Scene Instance object
     * 
     * @tparam TScenceInstanceClass 
     * @param InSceneInstanceId 
     * @return TScenceInstanceClass* 
     */
    template<typename TScenceInstanceClass>
    TScenceInstanceClass* GetSceneInstance(int8 InSceneInstanceId) const
    {
        TScenceInstanceClass* instance = SceneInstanceList.IsValidIndex(InSceneInstanceId)
                                             ? Cast<TScenceInstanceClass>(SceneInstanceList[InSceneInstanceId])
                                             : nullptr;
        ensure(instance);
        return instance;
    }

    /**
     * @brief Check SceneInstance with given Id is exists.
     * 
     * @param InSceneInstanceId 
     * @return true 
     * @return false 
     */
    bool HasSceneInstance(int8 InSceneInstanceId)
    {
        return SceneInstanceList.IsValidIndex(InSceneInstanceId) && SceneInstanceList[InSceneInstanceId];
    }

    virtual bool HasSceneInstanceListBeenCreated(bool bIsLogged = false) const;
    virtual bool HasInitialized(bool bIsLogged = false) const;

    virtual bool HaveAllSceneInstancesCompleted() const;

    // ENVIRONMENT
    UPROPERTY()
    AActor* MainEnvironment = nullptr;

    UPROPERTY()
    AActor* MainFloor = nullptr;

    UPROPERTY()
    AActor* MainWall = nullptr;

    UPROPERTY()
    TArray<ALight*> MainLights;

    // SIM OUTPUTS
    UPROPERTY(config)
    FString SIM_OUTPUTS_BASE_FOLDER_NAME = TEXT("OutputData");

    /**
     * @brief Get the Sim Outputs Base Folder Path object
     * To faciliate testing on CI, Outputs base folder need to be cleared during the test.
     * Thus, it would be clearer as using [ProjectSavedDir()] as the CI default output folder.
     * @return FString 
     */
    FString GetSimOutputsBaseFolderPath() const
    {
        return FPaths::IsRelative(SIM_OUTPUTS_BASE_FOLDER_NAME)
                   ? FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / SIM_OUTPUTS_BASE_FOLDER_NAME)
                   : SIM_OUTPUTS_BASE_FOLDER_NAME;
    }

    // ENTITIES
    /* The vertex normals
     * INI in ROS
      7 -- 4      Z
     /|   /|      |
    2-1--5 6      | Y
    |/   |/       |/
    0 -- 3         -----X

     * UE
    4 -- 7         Z
    |\   |\     Y  |
    6 5--1 2     \ |
     \|   \|      \|
      3 -- 0  X----
     */
    // NOTE: THESE ARE CONFIGURED BY USERS IN ROS RIGHT-HANDED COORD FOR CONVENIENCE, THUS THE INDEX ORDERING FOLLOWS CLOCK-WISE
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

    // UE LEFT-HANDED COUNTER-CLOCKWISE ORDERING
    FORCEINLINE FVector GetEntityBBVertexNormal(int8 InVertexIdx)
    {
        static const TArray<FVector> bbVertexNormals = {ENTITY_BOUNDING_BOX_VERTEX_NORMALS[3],
                                                        ENTITY_BOUNDING_BOX_VERTEX_NORMALS[6],
                                                        ENTITY_BOUNDING_BOX_VERTEX_NORMALS[5],
                                                        ENTITY_BOUNDING_BOX_VERTEX_NORMALS[0],
                                                        ENTITY_BOUNDING_BOX_VERTEX_NORMALS[7],
                                                        ENTITY_BOUNDING_BOX_VERTEX_NORMALS[2],
                                                        ENTITY_BOUNDING_BOX_VERTEX_NORMALS[1],
                                                        ENTITY_BOUNDING_BOX_VERTEX_NORMALS[4]};
        return bbVertexNormals[InVertexIdx];
    }

    void SetAllEntitiesActivated(bool bIsActivated);
    ARRMeshActor* FindEntityByModel(const FString& InEntityModelName, bool bToActivate, bool bToTakeAway);
    void AddEntity(ARRMeshActor* InEntity);

    template<typename T>
    void AddEntities(const TArray<T*> InEntityList)
    {
        for (const auto& entity : InEntityList)
        {
            AddEntity(entity);
        }
    }

    //! Move all env static actors to a scene instance
    virtual void MoveEnvironmentToSceneInstance(int8 InSceneInstanceId);

protected:
    /**
    * @brief Create a Scene Instance object.
    * Create SceneInstance and SceneDirector's own PlayerController, which actually creates its instance based on PlayerControllerClass
    * configured in [GameMode]'s ctor! ! [PlayerController] MUST BE CREATED EARLIER THAN ALL OTHER SIM SCENE'S ACTORS AND
    * OBJECTS
    * @param InSceneInstanceId 
    */
    virtual void CreateSceneInstance(int8 InSceneInstanceId);

    /**
     * @brief 
     * Spawn #ARRSceneDirector, which runs the main operation of the mode
     * This, due to making use of scene instance's actors, is spawned last.
     * ALSO, we should not write Scene-specific operation/functionality-related -starting codes inside scene instance's actors
     * (camera, actors, spawners) and leave it up to [SceneDirector] to decide the start sequence order!
     * @param InSceneInstanceId 
     */
    virtual void InitializeSim(int8 InSceneInstanceId);

    /**
     * @brief Trigger OnStartSim of URRSceneInstance::ActorCommon
     * 
     * @param InSceneInstanceId 
     */
    virtual void StartSubSim(int8 InSceneInstanceId);

    /**
     * @brief Create scene's common objects. 
     * Instantiate Actor Common, of which the ctor should not depend on any play resource.
     * Common objects in general must be created here first, then will be referenced anywhere else.
     * @param InSceneInstanceId 
     */
    virtual void CreateServiceObjects(int8 InSceneInstanceId);

    //! Stream level & Fetch static-env actors
    virtual void SetupEnvironment();

    //! Fetch env static actors (floors, walls, lights, etc.) if setup in the scene/level
    virtual void FetchEnvStaticActors();

    virtual void FinalizeSim();
    virtual void PrintSimConfig() const;

    /**
     * @brief Call #BeginSubPlay
     */
    virtual void BeginPlay() override;

    /**
     * @brief Trigger OnBeginPlay() for plugins' own common artifacts, which have been created earlier in this ::[StartSim()]
     * 
     */
    virtual void BeginSubPlay();

    /**
     * @brief This function works like an unwinding stack. where the children class's contents
     * are finalized ahead of parent's
     * => Reaching here means that all ~Common's OnEndPlay() have already finished!
     * 
     * @param EndPlayReason 
     */
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
    UPROPERTY()
    TSubclassOf<URRSceneInstance> SceneInstanceClass;

    //! Pool of all entities having been spawned
    UPROPERTY()
    TArray<ARRMeshActor*> AllDynamicMeshEntities;

private:
    //! To avoid early GC, this exists only to keep ones temporarily taken away from [AllDynamicMeshEntities] & recycled later
    UPROPERTY()
    TArray<AActor*> OrphanEntities;
};
