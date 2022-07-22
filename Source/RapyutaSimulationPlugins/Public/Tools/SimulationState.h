/**
 * @file SimulationState.h
 * @brief SimulationState class provides ROS2 service & action implementation to interact with UE4.
 * Supported interactions: get/set actor state, spawn/delete actor, attach/detach actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "ROS2Node.h"
#include "Srvs/ROS2AttachSrv.h"
#include "Srvs/ROS2DeleteEntitySrv.h"
#include "Srvs/ROS2GetEntityStateSrv.h"
#include "Srvs/ROS2SetEntityStateSrv.h"
#include "Srvs/ROS2SpawnEntitiesSrv.h"
#include "Srvs/ROS2SpawnEntitySrv.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

#include "SimulationState.generated.h"

/**
 * @brief FRREntities has only TArray<AActor*> Actors.
 * This struct is used to create TMap<FName, FRREntities>.
 *
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRREntities
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<AActor*> Actors;
};

// (NOTE) To be renamed ARRROS2SimulationState, due to its inherent attachment to ROS2 Node
// & thus house [Entities] spawned by ROS services, and  with ROS relevance.
// However, check for its usage in BP and refactor if there is accordingly!
/**
 * @brief Provide ROS2 implementations of services, actions to interact with UE4.
 * Supported interactions: GetEntityState, SetEntityState, Attach, SpawnEntity, DeleteEntity
 *
 * SimulationState can manipulate only actors in #Entities and #EntitiesWithTag. All actors in the world are added to #Entities and
 * #EntitiesWithTag with #Init method and actors can be added to those list by #AddEntity method individually as well.
 *
 * SimulationState can spawn only actors in #SpawnableEntities which actors can be added to by #AddSpawnableEntities.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ASimulationState : public AActor
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ASimulationState object
     *
     */
    ASimulationState();

public:
    /**
     * @brief Start ROSservices.
     *
     * @param InROS2Node
     * @sa [ROS2Node](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html)
     */
    UFUNCTION(BlueprintCallable)
    virtual void Init(AROS2Node* InROS2Node);

    /**
     * @brief Fetch all entities in the current map
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitEntities();

    UPROPERTY(BlueprintReadOnly)
    FROSGetEntityState_Request PreviousGetEntityStateRequest;

    // Set Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerCheckSetEntityStateRequest(const FROSSetEntityState_Request& InRequest);

    UFUNCTION(BlueprintCallable)
    void ServerSetEntityState(const FROSSetEntityState_Request& InRequest);

    UPROPERTY(BlueprintReadOnly)
    FROSSetEntityState_Request PreviousSetEntityStateRequest;

    // Attach Functions
    UFUNCTION(BlueprintCallable)
    bool ServerCheckAttachRequest(const FROSAttach_Request& InRequest);

    UFUNCTION(BlueprintCallable)
    void ServerAttach(const FROSAttach_Request& Request);

    UPROPERTY(BlueprintReadOnly)
    FROSAttach_Request PreviousAttachRequest;

    // Spawn Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerCheckSpawnRequest(const FROSSpawnEntityRequest& InRequest);

    UFUNCTION(BlueprintCallable)
    AActor* ServerSpawnEntity(const FROSSpawnEntityRequest& InRequest);

    UPROPERTY(BlueprintReadOnly)
    FROSSpawnEntityRequest PreviousSpawnRequest;

    // Delete Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerCheckDeleteRequest(const FROSDeleteEntity_Request& InRequest);

    UFUNCTION(BlueprintCallable)
    void ServerDeleteEntity(const FROSDeleteEntity_Request& InRequest);

    UPROPERTY(BlueprintReadOnly)
    FROSDeleteEntity_Request PreviousDeleteRequest;

    UFUNCTION(BlueprintCallable)
    /**
     * @brief Add Entity to #Entities and #EntitiesWithTag
     * Entity become able to be manipulated by Simulationstate's ROS2 servs.
     * @param Entity
     */
    void AddEntity(AActor* InEntity);

    UFUNCTION(BlueprintCallable)
    void OnRep_Entity();

    UFUNCTION(BlueprintCallable)
    void OnRep_SpawnableEntity();

    UFUNCTION(BlueprintCallable)
    void AddTaggedEntities(AActor* InEntity, const FName& InTag);

    UFUNCTION(BlueprintCallable)
    /**
     * @brief Add Entities to #SpawnableEntities which can be spawn by SpawnEntity ROS2 service.
     * BP callable thus the param could not be const&
     * @param InSpawnableEntities
     */
    void AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities);

    UFUNCTION(BlueprintCallable)
    void GetSplitSpawnableEntities();

    //! Main ROS2 Node for NM_Standalone to register ROS2 services, actions
    UPROPERTY(BlueprintReadOnly)
    AROS2Node* MainROS2Node = nullptr;

    //! All existing entities
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    //! Entities which can be manipulated by this class via ROS2 services.
    UPROPERTY()
    TMap<FName, FRREntities> EntitiesWithTag;

    //! Entities which can be manipulated by this class via ROS2 services.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_Entity)
    TArray<AActor*> EntityList;

    //! Spawnable entities from SpawnEntity ROS2 service.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_SpawnableEntity)
    TArray<TSubclassOf<AActor>> SpawnableEntityList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_SpawnableEntity)
    TArray<FString> SpawnableEntityNameList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, TSubclassOf<AActor>> SpawnableEntities;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle TimerHandle;

private:
    /**
     * @brief Spawn entity with tag & init nav surrogate
     * @param InROSSpawnRequest (FROSSpawnEntityRequest)
     * @param InEntityClass
     * @param InEntityTransform
     * @return AActor*
     */
    AActor* ServerSpawnEntity(const FROSSpawnEntityRequest& InROSSpawnRequest,
                              const TSubclassOf<AActor>& InEntityClass,
                              const FTransform& InEntityTransform);
};
