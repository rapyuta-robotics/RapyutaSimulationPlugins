/**
 * @file SimulationState.h
 * @brief SimulationState class provides ROS2 interface implementation to interact with UE4.
 * Supported interactions: get/set actor state, spawn/delete actor, attach/detach actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
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
 * @brief FRREntityInfo
 * This struct is used to create #SpawnableEntityInfoList
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRREntityInfo
{
    GENERATED_BODY()

    UPROPERTY()
    FString EntityTypeName;

    UPROPERTY()
    TSubclassOf<AActor> EntityClass;

    FRREntityInfo()
    {
    }
    FRREntityInfo(const TPair<FString, TSubclassOf<AActor>>& InEntityInfo)
        : EntityTypeName(InEntityInfo.Key), EntityClass(InEntityInfo.Value)
    {
    }
};

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
 * @brief Provide ROS2 interface implementations to interact with UE4.
 * Supported interactions: Service [GetEntityState, SetEntityState, Attach, SpawnEntity, DeleteEntity]
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
     * @brief Fetch all entities in the current map
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitEntities();

    //! Cached the previous [GetEntityState] request for duplicated incoming request filtering
    UPROPERTY(BlueprintReadOnly)
    FROSGetEntityState_Request PrevGetEntityStateRequest;

    /**
     * @brief Check set-entity-state-request for duplication on server
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckSetEntityStateRequest(const FROSSetEntityState_Request& InRequest);

    /**
     * @brief Set Entity state on server
     */
    UFUNCTION(BlueprintCallable)
    void ServerSetEntityState(const FROSSetEntityState_Request& InRequest);

    //! Cached the previous [SetEntityState] request for duplicated incoming request filtering
    UPROPERTY(BlueprintReadOnly)
    FROSSetEntityState_Request PrevSetEntityStateRequest;

    /**
     * @brief Check entity-attach request for duplication on server
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckAttachRequest(const FROSAttach_Request& InRequest);

    /**
     * @brief Attach an entity to another on Server
     */
    UFUNCTION(BlueprintCallable)
    void ServerAttach(const FROSAttach_Request& Request);

    //! Cached the previous [Attach] request for duplicated incoming request filtering
    UPROPERTY(BlueprintReadOnly)
    FROSAttach_Request PrevAttachEntityRequest;

    /**
     * @brief Check entity-spawn-request for duplication on Server
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckSpawnRequest(const FROSSpawnEntityRequest& InRequest);

    /**
     * @brief Spawn entity on Server
     */
    UFUNCTION(BlueprintCallable)
    AActor* ServerSpawnEntity(const FROSSpawnEntityRequest& InRequest);

    //! Cached the previous [SpawnEntity] request for duplicated incoming request filtering
    UPROPERTY(BlueprintReadOnly)
    FROSSpawnEntityRequest PreviousSpawnEntityRequest;

    /**
     * @brief Check delete-entity-request for duplication on Server
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckDeleteRequest(const FROSDeleteEntity_Request& InRequest);

    /**
     * @brief Delete entity on Server
     */
    UFUNCTION(BlueprintCallable)
    void ServerDeleteEntity(const FROSDeleteEntity_Request& InRequest);

    //! Cached the previous [DeleteEntity] request for duplicated incoming request filtering
    UPROPERTY(BlueprintReadOnly)
    FROSDeleteEntity_Request PrevDeleteEntityRequest;

    UFUNCTION(BlueprintCallable)
    /**
     * @brief Add Entity to #Entities and #EntitiesWithTag
     * Entity become able to be manipulated by Simulationstate's ROS2 servs.
     * @param Entity
     */
    void AddEntity(AActor* InEntity);

    /**
     * @brief Add an entity with tag
     */
    UFUNCTION(BlueprintCallable)
    void AddTaggedEntity(AActor* InEntity, const FName& InTag);

    /**
     * @brief Add Entity Types to #SpawnableEntities which can be spawn by SpawnEntity ROS2 service.
     * BP callable thus the param could not be const&
     * @param InSpawnableEntityTypes
     */
    UFUNCTION(BlueprintCallable)
    void AddSpawnableEntityTypes(TMap<FString, TSubclassOf<AActor>> InSpawnableEntityTypes);

    //! All existing entities
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    //! Entities with tags which can be manipulated by this class via ROS2 services.
    UPROPERTY()
    TMap<FName, FRREntities> EntitiesWithTag;

    //! Replicatable copy of #Entities
    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_EntityList)
    TArray<AActor*> EntityList;
    /**
     * @brief Callback when #EntityList is replicated
     */
    UFUNCTION(BlueprintCallable)
    void OnRep_EntityList();

    //! Spawnable entity types for SpawnEntity ROS2 service.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, TSubclassOf<AActor>> SpawnableEntityTypes;

    //! Replicatable Copy of #SpawnableEntityTypes
    UPROPERTY(EditAnywhere, ReplicatedUsing = OnRep_SpawnableEntityInfoList)
    TArray<FRREntityInfo> SpawnableEntityInfoList;

    /**
     * @brief Fetch #SpawnableEntityInfoList
     */
    UFUNCTION(BlueprintCallable)
    void GetSpawnableEntityInfoList();

    /**
     * @brief Callback when #SpawnableEntityInfoList is replicated
     */
    UFUNCTION(BlueprintCallable)
    void OnRep_SpawnableEntityInfoList();

    //! Timer handle to fetch #SpawnableEntityInfoList
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle FetchEntityListTimerHandle;

private:
    /**
     * @brief Verify a function is called for server
     */
    bool VerifyIsServerCall(const FString& InFunctionName);

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
