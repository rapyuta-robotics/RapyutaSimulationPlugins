/**
 * @file SimulationState.h
 * @brief SimulationState class provides ROS 2 interface implementation to interact with UE4.
 * Supported interactions: get/set actor state, spawn/delete actor, attach/detach actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "Srvs/ROS2Attach.h"
#include "Srvs/ROS2DeleteEntity.h"
#include "Srvs/ROS2GetEntityState.h"
#include "Srvs/ROS2SetEntityState.h"
#include "Srvs/ROS2SpawnEntities.h"
#include "Srvs/ROS2SpawnEntity.h"

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
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRREntities
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    TArray<AActor*> Actors;
};

// (NOTE) To be renamed ARRROS2SimulationState, due to its inherent attachment to ROS 2 Node
// & thus house [Entities] spawned by ROS services, and  with ROS relevance.
// However, check for its usage in BP and refactor if there is accordingly!
/**
 * @brief Provide ROS 2 interface implementations to interact with UE4.
 * Supported interactions: Service [GetEntityState, SetEntityState, Attach, SpawnEntity, DeleteEntity]
 *
 * SimulationState can manipulate only actors in #Entities and #EntitiesWithTag. All actors in the world are added to #Entities and
 * #EntitiesWithTag with #InitEntities method and actors can be added to those list by #AddEntity method individually as well.
 *
 * SimulationState can spawn only actors in #SpawnableEntities which actors can be added to by #AddSpawnableEntityTypes.
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
     * @brief Register entity types from Blueprint class names, that are configured in #ARRROS2GameMode
     */
    UFUNCTION(BlueprintCallable)
    void RegisterSpawnableBPEntities(const TArray<FString>& InBPSpawnableClassNames);

    /**
     * @brief Fetch all entities in the current map under control of this Actor.
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitEntities();

    //! Cached the previous [GetEntityState] request for duplicated incoming request filtering
    //! @todo is this necessary?
    UPROPERTY(BlueprintReadOnly)
    FROSGetEntityStateReq PrevGetEntityStateRequest;

    /**
     * @brief Check set-entity-state-request for duplication on server
     * @todo is this necessary?
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckSetEntityStateRequest(const FROSSetEntityStateReq& InRequest);

    /**
     * @brief Set Entity state on server
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable)
    void ServerSetEntityState(const FROSSetEntityStateReq& InRequest);

    //! Cached the previous [SetEntityState] request for duplicated incoming request filtering
    //! @todo is this necessary?
    UPROPERTY(BlueprintReadOnly)
    FROSSetEntityStateReq PrevSetEntityStateRequest;

    /**
     * @brief Check entity-attach request for duplication on server
     * @param InRequest
     * @todo is this necessary?
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckAttachRequest(const FROSAttachReq& InRequest);

    /**
     * @brief Attach/detach an entity to/from another on Server
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable)
    void ServerAttach(const FROSAttachReq& InRequest);

    //! Cached the previous [Attach] request for duplicated incoming request filtering
    //! @todo is this necessary?
    UPROPERTY(BlueprintReadOnly)
    FROSAttachReq PrevAttachEntityRequest;

    /**
     * @brief Check entity-spawn-request for duplication on Server
     * @todo is this necessary?
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckSpawnRequest(const FROSSpawnEntityReq& InRequest);

    /**
     * @brief Spawn entity on Server
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable)
    AActor* ServerSpawnEntity(const FROSSpawnEntityReq& InRequest, const int32 NetworkPlayerId);

    //! Cached the previous [SpawnEntity] request for duplicated incoming request filtering
    //! @todo is this necessary?
    UPROPERTY(BlueprintReadOnly)
    FROSSpawnEntityReq PrevSpawnEntityRequest;

    /**
     * @brief Check delete-entity-request for duplication on Server
     * @todo is this necessary?
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable)
    bool ServerCheckDeleteRequest(const FROSDeleteEntityReq& InRequest);

    /**
     * @brief Delete entity on Server
     * @todo is this necessary?
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable)
    void ServerDeleteEntity(const FROSDeleteEntityReq& InRequest);

    //! Cached the previous [DeleteEntity] request for duplicated incoming request filtering
    //! @todo is this necessary?
    UPROPERTY(BlueprintReadOnly)
    FROSDeleteEntityReq PrevDeleteEntityRequest;

    /**
     * @brief Add Entity to #Entities and #EntitiesWithTag
     * Entity become able to be manipulated by Simulationstate's ROS 2 servs.
     * @param InEntity
     */
    UFUNCTION(BlueprintCallable)
    void ServerAddEntity(AActor* InEntity);

    /**
     * @brief Add an entity with tag
     */
    UFUNCTION(BlueprintCallable)
    void AddTaggedEntity(AActor* InEntity, const FName& InTag);

    /**
     * @brief Add Entity Types to #SpawnableEntities which can be spawn by SpawnEntity ROS 2 service.
     * BP callable thus the param could not be const&
     * @param InSpawnableEntityTypes
     */
    UFUNCTION(BlueprintCallable)
    void AddSpawnableEntityTypes(TMap<FString, TSubclassOf<AActor>> InSpawnableEntityTypes);

    //! All existing entities which can be manipulated by this class via ROS 2 services.
    //! @todo Converting to TArrays to be able to be replicated
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

#if WITH_EDITOR
    TMap<FString, AActor*> EntitiesWithDisplayName;
#endif

    //! All existing entities with tags which can be manipulated by this class via ROS 2 services.
    //! @todo Converting to TArrays to be able to be replicated
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FName, FRREntities> EntitiesWithTag;

    //! Replicatable copy of #Entities
    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_EntityList)
    TArray<AActor*> EntityList;

    /**
     * @brief Callback when #EntityList is replicated
     */
    UFUNCTION(BlueprintCallable)
    void OnRep_EntityList();

    //! Spawnable entity types for SpawnEntity ROS 2 service.
    //! @todo Converting to TArrays to be able to be replicated
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

    /**
     * @brief Matches UE strings to original char buffers, for ros messages
     * This can be needed for unicode encoded strings in ros message when ROS->UE->ROS conversion does not work well
     */
    TMap<FString, std::string> EncodedStrings;

private:
    /**
     * @brief Verify a function is called from server
     */
    bool VerifyIsServerCall(const FString& InFunctionName);

    /**
     * @brief Spawn entity with tag & init nav surrogate
     * @param InROSSpawnRequest (FROSSpawnEntityReq)
     * @param InEntityClass
     * @param InEntityTransform
     * @param InNetworkPlayerId
     * @return AActor*
     */
    AActor* ServerSpawnEntity(const FROSSpawnEntityReq& InROSSpawnRequest,
                              const TSubclassOf<AActor>& InEntityClass,
                              const FTransform& InEntityTransform,
                              const int32& InNetworkPlayerId);
};
