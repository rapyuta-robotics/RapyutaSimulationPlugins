/**
 * @file SimulationState.h
 * @brief SimulationState class provides ROS2 service interfaces to interact with UE4.
 * Supported interactions: get/set actor state, spawn/delete actor, attach/detach actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "ROS2Node.h"
#include "Srvs/ROS2SpawnEntitySrv.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

#include "Srvs/ROS2AttachSrv.h"
#include "Srvs/ROS2DeleteEntitySrv.h"
#include "Srvs/ROS2GetEntityStateSrv.h"
#include "Srvs/ROS2SetEntityStateSrv.h"
#include "Srvs/ROS2SpawnEntitySrv.h"
#include "SimulationState.generated.h"

/**
 * @brief FActors has only TArray<AActor*> Actors.
 * This struct is used to create TMap<FName, FActors>.
 *
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FActors
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<AActor*> Actors;
};

class UROS2GenericSrv;

// (NOTE) To be renamed ARRROS2SimulationState, due to its inherent attachment to ROS2 Node
// & thus house [Entities] spawned by ROS services, and  with ROS relevance.
// However, check for its usage in BP and refactor if there is accordingly!
/**
 * @brief Provide ROS2 interfaces to interact with UE4.
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
     * @brief Callback function of GetEntityState ROS2 service.
     * Return the pose from reference frame.
     * @param Service
     * @sa [ue_mgs/GetEntityState.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/GetEntityState.srv)
     * @todo Twist is zero. Should return proper value for physics actors.
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitEntities();

    /**
     * @brief Callback function of SetEntityState ROS2 service.
     * @param Service
     * @sa [ue_mgs/SetEntityState.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SetEntityState.srv)
     * @todo Twist is zero. Should able to set value for physics actors.
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitROS2Node(AROS2Node* InROS2Node);

    /**
     * @brief Callback function of Attach ROS2 service.
     * Attach actors if those are not attached and detach actors if those are attached.
     * @param Service
     * @sa [ue_mgs/Attach.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/Attach.srv)
     */
    UFUNCTION(BlueprintCallable)
    void AttachSrv(UROS2GenericSrv* InService);

    UPROPERTY(BlueprintReadOnly)
    FROSGetEntityState_Request PreviousGetEntityStateRequest;

    //Set Entity Functions
     */
    UFUNCTION(BlueprintCallable)
    bool ServerSetEntityStateCheckRequest(FROSSetEntityState_Request Request);

    /**
     * @brief Callback function of DeleteEntity ROS2 service.
     * @param Service
     * @sa [ue_mgs/DeleteEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/DeleteEntity.srv)
     */
    UFUNCTION(BlueprintCallable)
    void ServerSetEntityState(FROSSetEntityState_Request Request);

    UPROPERTY(BlueprintReadOnly)
    FROSSetEntityState_Request PreviousSetEntityStateRequest;


    //Attach Functions
    UFUNCTION(BlueprintCallable)
    bool ServerAttachCheckRequest(FROSAttach_Request Request);

    UFUNCTION(BlueprintCallable)
    void ServerAttach(FROSAttach_Request Request);

    UPROPERTY(BlueprintReadOnly)
    FROSAttach_Request PreviousAttachRequest;

    //Spawn Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerSpawnCheckRequest(FROSSpawnEntityRequest Request);

    UFUNCTION(BlueprintCallable)
    void ServerSpawnEntity(FROSSpawnEntityRequest Request);

    UPROPERTY(BlueprintReadOnly)
    FROSSpawnEntityRequest PreviousSpawnRequest;

    //Delete Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerDeleteCheckRequest(FROSDeleteEntity_Request Request);

    UFUNCTION(BlueprintCallable)
    void ServerDeleteEntity(FROSDeleteEntity_Request Request);

    UPROPERTY(BlueprintReadOnly)
    FROSDeleteEntity_Request PreviousDeleteRequest;

    //Simulation State Manipulation Functions
    UFUNCTION(BlueprintCallable)

    UFUNCTION(BlueprintCallable)
    void OnRep_Entity();

    UFUNCTION(BlueprintCallable)
    void OnRep_SpawnableEntity();

    UFUNCTION(BlueprintCallable)
    void AddTaggedEntities(AActor* Entity, const FName& InTag);

    UFUNCTION(BlueprintCallable)
    /**
     * @brief Add Entities to #SpawnableEntities which can be spawn by SpawnEntity ROS2 service.
     * BP callable thus the param could not be const&
     * @param InSpawnableEntities
     */
    void AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities);

    UFUNCTION(BlueprintCallable)
    void GetSplitSpawnableEntities();

    template<typename T>
    bool CheckEntity(TMap<FString, T>& InEntities, const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckEntity(const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty = false);

    //! need node that will handle services - this class will only define and register the service
    UPROPERTY(BlueprintReadOnly)
    AROS2Node* ROSServiceNode = nullptr;

    //! Entities which can be manipulated by this class via ROS2 services.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    //! Entities which can be manipulated by this class via ROS2 services.
    UPROPERTY()
    TMap<FName, FActors> EntitiesWithTag;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_Entity)
    TArray<AActor*> EntityList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_SpawnableEntity)
    TArray<TSubclassOf<AActor>> SpawnableEntityList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_SpawnableEntity)
    TArray<FString> SpawnableEntityNameList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, TSubclassOf<AActor>> SpawnableEntities;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle TimerHandle;
     * @brief Spawn entity with tag & init nav surrogate
     * @param InROSSpawnRequest (FROSSpawnEntityRequest)
     * @param InEntityClass
     * @param InEntityTransform
     * @return AActor*
     */
    AActor* SpawnEntity(const FROSSpawnEntityRequest& InROSSpawnRequest,
                        const TSubclassOf<AActor>& InEntityClass,
                        const FTransform& InEntityTransform);
};
