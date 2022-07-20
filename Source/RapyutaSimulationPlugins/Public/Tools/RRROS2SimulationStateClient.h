/**
 * @file RRROS2SimulationStateClient.h
 * @brief RRROS2SimulationStateClient class provides ROS2 service interfaces to interact with UE4.
 * Supported interactions: get/set actor state, spawn/delete actor, attach/detach actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

#include "RRROS2SimulationStateClient.generated.h"

class UROS2GenericSrv;
class ASimulationState;

/**
 * @brief Provide ROS2 interfaces to interact with UE4.
 * Supported interactions: GetEntityState, SetEntityState, Attach, SpawnEntity, DeleteEntity
 *
 * URRROS2SimulationStateClient can manipulate only actors in #Entities and #EntitiesWithTag. All actors in the world are added to
 * #Entities and #EntitiesWithTag with #Init method and actors can be added to those list by #AddEntity method individually as well.
 *
 * URRROS2SimulationStateClient can spawn only actors in #SpawnableEntities which actors can be added to by #AddSpawnableEntities.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRROS2SimulationStateClient : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadOnly, Replicated)
    AROS2Node* ClientROS2Node = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* SimulationState = nullptr;
    void InitSimulationState();

    /**
     * @brief Start ROSservices.
     *
     * @param InROS2Node
     * @sa [ROS2Node](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html)
     */
    UFUNCTION(BlueprintCallable)
    virtual void Init(AROS2Node* InROS2Node);

    virtual bool IsSupportedForNetworking() const override
    {
        return true;
    }

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    /**
     * @brief Callback function of GetEntityState ROS2 service.
     * Return the pose from reference frame.
     * @param Service
     * @sa [ue_mgs/GetEntityState.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/GetEntityState.srv)
     * @todo Twist is zero. Should return proper value for physics actors.
     */
    UFUNCTION(BlueprintCallable)
    void GetEntityStateSrv(UROS2GenericSrv* InService);

    /**
     * @brief Callback function of SetEntityState ROS2 service.
     * @param Service
     * @sa [ue_mgs/SetEntityState.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SetEntityState.srv)
     * @todo Twist is zero. Should able to set value for physics actors.
     */
    UFUNCTION(BlueprintCallable)
    void SetEntityStateSrv(UROS2GenericSrv* InService);

    /**
     * @brief Callback function of Attach ROS2 service.
     * Attach actors if those are not attached and detach actors if those are attached.
     * @param Service
     * @sa [ue_mgs/Attach.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/Attach.srv)
     */
    UFUNCTION(BlueprintCallable)
    void AttachSrv(UROS2GenericSrv* InService);

    /**
     * @brief Callback function of SpawnEntity ROS2 service.
     * @param Service
     * @sa [ue_mgs/SpawnEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv)
     */
    UFUNCTION(BlueprintCallable)
    void SpawnEntitySrv(UROS2GenericSrv* InService);

    /**
     * @brief Callback function of DeleteEntity ROS2 service.
     * @param Service
     * @sa [ue_mgs/DeleteEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/DeleteEntity.srv)
     */
    UFUNCTION(BlueprintCallable)
    void SpawnEntitiesSrv(UROS2GenericSrv* InService);

    UFUNCTION(BlueprintCallable)
    void DeleteEntitySrv(UROS2GenericSrv* InService);

#if 0
    // need node that will handle services - this class will only define and register the service
    UPROPERTY(BlueprintReadOnly)
    UROS2SpawnEntitySrv* SpawnEntityService = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    FROSSpawnEntityResponse SpawnResponse;
#endif

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle TimerHandle;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle TimerHandleResponder;

private:
    template<typename T>
    bool CheckEntity(TMap<FString, T>& InEntities, const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckEntity(const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty = false);
};
