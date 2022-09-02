/**
 * @file RRROS2SimulationStateClient.h
 * @brief RRROS2SimulationStateClient class provides ROS2 service interfaces to interact with UE4.
 * Supported interactions: get/set actor state, spawn/delete actor, attach/detach actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/ActorComponent.h"
#include "CoreMinimal.h"

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
class RAPYUTASIMULATIONPLUGINS_API URRROS2SimulationStateClient : public UActorComponent
{
    GENERATED_BODY()

public:
    //! ROS2 node of each client's own in the network
    UPROPERTY(BlueprintReadOnly, Replicated)
    AROS2Node* ROS2Node = nullptr;

    //! Handle to server's main sim state
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* ServerSimState = nullptr;

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
     * @brief RPC call to Server's SetEntityState
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerSetEntityState(const FROSSetEntityState_Request& InRequest);

    /**
     * @brief Callback function of Attach ROS2 service.
     * Attach actors if those are not attached and detach actors if those are attached.
     * @param Service
     * @sa [ue_mgs/Attach.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/Attach.srv)
     */
    UFUNCTION(BlueprintCallable)
    void AttachSrv(UROS2GenericSrv* InService);

    /**
     * @brief RPC call to Server's Attach
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerAttach(const FROSAttach_Request& InRequest);

    /**
     * @brief Callback function of SpawnEntity ROS2 service.
     * @param Service
     * @sa [ue_mgs/SpawnEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv)
     */
    UFUNCTION(BlueprintCallable)
    void SpawnEntitySrv(UROS2GenericSrv* InService);

    /**
     * @brief Callback function of SpawnEntities ROS2 service.
     * @param Service
     * @sa [ue_mgs/SpawnEntities.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntities.srv)
     */
    UFUNCTION(BlueprintCallable)
    void SpawnEntitiesSrv(UROS2GenericSrv* InService);

    /**
     * @brief RPC call to Server's SpawnEntity
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerSpawnEntity(const FROSSpawnEntityRequest& InRequest);

    /**
     * @brief Callback function of DeleteEntity ROS2 service.
     * @param Service
     * @sa [ue_mgs/DeleteEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/DeleteEntity.srv)
     */
    UFUNCTION(BlueprintCallable)
    void DeleteEntitySrv(UROS2GenericSrv* InService);

    /**
     * @brief RPC call to Server's DeleteEntity
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerDeleteEntity(const FROSDeleteEntity_Request& InRequest);

    /**
     * @brief RPC call to Server's AddEntity
     * @param InEntity
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerAddEntity(AActor* InEntity);

    /**
     * @brief Set Player Id
     * @param InNetworkPlayerId
     */
    UFUNCTION(BlueprintCallable)
    void SetNetworkPlayerId(const int32 InNetworkPlayerId)
    {
        NetworkPlayerId = InNetworkPlayerId;
    }

    /**
     * @brief Get Player Id
     * @param InEntity
     */
    UFUNCTION(BlueprintCallable)
    int32 GetNetworkPlayerId() const
    {
        return NetworkPlayerId;
    }

protected:
    virtual void OnComponentCreated() override;

    //! NetworkPlayerId which is used to differenciate client in server.
    UPROPERTY(BlueprintReadOnly, Replicated)
    int32 NetworkPlayerId;

private:
    template<typename T>
    bool CheckEntity(TMap<FString, T>& InEntities, const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckEntity(const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty = false);
};
