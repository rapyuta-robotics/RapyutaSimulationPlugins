/**
 * @file RRROS2SimulationStateClient.h
 * @brief RRROS2SimulationStateClient class provides ROS 2 service interfaces to interact with UE4.
 * Supported interactions: get/set actor state, spawn/delete actor, attach/detach actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/ActorComponent.h"
#include "CoreMinimal.h"

// rclUE
#include "ROS2NodeComponent.h"
#include "Srvs/ROS2Attach.h"
#include "Srvs/ROS2DeleteEntity.h"
#include "Srvs/ROS2GetEntityState.h"
#include "Srvs/ROS2SetEntityState.h"
#include "Srvs/ROS2SpawnEntities.h"
#include "Srvs/ROS2SpawnEntity.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

#include "RRROS2SimulationStateClient.generated.h"

class UROS2GenericSrv;
class ASimulationState;

/**
 * @brief Provide ROS 2 interfaces to interact with UE4. This provide only ROS 2 interfaces and implementation is in #ASimulationState
 * Supported interactions: GetEntityState, SetEntityState, Attach, SpawnEntity, DeleteEntity
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRROS2SimulationStateClient : public UActorComponent
{
    GENERATED_BODY()

public:
    //! ROS 2 node of each client's own in the network
    UPROPERTY(BlueprintReadOnly, Replicated)
    UROS2NodeComponent* ROS2Node = nullptr;

    //! Handle to server's main sim state which has ROS 2 interface implementation
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* ServerSimState = nullptr;

    /**
     * @brief Start ROSservices.
     *
     * @param InROS2Node
     * @sa [ROS2Node](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html)
     */
    UFUNCTION(BlueprintCallable)
    virtual void Init(UROS2NodeComponent* InROS2Node);

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
     * @brief Callback function of GetEntityState ROS 2 service.
     * Return the pose from reference frame.
     * @param Service
     * @sa [ue_mgs/GetEntityState.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/GetEntityState.srv)
     * @todo Twist is zero. Should return proper value for physics actors.
     */
    UFUNCTION(BlueprintCallable)
    void GetEntityStateSrv(UROS2GenericSrv* InService);

    /**
     * @brief Callback function of SetEntityState ROS 2 service.
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
    void ServerSetEntityState(const FROSSetEntityStateReq& InRequest);

    /**
     * @brief Callback function of Attach ROS 2 service.
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
    void ServerAttach(const FROSAttachReq& InRequest);

    /**
     * @brief Callback function of SpawnEntity ROS 2 service.
     * @param Service
     * @sa [ue_mgs/SpawnEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv)
     */
    UFUNCTION(BlueprintCallable)
    virtual void SpawnEntitySrv(UROS2GenericSrv* InService);

    /**
     * @brief Callback function of SpawnEntities ROS 2 service.
     * @param Service
     * @sa [ue_mgs/SpawnEntities.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntities.srv)
     */
    UFUNCTION(BlueprintCallable)
    virtual void SpawnEntitiesSrv(UROS2GenericSrv* InService);

    /**
     * @brief RPC call to Server's SpawnEntity
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerSpawnEntity(const FROSSpawnEntityReq& InRequest);

    /**
     * @brief Callback function of DeleteEntity ROS 2 service.
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
    void ServerDeleteEntity(const FROSDeleteEntityReq& InRequest);

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

    template<typename T>
    bool CheckEntity(TMap<FString, T>& InEntities, const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckEntity(const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty = false);
    virtual FROSSpawnEntityRes SpawnEntityImpl(FROSSpawnEntityReq& InRequest);
};
