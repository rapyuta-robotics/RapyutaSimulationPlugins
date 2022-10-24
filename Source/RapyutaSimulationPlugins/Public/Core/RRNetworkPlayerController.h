/**
 * @file RRNetworkPlayerController.h
 * @brief Network Player controller provides functionality for client-server.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */
#pragma once

// UE
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "UnrealClient.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Robots/RRBaseRobot.h"
#include "Tools/RRROS2ClockPublisher.h"
#include "Tools/RRROS2SimulationStateClient.h"
#include "Tools/SimulationState.h"

#include "RRNetworkPlayerController.generated.h"

class ARRGameState;

/**
 * @brief Network Player controller provides functionality for client-server. Major functionalites are
 * - [AROS2Node](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html),  #URRROS2ClockPublisher,  #URRROS2SimulationStateClient are created for each client to provide ROS2 services which are provided by #ARRROS2GameMode in standalone game.
 * - Clock sync between server and the clients with delay compensation
 * - RPC call to sync Robot movements between serer and clients.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRNetworkPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ARRNetworkPlayerController();

    virtual void Tick(float DeltaSeconds) override;

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    UPROPERTY()
    ARRGameState* GameState = nullptr;

    //! Sim ROS2 node in each client. Created only in the client.
    UPROPERTY(Transient)
    AROS2Node* SimStateClientROS2Node = nullptr;

    //! Publish /clock.
    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* SimStateClientClockPublisher = nullptr;

    // SIM STATE & ROS2 STATE CLIENT
    //! Pointer to ServerSimState. #ROS2SimStateClient in each client use this to execute sim-wide operation in the server.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* ServerSimState = nullptr;

    //! ROS2 Sim state client. Provide ROS2 interface of sim-wide operations implemented by #ServerSimState.
    UPROPERTY(Transient, Replicated, ReplicatedUsing = OnRep_SimStateClient)
    URRROS2SimulationStateClient* ROS2SimStateClient = nullptr;

    /**
     * @brief Function called with #SimStateClient replication. Initialize #ROS2SimStateClient.
     */
    UFUNCTION(BlueprintCallable)
    virtual void OnRep_SimStateClient();

    // PLAYER
    //! Controller's PlayerName.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString PlayerName;

    /**
     * @brief Create ROS2SimStateClient without initializing yet
     * 
     * @param InSimStateClientClass 
     */
    UFUNCTION(BlueprintCallable)
    void CreateROS2SimStateClient(const TSubclassOf<URRROS2SimulationStateClient>& InSimStateClientClass);

    /**
     * @brief Init SimStateClient ROS2 Component, e.g. #SimStateClientROS2Node, #SimStateClientClockPublisher.
     */
    UFUNCTION(Client, Reliable)
    void ClientInitSimStateClientROS2();

    /**
     * @brief Set player name in server
     */
    UFUNCTION(Server, Reliable)
    void ServerSetPlayerName(const FString& InPlayerName);

    // LOCAL CLOCK
    //! Timer handle used by RequestServerTime(). This will used to sync simulation time.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle ClockRequestTimerHandle;

    //! Local clock time
    UPROPERTY()
    float LocalTime = 0.0f;
    virtual float GetLocalTime()
    {
        return LocalTime;
    }

    /**
     * @brief Request server time update from client via #ServerRequestLocalClockUpdate
     */
    void RequestServerTimeUpdate();

    /**
     * @brief Called from the server and execute in the client via RPC. 
     * Sync client time with InServerCurrentTime + delay compendation.
     * Delay is estimated as 0.5 * (current client time - InClientRequestTime)
     * 
     * @param InClientRequestTime Client time when client calls #ServerRequestLocalClockUpdate
     * @param InServerCurrentTime Server time when server call this method
     */
    UFUNCTION(Client, Reliable)
    void ClientSendLocalClockUpdate(float InClientRequestTime, float InServerCurrentTime);

    /**
     * @brief Called from the client and execute in the server via RPC. 
     * Get current server time and call #ClientSendLocalClockUpdate with current server time and given InClientRequestTime
     * 
     * @param InClientRequestTime Client Time when client call this method.
     */
    UFUNCTION(Server, Reliable)
    void ServerRequestLocalClockUpdate(float InClientRequestTime);

    /**
     * @brief Increase local clock's time by delta seconds
     * @param InDeltaSeconds
     */
    void UpdateLocalClock(float InDeltaSeconds);

    /**
     * @brief Set server linear velocity to #RobotVehicleMoveComponent. Non player object can use this method to call RPC.
     * @param InServerRobot target server-owned robot
     * @param InClientTimeStamp
     * @param InClientRobotPosition
     * @param InLinearVel
     * @note This method is because RRRobotBaseVehicle can't use rpc since it is not controlled/possessed by the Player. 
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    virtual void ServerSetLinearVel(ARRBaseRobot* InServerRobot,
                                    float InClientTimeStamp,
                                    const FTransform& InClientRobotTransform,
                                    const FVector& InLinearVel);

    /**
     * @brief Set server angular velocity to #RobotVehicleMoveComponent. Non player object can use this method to call RPC.
     * @param InServerRobot target server-owned robot
     * @param InClientTimeStamp
     * @param InClientRobotRotation
     * @param InAngularVel
     * @note This method is because RRRobotBaseVehicle can't use rpc since it is not controlled/possessed by the Player. 
     */
    UFUNCTION(BlueprintCallable, Server, Reliable)
    virtual void ServerSetAngularVel(ARRBaseRobot* InServerRobot,
                                     float InClientTimeStamp,
                                     const FRotator& InClientRobotRotation,
                                     const FVector& InAngularVel);

protected:
    virtual void BeginPlay() override;
    virtual void ReceivedPlayer() override;
};
