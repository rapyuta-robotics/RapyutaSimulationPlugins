/**
 * @file RRNetworkPlayerController.h
 * @brief AMR Network Player controller to possess ARRBaseRobot in a client-server application
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

/**
 * @brief Network Player controller with its own {ROS2 Node + Clock publisher + ROS2 Sim state client} created for each possessed
 * robot
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

    // ROS2
    UPROPERTY(Transient)
    AROS2Node* SimStateClientROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* SimStateClientClockPublisher = nullptr;

    // SIM STATE & ROS2 STATE CLIENT
    //! Pointer to ServerSimState
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* ServerSimState = nullptr;

    //! ROS2 Sim state client
    UPROPERTY(Transient, Replicated, ReplicatedUsing = OnRep_SimStateClient)
    URRROS2SimulationStateClient* ROS2SimStateClient = nullptr;

    UFUNCTION(BlueprintCallable)
    virtual void OnRep_SimStateClient();

    // PLAYER
    //! Controller's PlayerName, taken from [robotname] param in Engine build.
    //! In Editor build, it will be automatically set from #PossessedPawn's name upon possession
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString PlayerName;

    /**
     * @brief Create ROS2SimStateClient without initializing yet
     */
    UFUNCTION(BlueprintCallable)
    void CreateROS2SimStateClient(const TSubclassOf<URRROS2SimulationStateClient>& InSimStateClientClass);

    /**
     * @brief Init SimStateClient's ROS2 (Node, ClockPublisher, etc.)
     */
    UFUNCTION(Client, Reliable)
    void ClientInitSimStateClientROS2();

    /**
     * @brief Possess a pawn on server
     */
    UFUNCTION(Server, Reliable)
    void ServerSetPlayerName(const FString& InPlayerName);

    // LOCAL CLOCK
    //! Timer handle used by RequestServerTime()
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
     * @brief Request server time update
     */
    void RequestServerTimeUpdate();

    /**
     * @brief TODO
     */
    UFUNCTION(Client, Reliable)
    void ServerSendLocalClockUpdate(float InClientRequestTime, float InServerCurrentTime);

    /**
     * @brief TODO
     */
    UFUNCTION(Server, Reliable)
    void ClientRequestLocalClockUpdate(float InClientRequestTime);

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
