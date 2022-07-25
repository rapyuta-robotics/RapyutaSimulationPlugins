/**
 * @file RRNetworkPlayerController.h
 * @todo add doc
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
    void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    // ROS2
    UPROPERTY(Transient, Replicated)
    AROS2Node* ServerROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    URRROS2ClockPublisher* ServerClockPublisher = nullptr;
    void InitServerROS2();

    // SIM STATE & ROS2 STATE CLIENT
    //! Pointer to ServerSimState
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* ServerSimState = nullptr;
    //! ROS2 Sim state client
    UPROPERTY(Transient, Replicated)
    URRROS2SimulationStateClient* ROS2SimStateClient = nullptr;
    void CreateROS2SimStateClient(const TSubclassOf<URRROS2SimulationStateClient>& InSimStateClientClass);

    // POSSESSED PAWN
    //! Pawn that has been possessed by this controller
    UPROPERTY(Transient, Replicated)
    APawn* PossessedPawn = nullptr;

    UFUNCTION()
    void WaitToPossessPawn();

    UFUNCTION(Server, Reliable)
    void ServerPossessPawn(APawn* InPawn);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle PossessTimerHandle;

    //! Client - Init possessed robot's ROS2Interface & MoveComponent, etc.
    UFUNCTION(Client, Reliable)
    void ClientInitPawn(AActor* InActor);

    // PLAYER
    //! Controller's PlayerName
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString PlayerName;

    UFUNCTION(Server, Reliable)
    void ServerSetPlayerName(const FString& InPlayerName);

    // LOCAL CLOCK
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle ClockRequestTimerHandle;

    UPROPERTY()
    float ServerTime = 0.0f;
    virtual float GetServerTime()
    {
        return ServerTime;
    }
    void RequestServerTime();

    UFUNCTION(Client, Reliable)
    void ServerSendClock(float InClientRequestTime, float InServerCurrentTime);

    UFUNCTION(Server, Reliable)
    void ClientRequestClock(float InClientRequestTime);

    void UpdateLocalClock(float InDeltaSeconds);

protected:
    virtual void BeginPlay() override;
    virtual void ReceivedPlayer() override;
};
