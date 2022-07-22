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

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRNetworkPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ARRNetworkPlayerController();

    virtual void Tick(float DeltaSeconds) override;
    void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    UPROPERTY(Transient, Replicated)
    AROS2Node* ClientROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    UFUNCTION()
    void WaitForPawnPossess();

    UFUNCTION(Server, Reliable)
    void ServerPossessPawn(APawn* InPawn);

    UFUNCTION(Client, Reliable)
    void ClientInitRobotMoveComp(AActor* InActor);

    UFUNCTION(Client, Reliable)
    void ClientInitRobotROS2Interface(AActor* InActor);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* MainSimState = nullptr;
    UPROPERTY(Transient, Replicated)
    URRROS2SimulationStateClient* ROS2SimStateClient = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString PlayerName;

    UPROPERTY(Transient, Replicated)
    APawn* PossessedPawn = nullptr;

    // Services
    //    UFUNCTION(BlueprintCallable)
    //    void SpawnEntitySrv(UROS2GenericSrv* Service);
    //
    //    UFUNCTION(BlueprintCallable, Server, Reliable)
    //    void ServerSpawnEntity(FROSSpawnEntityRequest Request);

    virtual void BeginPlay() override;

    UFUNCTION(Server, Reliable)
    void ServerSetPlayerName(const FString& InPlayerName);

    //    UFUNCTION(BlueprintCallable, Client, Reliable)
    //    void ReplicateTag();

    //    UFUNCTION(Client, Reliable)
    //    void InitPawn(APawn* InPawn);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle PossessTimerHandle;

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

    void LocalClockUpdate(float InDeltaSeconds);

    virtual void ReceivedPlayer() override;
};
