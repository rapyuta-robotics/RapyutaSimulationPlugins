// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#pragma once
// UE
#include "GameFramework/PlayerController.h"
#include "UnrealClient.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Tools/SimulationStateData.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"
#include "Tools/RRROS2ClockPublisher.h"

#include "RRNetworkPlayerController.generated.h"

class ARRGameMode;
class ARRGameState;
class URRGameInstance;
class ARRActorCommon;

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRNetworkPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ARRNetworkPlayerController();

    virtual void Tick(float DeltaSeconds) override;

    UFUNCTION()
    void WaitForPawnToPossess();

    UFUNCTION(Server, Reliable)
    void ServerPossessPawn(AActor* InActor);

    UPROPERTY(BlueprintReadOnly, Replicated)
    AROS2Node* ROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    APawn* PossessedPawn = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationStateData* SimulationStateData = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString PlayerName = "";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString Namespace = "";

    UPROPERTY(Transient, Replicated)
    AROS2Node* RobotROS2Node = nullptr;

    void InitRobotROS2Node(APawn* InPawn);

    UPROPERTY()
    FVector InitialPosition = FVector::ZeroVector;

    UPROPERTY()
    FRotator InitialOrientation = FRotator::ZeroRotator;

    UPROPERTY(Transient, BlueprintReadWrite)
    URRROS2OdomPublisher* OdomPublisher = nullptr;
    UFUNCTION()
    virtual bool InitPublishers(APawn* InPawn);

    UFUNCTION()
    void InitRobotPublisher(APawn* InPawn);

    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    virtual void BeginPlay() override;

    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    virtual void AcknowledgePossession(APawn* InPawn) override;

    UFUNCTION(Server, Reliable)
    void SetServerPlayerName(const FString& InPlayerName);

//    UFUNCTION(Client, Reliable)
//    void InitPawn(APawn* InPawn);

    UFUNCTION(BlueprintCallable, Client, Reliable)
    void SubscribeToMovementCommandTopic(const FString& InTopicName);

    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdom = true;

    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdomTf = false;
};
