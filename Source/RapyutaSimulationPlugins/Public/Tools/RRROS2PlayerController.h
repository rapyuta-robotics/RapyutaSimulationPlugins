// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"


// rclUE
#include "ROS2Node.h"

#include "Tools/RRROS2ClockPublisher.h"
#include "Tools/SimulationState.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"

#include "RRROS2PlayerController.generated.h"
UCLASS(config=Game)
class RAPYUTASIMULATIONPLUGINS_API ARRROS2PlayerController : public APlayerController
{
    GENERATED_BODY()

public:
    void Init(FString InName, ASimulationStateData* SimulationStateData);

    ARRROS2PlayerController();

    virtual void Tick(float DeltaSeconds) override;

    UFUNCTION(Server, Reliable)
    void WaitForPawnToPossess();

    UFUNCTION(Client, Reliable)
    void CheckClientTiming();

    UPROPERTY(BlueprintReadOnly, Replicated)
    AROS2Node* ROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    APawn* PossessedPawn = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    ASimulationStateData* SimulationStateData = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString PlayerName = "";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString Namespace = "";

//protected:
//    virtual void InitPlayer();

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

    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    virtual void AcknowledgePossession(APawn* InPawn) override;

    UFUNCTION(Client, Reliable)
    virtual void GetClientPawn(APawn* InPawn);

    UFUNCTION(Client, Reliable)
    void InitPawn(APawn* InPawn);

    UFUNCTION(BlueprintCallable, Client, Reliable)
    void SubscribeToMovementCommandTopic(const FString& InTopicName);

    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdom = true;

    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdomTf = false;

//    std::random_device Rng;
//    std::mt19937 Gen = std::mt19937{Rng()};
//    std::normal_distribution<> GaussianRNGOdom;
//
//    UPROPERTY(EditAnywhere, Category = "Noise")
//    float OdomNoiseMean = 0.f;
//
//    UPROPERTY(EditAnywhere, Category = "Noise")
//    float OdomNoiseVariance = 1.f;
//
//    UPROPERTY(EditAnywhere, Category = "Noise")
//    bool bWithNoise = true;
};
