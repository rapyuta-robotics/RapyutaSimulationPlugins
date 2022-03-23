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
    void Init(FString InName);

    ARRROS2PlayerController();

    UPROPERTY(BlueprintReadOnly)
    AROS2Node* ROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    UPROPERTY(BlueprintReadOnly)
    ASimulationState* SimulationState = nullptr;

    UPROPERTY(BlueprintReadWrite)
    FString PlayerName = "";

    UPROPERTY(BlueprintReadWrite)
    FString Namespace = "";

//protected:
//    virtual void InitPlayer();

    UPROPERTY(Transient)
    AROS2Node* RobotROS2Node = nullptr;

//    UFUNCTION(Client, Reliable)
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
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    UFUNCTION(BlueprintCallable)
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
