// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "AIController.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

#include "RRRobotVehicleROSController.generated.h"

class AROS2Node;
class URRROS2TFPublisher;
class URRROS2OdomPublisher;
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRRobotVehicleROSController : public AAIController
{
    GENERATED_BODY()

public:
    ARRRobotVehicleROSController(const FObjectInitializer& ObjectInitializer);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 RobotID = 0;

    // total number of agents (== maxID)
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NAgents = 1;

protected:
    UPROPERTY(Transient)
    AROS2Node* VehicleROS2Node = nullptr;
    UFUNCTION()
    void InitVehicleROS2Node(APawn* InPawn);

    UPROPERTY()
    FVector InitialPosition = FVector::ZeroVector;

    UPROPERTY()
    FRotator InitialOrientation = FRotator::ZeroRotator;

    UPROPERTY(Transient)
    URRROS2TFPublisher* TFPublisher = nullptr;
    UPROPERTY(Transient)
    URRROS2OdomPublisher* OdomPublisher = nullptr;
    UFUNCTION()
    virtual bool InitPublishers(APawn* InPawn);

    UFUNCTION()
    virtual bool InitSensors(APawn* InPawn);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URRBaseLidarComponent> LidarComponentClass;

    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    UFUNCTION()
    void SubscribeToMovementCommandTopic(const FString& InTopicName);
};
