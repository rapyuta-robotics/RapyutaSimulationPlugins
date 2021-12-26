// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "AIController.h"
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2LaserScanMsg.h"
#include "Msgs/ROS2OdometryMsg.h"
#include "Msgs/ROS2TFMsg.h"
#include "Msgs/ROS2TwistMsg.h"
#include "ROS2Node.h"
#include "ROS2Publisher.h"
#include "Tools/ROS2TFPublisher.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

#include "RRRobotVehicleROSController.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRRobotVehicleROSController : public AAIController
{
    GENERATED_BODY()

public:
    ARRRobotVehicleROSController(const FObjectInitializer& ObjectInitializer);

    UFUNCTION(BlueprintCallable)
    virtual bool GetOdomData(FROSOdometry& OutOdomData) const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 RobotID = 0;

    // total number of agents (== maxID)
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NAgents = 1;

protected:
    UPROPERTY(Transient)
    AROS2Node* VehicleROS2Node = nullptr;
    void InitVehicleROS2Node(APawn* InPawn;

    UPROPERTY()
    FVector InitialPosition = FVector::ZeroVector;

    UPROPERTY()
    FRotator InitialOrientation = FRotator::ZeroRotator;

    void InitPublishers(APawn* InPawn);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2TFPublisher* TFPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TFFrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TFChildFrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2Publisher* OdomPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString OdomTopicName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString CommandTopicName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URRBaseLidarComponent> LidarComponentClass;
    UPROPERTY(Transient, BlueprintReadWrite)
    FVector LidarOffset = FVector::ZeroVector;

    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    virtual void SetPawn(APawn* InPawn) override;

    UFUNCTION()
    void SubscribeToMovementCommandTopic(const FString& InTopicName);

    UFUNCTION()
    void OdomMessageUpdate(UROS2GenericMsg* TopicMessage);
};
