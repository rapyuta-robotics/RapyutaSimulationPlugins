// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "AIController.h"
#include "CoreMinimal.h"

#include <Msgs/ROS2LaserScanMsg.h>
#include <Msgs/ROS2OdometryMsg.h>
#include <Msgs/ROS2TFMsg.h>
#include <Msgs/ROS2TwistMsg.h>
#include <ROS2Node.h>
#include <ROS2Publisher.h>
#include <Sensors/SensorLidar.h>
#include <Tools/ROS2TFPublisher.h>

#include "TurtlebotROSController.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotROSController : public AAIController
{
    GENERATED_BODY()

protected:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<ASensorLidar> LidarClass;

    UPROPERTY(Transient)
    AROS2Node* TurtleNode = nullptr;

    UPROPERTY(Transient, BlueprintReadWrite)
    FVector LidarOffset = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2TFPublisher* TFPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2Publisher* OdomPublisher = nullptr;

public:
    ATurtlebotROSController(const FObjectInitializer& ObjectInitializer);

    UFUNCTION(BlueprintCallable)
    virtual bool GetOdomData(FROSOdometry& OutOdomData) const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 RobotID = 0;

    // total number of agents (== maxID)
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NAgents = 1;

protected:
    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    virtual void SetPawn(APawn* InPawn) override;

    virtual void SetupCommandTopicSubscription(ARobotVehicle* InPawn);

    UFUNCTION()
    void OdomMessageUpdate(UROS2GenericMsg* TopicMessage);

protected:
    UPROPERTY()
    ARobotVehicle* Turtlebot = nullptr;

    UPROPERTY()
    FVector InitialPosition = FVector::ZeroVector;

    UPROPERTY()
    FRotator InitialOrientation = FRotator::ZeroRotator;
};
