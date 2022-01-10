// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "AIController.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "ROS2Node.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"

#include "RRRobotVehicleROSController.generated.h"

// (NOTE) Each robot would have a ROS-AI Controller thus also a ROS2Node for its own
// https://answers.unrealengine.com/questions/871116/view.html
// https://answers.unrealengine.com/questions/239159/how-many-ai-controllers-should-i-have.html
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRRobotVehicleROSController : public AAIController
{
    GENERATED_BODY()

protected:
    UPROPERTY(Transient)
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
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    UFUNCTION(BlueprintCallable)
    void SubscribeToMovementCommandTopic(const FString& InTopicName);

public:
    UPROPERTY(BlueprintReadWrite)
    bool PublishOdom = true;

    UPROPERTY(BlueprintReadWrite)
    bool PublishOdomTf = false;
};
