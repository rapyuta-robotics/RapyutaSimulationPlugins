// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2OdometryMsg.h"
#include "ROS2Publisher.h"

#include "RRROS2OdomPublisher.generated.h"

class UROS2GenericMsg;
class ARobotVehicle;
class URRROS2TFPublisher;

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2OdomPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadWrite)
    TWeakObjectPtr<ARobotVehicle> RobotVehicle = nullptr;

    UPROPERTY(BlueprintReadWrite)
    TWeakObjectPtr<URRROS2TFPublisher> TFPublisher = nullptr;

    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    UFUNCTION(BlueprintCallable)
    void InitOdomPublisher(AROS2Node* InROS2Node)
    {
        InitializeWithROS2(InROS2Node);
    }
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
    bool GetOdomData(FROSOdometry& OutOdomData) const;
};
