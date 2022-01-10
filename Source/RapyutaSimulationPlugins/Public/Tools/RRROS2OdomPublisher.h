// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2OdometryMsg.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2TFPublisher.h"

#include "RRROS2OdomPublisher.generated.h"

class UROS2GenericMsg;
class ARobotVehicle;

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2OdomPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadWrite)
    TWeakObjectPtr<ARobotVehicle> RobotVehicle = nullptr;

    UPROPERTY(BlueprintReadWrite)
    URRROS2TFPublisher* TFPublisher = nullptr;
    void InitializeTFWithROS2(AROS2Node* InROS2Node);

    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    UFUNCTION(BlueprintCallable)
    void InitOdomPublisher(AROS2Node* InROS2Node)
    {
        InitializeWithROS2(InROS2Node);
    }

    void RevokeUpdateCallback() override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
    bool GetOdomData(FROSOdometry& OutOdomData) const;

    UPROPERTY(BlueprintReadWrite)
    bool PublishOdomTf = false;

    UPROPERTY(BlueprintReadWrite)
    bool AppendNodeNamespace = true;
};
