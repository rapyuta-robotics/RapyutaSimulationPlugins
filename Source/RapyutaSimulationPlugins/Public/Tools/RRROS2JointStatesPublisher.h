// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2GenericMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2StatePublisher.h"

#include "RRROS2JointStatesPublisher.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2JointStatesPublisher : public URRROS2StatePublisher
{
    GENERATED_BODY()

public:
    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
