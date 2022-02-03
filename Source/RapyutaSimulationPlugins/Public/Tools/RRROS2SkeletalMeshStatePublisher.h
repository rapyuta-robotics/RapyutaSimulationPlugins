// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"
#include "Tools/RRROS2StatePublisher.h"

#include "RRROS2SkeletalMeshStatePublisher.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRROS2SkeletalMeshStatePublisher : public URRROS2StatePublisher
{
    GENERATED_BODY()

public:
    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;

    UPROPERTY(VisibleAnywhere)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    void SetTargetRobot(ARobotVehicle* InRobot) override;
};
