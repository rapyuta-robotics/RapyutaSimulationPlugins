/**
 * @file RRROS2ActorTFPublisher.h
 * @brief Publish TF of Actor relative to the reference actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "Kismet/GameplayStatics.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2TFPublisher.h"

#include "RRROS2JointTFPublisher.generated.h"

/**
 * @brief Publish TF of Actor relative to the reference actor.
 * If reference actor name is empty, publishes TF from world origin.
 * Provides ROS2Service to start/stop publishing tf.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2JointTFPublisher : public URRROS2TFPublisher

{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform ParentLinkToJoint = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform JointToChildLink = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform JointTF = FTransform::Identity;

    virtual void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
