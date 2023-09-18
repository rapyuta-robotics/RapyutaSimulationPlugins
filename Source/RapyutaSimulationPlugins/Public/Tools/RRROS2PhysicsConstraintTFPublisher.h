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
#include "Tools/RRROS2JointTFPublisher.h"

#include "RRROS2PhysicsConstraintTFPublisher.generated.h"

/**
 * @brief Publish TF of Actor relative to the reference actor.
 * If reference actor name is empty, publishes TF from world origin.
 * Provides ROS2Service to start/stop publishing tf.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2PhysicsConstraintTFPublisher : public URRROS2JointTFPublisher

{
    GENERATED_BODY()

protected:
    /**
     * @brief Initialize Transforms from #Constraint
     *
     * @param InROS2Node
     */
    virtual bool InitializeWithROS2(UROS2NodeComponent* InROS2Node) override;

    //! Physics Constraints
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Constraint = nullptr;

    FTransform InitialJointTF = FTransform::Identity;

    virtual void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
