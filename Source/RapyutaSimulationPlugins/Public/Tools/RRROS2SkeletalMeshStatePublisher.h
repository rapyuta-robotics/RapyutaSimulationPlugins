/**
 * @file RRROS2SkeletalMeshStatePublisher.h
 * @brief Publish pose of owner #ARobotVehicle which has skeletalmesh
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"
#include "Tools/RRROS2StatePublisher.h"

#include "RRROS2SkeletalMeshStatePublisher.generated.h"

/**
 * @brief Publish pose of owner #ARobotVehicle which has skeletalmesh
 * 
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRROS2SkeletalMeshStatePublisher : public URRROS2StatePublisher
{
    GENERATED_BODY()

public:
    void InitializeWithROS2(AROS2Node* InROS2Node) override;

    /**
     * @brief publish pose of owner #ARobotVehicle
     * @todo update to use conversion utils to convert UE to ROS coordinate.
     * @param InMessage 
     */
    void UpdateMessage(UROS2GenericMsg* InMessage) override;

    UPROPERTY(VisibleAnywhere)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    /**
     * @brief Set the Target Robot object. Set first USkeletalMeshComponent to #SkeletalMeshComp
     * 
     * @param InRobot 
     */
    void SetTargetRobot(ARobotVehicle* InRobot) override;
};
