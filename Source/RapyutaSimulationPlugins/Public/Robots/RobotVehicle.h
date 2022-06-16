/**
 * @file RobotVehicle.h
 * @brief Base RobotVehicle class. Other robot class should inherit from this class. Example is #ATurtlebotBurger.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"
#include "Engine/TargetPoint.h"
#include "GameFramework/Pawn.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Drives/RRJointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Sensors/RRROS2BaseSensorComponent.h"
#include "Robots/RobotEmptyVehicle.h"

// rclUE
#include "ROS2Node.h"

#include "RobotVehicle.generated.h"

class URobotVehicleMovementComponent;

/**
 * @brief Base RobotVehicle class. Other robot class should inherit from this class.
 * This actor moves with #URobotVehicleMovementComponent.
 * This actor is possessed by #ARRRobotVehicleROSController to be control from ROS2.
 * You can find example at #ATurtlebotBurger.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public ARobotEmptyVehicle
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ARobotVehicle object
     *
     */
    ARobotVehicle();

    /**
     * @brief Construct a new ARobotVehicle object
     *
     * @param ObjectInitializer
     */
    ARobotVehicle(const FObjectInitializer& ObjectInitializer);

    //! Robot Mesh
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    /**
     * @brief Initialize #SkeletalMeshComp.
     *
     */
    void SetupRootSkeletal();

};
