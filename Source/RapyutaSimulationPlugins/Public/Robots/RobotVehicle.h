/**
 * @file RobotVehicle.h
 * @brief Base RobotVehicle class.
 * Example is #ATurtlebotBurger.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"

#include "Core/RRSkeletalMeshWrapper.h"
// RapyutaSimulationPlugins
#include "Robots/RRRobotBaseVehicle.h"

// rclUE
#include "ROS2Node.h"

#include "RobotVehicle.generated.h"

/**
 * @brief RobotVehicle class.
 * This class represents robot vehicles built up from a Skeletal Mesh component that is also its Root.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public ARRRobotBaseVehicle
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

    RRSkeletalMeshWrapper SkeletalMeshWrapper;

    /**
     * @brief Initialize #SkeletalMeshComp.
     *
     */
    void SetupDefaultRootSkeletal();

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const;
};
