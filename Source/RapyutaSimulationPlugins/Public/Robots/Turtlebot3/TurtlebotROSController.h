/**
 * @file TurtlebotROSController.h
 * @brief Example of child class of #ARRRobotVehicleROSController
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */


#pragma once

// RapyutaSimulationPlugins
#include "Robots/RRRobotVehicleROSController.h"

#include "TurtlebotROSController.generated.h"

/**
 * @brief Example of child class of #ARRRobotVehicleROSController
 * 
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotROSController : public ARRRobotVehicleROSController
{
    GENERATED_BODY()

public:
    ATurtlebotROSController(const FObjectInitializer& ObjectInitializer);

protected:
    /**
     * @brief Init publishers and set to publish odom as topic and tf.
     * 
     * @param InPawn 
     * @return true 
     * @return false 
     */
    bool InitPublishers(APawn* InPawn) override;
};
