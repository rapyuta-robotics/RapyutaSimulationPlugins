/**
 * @file RRRobotVehicleROSController.h
 * @brief Base ROS Controller class for Robot Vehicle
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "RRBaseRobotROSController.h"

#include "RRRobotVehicleROSController.generated.h"

class URRRobotROS2Interface;

/**
 * @brief Base ROS Controller class for Robot Vehicle
 * @todo temporary class for backward compatibility
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRRobotVehicleROSController : public ARRBaseRobotROSController
{
    GENERATED_BODY()
protected:
};
