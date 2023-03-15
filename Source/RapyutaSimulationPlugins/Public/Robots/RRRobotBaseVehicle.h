/**
 * @file RRRobotBaseVehicle.h
 * @brief Base robot vehicle class, inherited by other robot vehicle classes.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
#include "RRRobotBaseVehicle.generated.h"

/**
 * @brief Base robot vehicle class, inherited by other robot vehicle classes.
 * @todo temporary class for backward compatibility of child BPs
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRRobotBaseVehicle : public ARRBaseRobot
{
    GENERATED_BODY()
};
