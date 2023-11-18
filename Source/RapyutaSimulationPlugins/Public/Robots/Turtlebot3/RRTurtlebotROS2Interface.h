/**
 * @file RRTurtlebotROS2Interface.h
 * @brief Example of  #URRRobotROS2Interface's child class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// RapyutaSimulationPlugins
#include "Robots/RRRobotROS2Interface.h"

#include "RRTurtlebotROS2Interface.generated.h"

/**
 * @brief Example of #URRRobotROS2Interface's child class
 *
 */
UCLASS(ClassGroup = (Custom))
class RAPYUTASIMULATIONPLUGINS_API URRTurtlebotROS2Interface : public URRRobotROS2Interface
{
    GENERATED_BODY()

protected:
    /**
     * @brief Init turlebot's ros parameter and set to publish odom as topic and tf.
     *
     */
    void SetupROSParams() override
    {
        bPublishOdom = true;
        bPublishOdomTf = true;
        bPublishJointTf = true;
        JointTfPublicationFrequencyHz = 30;
    };
};
