/**
 * @file RRCrowdROSController.h
 * @brief Base Crowd ROS Controller for ARRBaseRobot descedent classes, utilizing URRCrowdFollowingComponent
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Core/RRCrowdAIController.h"

#include "RRCrowdROSController.generated.h"

class URRRobotROS2Interface;

/**
 * @brief Base Crowd ROS Controller for ARRBaseRobot descedent classes, utilizing URRCrowdFollowingComponent
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRCrowdROSController : public ARRCrowdAIController
{
    GENERATED_BODY()

protected:
    //! ROS2 Interface handle, which must be null as ROS Controller is created
    UPROPERTY(Transient, EditAnywhere, BlueprintReadWrite)
    URRRobotROS2Interface* ROS2Interface = nullptr;

    /**
     * @brief Initialize by calling #InitRobotROS2Node, #ARRBaseRobot's InitSensors and #InitPublishers.
     *
     * @sa [OnPossess](https://docs.unrealengine.com/4.27/en-US/API/Runtime/AIModule/AAIController/OnPossess/)
     * @param InPawn
     */
    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;
};
