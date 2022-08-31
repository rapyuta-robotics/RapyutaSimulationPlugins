/**
 * @file RRCrowdROSController.h
 * @brief Base Crowd ROS Controller for ARRBaseRobot descedent classes, utilizing UCrowdFollowingComponent
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Core/RRCrowdAIController.h"

#include "RRCrowdROSController.generated.h"

class URRRobotROS2Interface;

/**
 * @brief Base Crowd ROS Controller for ARRBaseRobot descedent classes, utilizing UCrowdFollowingComponent
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRCrowdROSController : public ARRCrowdAIController
{
    GENERATED_BODY()

protected:
    /**
     * @brief Initialize robot pawn by calling #ARRBaseRobot::InitROS2Interface.
     *
     * @sa [OnPossess](https://docs.unrealengine.com/4.27/en-US/API/Runtime/AIModule/AAIController/OnPossess/)
     * @param InPawn
     */
    virtual void OnPossess(APawn* InPawn) override;

    /**
     * @brief Deinitialize robot pawn by calling #ARRBaseRobot::DeInitROS2Interface.
     *
     * @sa [OnUnPossess](https://docs.unrealengine.com/4.27/en-US/API/Runtime/AIModule/AAIController/OnUnPossess/)
     */
    virtual void OnUnPossess() override;
};
