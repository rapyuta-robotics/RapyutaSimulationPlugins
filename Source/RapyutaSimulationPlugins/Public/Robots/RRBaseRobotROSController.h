/**
 * @file RRRobotVehicleROSController.h
 * @brief Base Robot ROS controller class. Other robot controller class should inherit from this class. Example is
 * #ATurtlebotROSController.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "AIController.h"
#include "CoreMinimal.h"

#include "RRBaseRobotROSController.generated.h"

class URRBaseROS2Interface;

/**
 * @brief  Base Robot ROS controller class. Other robot controller class should inherit from this class.
 * This class has authority to start ROS 2 Component in pausses robot.
 *
 * @sa [AAIController](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/)
 * @sa https://answers.unrealengine.com/questions/871116/view.html
 * @sa https://answers.unrealengine.com/questions/239159/how-many-ai-controllers-should-i-have.html
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRBaseRobotROSController : public AAIController
{
    GENERATED_BODY()
protected:
    /**
     * @brief Initialize robot pawn by calling #URRBaseROS2Interface::Initialize or #ARRBaseRobot::InitROS2Interface.
     *
     * @sa [OnPossess](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/OnPossess/)
     * @param InPawn
     */
    virtual void OnPossess(APawn* InPawn) override;

    /**
     * @brief Deinitialize robot pawn by calling #URRBaseROS2Interface::DeInitialize or #ARRBaseRobot::DeInitROS2Interface.
     *
     * @sa [OnUnPossess](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/OnUnPossess/)
     */
    virtual void OnUnPossess() override;

    /**
     * This is spawn in #OnPossess if controlled pawn is not child class of RRBaseRobot
    */
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRBaseROS2Interface* ROS2Interface = nullptr;
};
