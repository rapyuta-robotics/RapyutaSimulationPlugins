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

#include "RRRobotVehicleROSController.generated.h"

class URRRobotROS2Interface;

/**
 * @brief  Base Robot ROS controller class. Other robot controller class should inherit from this class.
 * This class owns ROS2Node and provide ROS2 interfaces to control robot such as Twist msg.
 * You can find example at #ATurtlebotROSController.
 *
 * @sa [AAIController](https://docs.unrealengine.com/4.27/en-US/API/Runtime/AIModule/AAIController/)
 * @sa https://answers.unrealengine.com/questions/871116/view.html
 * @sa https://answers.unrealengine.com/questions/239159/how-many-ai-controllers-should-i-have.html
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRRobotVehicleROSController : public AAIController
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadWrite)
    bool bWarnAboutMissingLink = true;

protected:
    //! ROS2 Interface handle, which must be null as ROS Controller is created
    UPROPERTY(Transient, EditAnywhere, BlueprintReadWrite)
    URRRobotROS2Interface* ROS2Interface = nullptr;

    /**
     * @brief Initialize by calling #InitRobotROS2Node, #ARobotVehicle's InitSensors and #InitPublishers.
     *
     * @sa [OnPossess](https://docs.unrealengine.com/4.27/en-US/API/Runtime/AIModule/AAIController/OnPossess/)
     * @param InPawn
     */
    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;
};
