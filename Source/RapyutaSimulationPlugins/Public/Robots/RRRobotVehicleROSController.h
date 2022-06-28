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

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "ROS2Node.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"

#include "RRRobotVehicleROSController.generated.h"

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
    UPROPERTY(Transient)
    AROS2Node* RobotROS2Node = nullptr;

    /**
     * @brief Spawn ROS2Node and initialize it. This method is mainly used by #ASimulationState to spawn from ROS2 service.
     *
     * @param InPawn
     */
    void InitRobotROS2Node(APawn* InPawn);

    //! @todo is this used?
    UPROPERTY()
    FVector InitialPosition = FVector::ZeroVector;

    //! @todo is this used?
    UPROPERTY()
    FRotator InitialOrientation = FRotator::ZeroRotator;

    UPROPERTY(Transient, BlueprintReadWrite)
    URRROS2OdomPublisher* OdomPublisher = nullptr;

    /**
     * @brief Initialize non sensor publishes such as odometry.
     *
     */
    UFUNCTION()
    virtual bool InitPublishers(APawn* InPawn);

    /**
     * @brief MoveRobot by setting velocity to Pawn(=Robot) with given ROS2 msg.
     * Typically this receive Twist msg to move robot.
     *
     */
    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

    /**
     * @brief Move robot joints by setting position or velocity to Pawn(=Robot) with given ROS2 msg.
     * Supports only 1 DOF joints.
     * Effort control is not supported.
     * @sa [sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html)
     */
    UFUNCTION()
    virtual void JointStateCallback(const UROS2GenericMsg* Msg);

    /**
     * @brief Initialize by calling #InitRobotROS2Node, #ARobotVehicle's InitSensors and #InitPublishers.
     *
     * @sa [OnPossess](https://docs.unrealengine.com/4.27/en-US/API/Runtime/AIModule/AAIController/OnPossess/)
     * @param InPawn
     */
    virtual void OnPossess(APawn* InPawn) override;

    virtual void OnUnPossess() override;

    /**
     * @brief Setup ROS2 subscriber by binding #MovementCallback.
     *
     * @param InTopicName
     */
    UFUNCTION(BlueprintCallable)
    void SubscribeToMovementCommandTopic(const FString& InTopicName);

    /**
     * @brief Setup ROS2 subscriber by binding #JointStateCallback.
     *
     * @param InTopicName
     */
    UFUNCTION(BlueprintCallable)
    void SubscribeToJointsCommandTopic(const FString& InTopicName);

    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdom = true;

    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdomTf = false;

    //! Movement command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(BlueprintReadWrite)
	FString CmdVelTopicName = TEXT("cmd_vel");

    //! Joint control command topic. If empty is given, subscriber will not be initiated.
	UPROPERTY(BlueprintReadWrite)
	FString JointsCmdTopicName = TEXT("joint_states");

};
