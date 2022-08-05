/**
 * @file RRRobotROS2Interface.h
 * @brief Base Robot ROS controller class. Other robot ROS2 class should inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"
#include "Tools/RRROS2OdomPublisher.h"

#include "RRRobotROS2Interface.generated.h"

class ARRBaseRobot;
/**
 * @brief  Base Robot ROS2 interface class.
 * This class owns ROS2Node and provide ROS2 interfaces to control robot such as Twist msg.
 */
UCLASS(Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API URRRobotROS2Interface : public UObject
{
    GENERATED_BODY()

#define RR_ROBOT_ROS2_SUBSCRIBE_TO_TOPIC(InTopicName, InMsgClass, InCallback) \
    RR_ROS2_SUBSCRIBE_TO_TOPIC(RobotROS2Node, this, InTopicName, InMsgClass, InCallback)

public:
    //! Target robot
    UPROPERTY(Transient)
    ARRBaseRobot* Robot = nullptr;

    //! Target ROS2 node of this interface
    UPROPERTY(Transient)
    AROS2Node* RobotROS2Node = nullptr;

    /**
     * @brief Initialize robot's ROS2 interface
     *
     * @param InRobot
     */
    virtual void Initialize(ARRBaseRobot* InRobot);

    /**
     * @brief Spawn ROS2Node and initialize it. This method is mainly used by #ASimulationState to spawn from ROS2 service.
     *
     * @param InPawn
     */
    void InitRobotROS2Node(ARRBaseRobot* InRobot);

    UPROPERTY(Transient, BlueprintReadWrite)
    URRROS2OdomPublisher* OdomPublisher = nullptr;

    /**
     * @brief Initialize non sensor basic publishers such as odometry.
     * Overidden in child robot ROS2 interface classes for specialized publishers.
     */
    UFUNCTION()
    virtual bool InitPublishers();

    /**
     * @brief Stop all publishers
     *
     */
    UFUNCTION()
    virtual void StopPublishers();

    /**
     * @brief Initialize subscriptions for cmd_vel & joint_states topics
     * Overidden in child robot ROS2 interface classes for specialized topic subscriptions.
     */
    UFUNCTION()
    virtual void InitSubscriptions();

    /**
     * @brief MoveRobot by setting velocity to Pawn(=Robot) with given ROS2 msg.
     * Typically this receive Twist msg to move robot.
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

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bPublishOdom = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bPublishOdomTf = false;

    //! Movement command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(BlueprintReadWrite)
    FString CmdVelTopicName = TEXT("cmd_vel");

    //! Joint control command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(BlueprintReadWrite)
    FString JointsCmdTopicName = TEXT("joint_states");

    UPROPERTY(BlueprintReadWrite)
    bool bWarnAboutMissingLink = true;

protected:
    /**
     * @brief Create a ROS2 publisher
     *
     * @param InTopicName
     * @param OutPublisher
     * @param InPublisherClass
     * @param InMsgClass
     * @param InPubFrequency
     */
    void CreatePublisher(const FString& InTopicName,
                         const TSubclassOf<UROS2Publisher>& InPublisherClass,
                         const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                         int32 InPubFrequency,
                         uint8 InQoS,
                         UROS2Publisher*& OutPublisher);

    template<typename TROS2Message,
             typename TROS2MessageData,
             typename TRobot,
             typename TRobotMemFuncType = void (TRobot::*)(const TROS2MessageData&)>
    FORCEINLINE void OnMessageReceived(const TROS2Message* InMsg, const TRobotMemFuncType& InMemFunc)
    {
        const auto* msg = Cast<TROS2Message>(InMsg);
        if (IsValid(msg))
        {
            TROS2MessageData msgData;
            msg->GetMsg(msgData);

            // (Note) In this callback, which could be invoked from a ROS working thread,
            // thus any direct referencing to its member in this GameThread lambda needs to be verified.
            AsyncTask(ENamedThreads::GameThread,
                      [this, InMemFunc, msgData]
                      {
                          auto* robot = CastChecked<TRobot>(Robot);
                          if (IsValid(Cast<UObject>(robot)))
                          {
                              ::Invoke(InMemFunc, robot, msgData);
                          }
                      });
        }
    }
};
