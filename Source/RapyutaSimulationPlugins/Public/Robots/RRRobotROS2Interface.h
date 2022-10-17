/**
 * @file RRRobotROS2Interface.h
 * @brief Base Robot ROS2Interface class. Other robot ROS2Interface class should inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"
#include "ROS2ServiceClient.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/RRROS2OdomPublisher.h"

#include "RRRobotROS2Interface.generated.h"

class ARRBaseRobot;
/**
 * @brief  Base Robot ROS2 interface class.
 * This class owns ROS2Node and controls ROS2 interfaces of the #Robot, by
 * - Providing ROS2 subscribers to control robot joints and movement 
 * - Providing Odometry publisher.
 * - Controling #URRROS2BaseSensorComponent in #ARRBaseRobot.
 *
 * 
 * Please create child class of this class to custom ROS2Interface which have your own ROS2Interfaces.
 * @todo add handling of service and action.
 */
UCLASS(Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API URRRobotROS2Interface : public UObject
{
    GENERATED_BODY()

#define RR_ROBOT_ROS2_SUBSCRIBE_TO_TOPIC(InTopicName, InMsgClass, InCallback) \
    RR_ROS2_SUBSCRIBE_TO_TOPIC(RobotROS2Node, this, InTopicName, InMsgClass, InCallback)

public:
    //! Target robot
    UPROPERTY(Transient, Replicated)
    ARRBaseRobot* Robot = nullptr;

    virtual bool IsSupportedForNetworking() const override
    {
        return true;
    }

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    //! ROS2 node of this interface created by #InitRobotROS2Node
    UPROPERTY(Transient, Replicated)
    AROS2Node* RobotROS2Node = nullptr;

    //! ROS2SpawnParameters which is created when robot is spawn from /SpawnEntity srv provided by #ASimulationState.
    UPROPERTY(VisibleAnywhere, Replicated)
    UROS2Spawnable* ROSSpawnParameters = nullptr;

    /**
     * @brief Initialize robot's ROS2 interface by calling #InitRobotROS2Node, #InitPublishers, #InitSubscriptions and #ARRBaseRobot::InitSensors.
     * This method is mainly used by #ARRBaseoRbotROSController via #ARRBaseRobot::InitROS2Interface.
     * @param InRobot
     */
    virtual void Initialize(ARRBaseRobot* InRobot);

    /**
     * @brief DeInitialize robot's ROS2 interface by stopping publisher
     *
     * @param InRobot
     */
    virtual void DeInitialize();

    /**
     * @brief Spawn ROS2Node and initialize it.
     *
     * @param InPawn
     */
    void InitRobotROS2Node(ARRBaseRobot* InRobot);

    /**
     * @brief Move robot joints by setting position or velocity to Pawn(=Robot) with given ROS2 msg.
     * Supports only 1 DOF joints.
     * Effort control is not supported.
     * @sa [sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html)
     */
    UFUNCTION()
    virtual void JointStateCallback(const UROS2GenericMsg* Msg);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    bool bPublishOdom = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    bool bPublishOdomTf = false;

    //! Movement command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(BlueprintReadWrite, Replicated)
    FString CmdVelTopicName = TEXT("cmd_vel");

    //! Joint control command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(BlueprintReadWrite, Replicated)
    FString JointsCmdTopicName = TEXT("joint_states");

    UPROPERTY(BlueprintReadWrite, Replicated)
    bool bWarnAboutMissingLink = true;

    /**
     * @brief Setup ROS Params, overridable by child classes to config custom ROS2 Interface's params
     */
    UFUNCTION()
    virtual void SetupROSParams();

    //! Odom publisher
    UPROPERTY(Transient, BlueprintReadWrite, Replicated)
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
     * @brief Initialize services clients
     * Overidden in child robot ROS2 interface classes for specialized services clients.
     */
    UFUNCTION()
    virtual bool InitServicesClients();

    /**
     * @brief Stop all service clients
     */
    UFUNCTION()
    virtual void StopServicesClients();

    /**
     * @brief MoveRobot by setting velocity to Pawn(=Robot) with given ROS2 msg.
     * Typically this receive Twist msg to move robot.
     */
    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

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

    UPROPERTY()
    TMap<FName /*ServiceName*/, UROS2ServiceClient*> ServiceClientList;

    template<typename TService, typename TServiceRequest>
    void MakeServiceRequest(const FName& InServiceName, const TServiceRequest& InRequest)
    {
        // Create and update request
        auto* clienPtr = ServiceClientList.Find(InServiceName);
        if (clienPtr)
        {
            UROS2ServiceClient* client = *clienPtr;
            TService* service = CastChecked<TService>(client->Service);
            client->SendRequest(service, InRequest);

            UE_LOG(LogTemp, Warning, TEXT("%s [%s] Request made"), *InServiceName.ToString(), *GetName());
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[MakeServiceRequest] [%s] srv client not found"), *InServiceName.ToString());
        }
    }
};
