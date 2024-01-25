/**
 * @file RRRobotROS2Interface.h
 * @brief Base Robot ROS2Interface class. Other robot ROS2Interface class should inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2NodeComponent.h"
#include "ROS2ServiceClient.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/RRROS2OdomPublisher.h"
#include "Tools/RRROS2TFPublisher.h"

// RapyutaSimulationPlugins
#include "Core/RRUObjectUtils.h"
#include "Robots/RRBaseROS2Interface.h"
#include "Sensors/RRBaseOdomComponent.h"

#include "RRRobotROS2Interface.generated.h"

class ARRBaseRobot;
/**
 * @brief  Base Robot ROS 2 interface class.
 * This class owns ROS2Node and controls ROS 2 interfaces of the #Robot, by
 * - Providing ROS 2 subscribers to control robot joints and movement
 * - Providing Odometry publisher.
 * - Controling #URRROS2BaseSensorComponent in #ARRBaseRobot.
 *
 *
 * Please create child class of this class to custom ROS2Interface which have your own ROS2Interfaces.
 * @todo add handling of service and action.
 */
UCLASS(Blueprintable, EditInlineNew)
class RAPYUTASIMULATIONPLUGINS_API URRRobotROS2Interface : public URRBaseROS2Interface
{
    GENERATED_BODY()

public:
    //! Target robot
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TObjectPtr<ARRBaseRobot> Robot = nullptr;

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    /**
     * @brief Initialize robot's ROS 2 interface by calling #InitRobotROS2Node, #InitPublishers, #InitSubscriptions and #ARRBaseRobot::InitSensors.
     * This method is mainly used by #ARRBaseoRbotROSController via #ARRBaseRobot::InitROS2Interface.
     * @param InRobot
     */
    virtual void Initialize(AActor* Owner) override;

    virtual void InitInterfaces() override;

    /**
     * @brief DeInitialize robot's ROS 2 interface by stopping publisher
     *
     * @param InRobot
     */
    virtual void DeInitialize() override;

    virtual void InitROS2NodeParam(AActor* Owner) override;

    //////////////////////////////
    //Mobile
    //////////////////////////////

    //! Odometry source
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TObjectPtr<URRBaseOdomComponent> OdomComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    bool bPublishOdom = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    bool bPublishOdomTf = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    float OdomPublicationFrequencyHz = 30;

    //! Movement command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString CmdVelTopicName = TEXT("cmd_vel");

    //////////////////////////////
    //Joint
    //////////////////////////////

    /**
     * @brief Move robot joints by setting position or velocity to Pawn(=Robot) with given ROS 2 msg.
     * Supports only 1 DOF joints.
     * Effort control is not supported.
     * @sa [sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html)
     */
    UFUNCTION()
    virtual void JointCmdCallback(const UROS2GenericMsg* Msg);

    //! Joint control command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString JointCmdTopicName = TEXT("ue_joint_commands");

    /**
     * @brief Update Joint State msg
     *
     * @param InMessage
     */
    UFUNCTION()
    void UpdateJointState(UROS2GenericMsg* InMessage);

    //! JointState Publisher
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TObjectPtr<UROS2Publisher> JointStatePublisher = nullptr;

    //! Joint control command topic. If empty is given, subscriber will not be initiated.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    float JointStatePublicationFrequencyHz = 30.f;

    //! Joint state topic
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString JointStateTopicName = TEXT("ue_joint_states");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    bool bWarnAboutMissingLink = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TObjectPtr<URRROS2TFsPublisher> JointsTFPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    bool bPublishJointTf = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    float JointTfPublicationFrequencyHz = 1.f;

    //! Odom publisher
    UPROPERTY(Transient, BlueprintReadWrite, Replicated)
    TObjectPtr<URRROS2OdomPublisher> OdomPublisher = nullptr;

    /**
     * @brief Initialize non sensor basic publishers such as odometry.
     * Overidden in child robot ROS 2 interface classes for specialized publishers.
     */
    virtual bool InitPublishers() override;

    /**
     * @brief Initialize subscriptions for cmd_vel & joint_states topics
     * Overidden in child robot ROS 2 interface classes for specialized topic subscriptions.
     */
    virtual bool InitSubscriptions() override;

    /**
     * @brief MoveRobot by setting velocity to Pawn(=Robot) with given ROS 2 msg.
     * Typically this receive Twist msg to move robot.
     */
    UFUNCTION()
    virtual void MovementCallback(const UROS2GenericMsg* Msg);

protected:
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
                          auto* robot = Cast<TRobot>(Robot);
                          if (IsValid(Cast<UObject>(robot)))
                          {
                              ::Invoke(InMemFunc, robot, msgData);
                          }
                      });
        }
    }

    template<typename TROS2Message, typename TROS2MessageData, typename TRobot>
    FORCEINLINE void OnMessageReceivedFunc(const TROS2Message* InMsg, const TFunction<void(const TROS2MessageData&)>& InFunc)
    {
        if (IsValid(InMsg))
        {
            TROS2MessageData msgData;
            InMsg->GetMsg(msgData);

            // (Note) In this callback, which could be invoked from a ROS working thread,
            // thus any direct referencing to its member in this GameThread lambda needs to be verified.
            AsyncTask(ENamedThreads::GameThread,
                      [this, InFunc, msgData]
                      {
                          auto* robot = Cast<TRobot>(Robot);
                          if (IsValid(Cast<UObject>(robot)))
                          {
                              InFunc(msgData);
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
        if (auto* client = ServiceClientList.FindRef(InServiceName))
        {
            TService* service = CastChecked<TService>(client->Service);
            client->SendRequest(service, InRequest);

            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("%s [%s] Request made"), *InServiceName.ToString(), *GetName());
        }
        else
        {
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[MakeServiceRequest] [%s] srv client not found"), *InServiceName.ToString());
        }
    }
};

/**
 * @brief Wrapper class of URRRobotROS2Interfaceas component
 * This class should be useful to create custom ROSInterface.
 *
 */
UCLASS(Blueprintable, BlueprintType, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRRobotROS2InterfaceComponent : public URRBaseROS2InterfaceComponent
{
    GENERATED_BODY()
public:
    URRRobotROS2InterfaceComponent()
    {
        ROS2InterfaceClass = URRRobotROS2Interface::StaticClass();
    };
};
