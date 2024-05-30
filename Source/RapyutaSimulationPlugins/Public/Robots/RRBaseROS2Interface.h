/**
 * @file RRBaseROS2Interface.h
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
#include "Sensors/RRBaseOdomComponent.h"

#include "RRBaseROS2Interface.generated.h"

class ARRBaseRobot;
/**
 * @brief  Base Robot ROS 2 interface class.
 * This class owns ROS2Node and controls ROS 2 interfaces
 *
 * Please create child class of this class to custom ROS2Interface which have your own ROS2Interfaces.
 */
UCLASS(Blueprintable, EditInlineNew)
/**
 * @class URRBaseROS2Interface
 * @brief This class represents the base ROS 2 interface for a robot in the RapyutaSimulationPlugins module.
 *        It provides functionality for initializing and managing ROS 2 nodes, publishers, subscribers, service clients,
 *        service servers, action clients, and action servers.
 */
class RAPYUTASIMULATIONPLUGINS_API URRBaseROS2Interface : public UObject
{
    GENERATED_BODY()

#define RR_ROBOT_ROS2_SUBSCRIBE_TO_TOPIC(InTopicName, InMsgClass, InCallback) \
    ROS2_CREATE_SUBSCRIBER(RobotROS2Node, this, InTopicName, InMsgClass, InCallback)

public:
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

    //! ROS 2 node of this interface created by #InitRobotROS2Node
    //! @todo rename to ROS2Node
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TObjectPtr<UROS2NodeComponent> RobotROS2Node = nullptr;

    //! ROS2SpawnParameters which is created when robot is spawn from /SpawnEntity srv provided by #ASimulationState.
    UPROPERTY(VisibleAnywhere, Replicated)
    TObjectPtr<UROS2Spawnable> ROSSpawnParameters = nullptr;

    //! Defautl ROS namespace.
    //! This is not used if robot spawn from ROS or Pawn is #ARRBaseRobot
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString DefautlROSNamespace = TEXT("");

    //! This will overwrite the namespace of the ROS2Node with the actor's name.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bUseActorNameAsNamespace = true;

    /**
     * @brief Initialize robot's ROS 2 interface by calling #InitRobotROS2Node, #InitPublishers, #InitSubscriptions and #ARRBaseRobot::InitSensors.
     * This method is mainly used by #ARRBaseoRbotROSController via #ARRBaseRobot::InitROS2Interface.
     * @param InRobot
     */
    UFUNCTION(BlueprintCallable)
    virtual void Initialize(AActor* Owner);

    /**
     * @brief Init publishers, subscribers, service clients, service servers, action clients, and action servers.
     *
     */
    virtual void InitInterfaces();

    /**
     * @brief AdditionalInitialization implemented in BP.
     * Child BP class can use this method to add initialization behaviour
     */
    UFUNCTION(BlueprintImplementableEvent, BlueprintCallable)
    void BPInitialize();

    /**
     * @brief DeInitialize robot's ROS 2 interface by stopping publisher
     *
     * @param InRobot
     */
    virtual void DeInitialize();

    /**
     * @brief Spawn ROS2Node and initialize it.
     *
     * @param InPawn
     */
    virtual void InitRobotROS2Node(AActor* Owner);

    /**
     * @brief Spawn ROS2 Node, and initialize it if ROS2 Node = nullptr
     *
     * @param Owner
     */
    virtual void InitROS2NodeParam(AActor* Owner);

    /**
     * @brief Setup ROS Params, overridable by child classes to config custom ROS 2 Interface's params
     */
    UFUNCTION()
    virtual void SetupROSParams(){};

    /**
     * @brief Setup ROS params called after #SetupROSParams. override by BP child class to config custom ROS 2 Interface's params
    */
    UFUNCTION(BlueprintImplementableEvent, BlueprintCallable)
    void BPSetupROSParams();

    /**
     * @brief Call #SetupROSParams and #BPSetupROSParams
     */
    UFUNCTION(BlueprintCallable)
    void SetupROSParamsAll()
    {
        SetupROSParams();
        BPSetupROSParams();
    }

    //! You can add your publishers here to ask ROS2Interface to manage.
    //! Other option is to create child class to overwrite each method.
    UPROPERTY(Transient, BlueprintReadWrite)
    TMap<FString, UROS2Publisher*> Publishers;

    //! You can add your subscribers here to ask ROS2Interface to manage.
    //! Other option is to create child class to overwrite each method.
    UPROPERTY(Transient, BlueprintReadWrite)
    TMap<FString, UROS2Subscriber*> Subscribers;

    //! You can add your publishers here to ask ROS2Interface to manage.
    //! Other option is to create child class to overwrite each method.
    UPROPERTY(Transient, BlueprintReadWrite)
    TMap<FString, UROS2ServiceClient*> ServiceClients;

    //! You can add your publishers here to ask ROS2Interface to manage.
    //! Other option is to create child class to overwrite each method.
    UPROPERTY(Transient, BlueprintReadWrite)
    TMap<FString, UROS2ServiceServer*> ServiceServers;

    //! You can add your publishers here to ask ROS2Interface to manage.
    //! Other option is to create child class to overwrite each method.
    UPROPERTY(Transient, BlueprintReadWrite)
    TMap<FString, UROS2ActionClient*> ActionClients;

    //! You can add your publishers here to ask ROS2Interface to manage.
    //! Other option is to create child class to overwrite each method.
    UPROPERTY(Transient, BlueprintReadWrite)
    TMap<FString, UROS2ActionServer*> ActionServers;

    /**
     * @brief Initialize non sensor basic publishers such as odometry.
     * Overidden in child robot ROS 2 interface classes for specialized publishers.
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
     * Overidden in child robot ROS 2 interface classes for specialized topic subscriptions.
     */
    UFUNCTION()
    virtual bool InitSubscriptions();

    /**
     * @brief Initialize service clients
     * Overidden in child robot ROS 2 interface classes for specialized services clients.
     */
    UFUNCTION()
    virtual bool InitServiceClients();

    /**
     * @brief Initialize service servers
     * Overidden in child robot ROS 2 interface classes for specialized services servers.
     */
    UFUNCTION()
    virtual bool InitServiceServers();

    /**
     * @brief Initialize action clients
     * Overidden in child robot ROS 2 interface classes for specialized action clients.
     */
    UFUNCTION()
    virtual bool InitActionClients();

    /**
     * @brief Initialize action servers
     * Overidden in child robot ROS 2 interface classes for specialized action servers.
     */
    UFUNCTION()
    virtual bool InitActionServers();
};

/**
 * @brief Wrapper class of URRBaseROS2Interfaceas component
 * This class should be useful to create custom ROSInterface.
 *
 */
UCLASS(Blueprintable, BlueprintType, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRBaseROS2InterfaceComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Instanced)
    TObjectPtr<URRBaseROS2Interface> ROS2Interface = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URRBaseROS2Interface> ROS2InterfaceClass = URRBaseROS2Interface::StaticClass();

    //! add all subcomonents of owner to #ROS2Interface
    //! You can manually add one by one instead of using this class.
    UFUNCTION(BlueprintCallable)
    void AddAllSubComponentToROSInterface();
};
