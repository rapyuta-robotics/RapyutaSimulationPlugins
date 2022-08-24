/**
 * @file RRBaseRobot.h
 * @brief Base Robot class. Other robot class can inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Drives/RRJointComponent.h"
#include "Tools/ROS2Spawnable.h"

// rclUE
#include "ROS2Node.h"

#include "RRBaseRobot.generated.h"

class ARRNetworkGameState;
class URRRobotROS2Interface;
class ARRNetworkPlayerController;


/**
 * @brief Which server or client has robot movement authority.
 * Server: robot moves in server first and movement replicates to clients.
 * Client: robot moves in client first and use rpc to apply movement to server.
 * @todo implement Server authority.
 */
UENUM(BlueprintType)
enum class ERRNetworkAuthorityType : uint8
{
    SERVER UMETA(DisplayName = "Server"),
    CLIENT UMETA(DisplayName = "Client"),
};

/**
 * @brief Base Robot class. Other robot class should inherit from this class. This actor:
 * - Use #URRRobotROS2Interface as the main ROS2 communication tool
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRBaseRobot : public ARRBaseActor
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ARRRobotBaseVehicle object
     *
     */
    ARRBaseRobot();

    /**
     * @brief Construct a new ARRRobotBaseVehicle object
     *
     * @param ObjectInitializer
     */
    ARRBaseRobot(const FObjectInitializer& ObjectInitializer);

    /**
     * @brief Initialize default components being configurable in child BP classes.
     * Could only be called in constructor.
     */
    void SetupDefault();

    //! Default class to use when ROS2 Interface is setup for robot
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "ROS2 Interface Class"), Replicated)
    TSubclassOf<URRRobotROS2Interface> ROS2InterfaceClass;

    //! Robot's ROS2 Interface
    UPROPERTY(VisibleAnywhere, Replicated, ReplicatedUsing = OnRep_ROS2Interface)
    URRRobotROS2Interface* ROS2Interface = nullptr;
    
    UFUNCTION(BlueprintCallable)
    virtual void OnRep_ROS2Interface();

    /**
     * @brief Flag to start/stop ROS2Interfaces. Since RPC can't be used, use replication to trigger initialization.
     */   
    UPROPERTY(VisibleAnywhere, Replicated, ReplicatedUsing = OnRep_StartStopROS2Interface)
    bool StartStopROS2Interface = false;

    UFUNCTION(BlueprintCallable)
    virtual void OnRep_StartStopROS2Interface();

    /**
     * @brief Check necessary variables has initialized and PlayerId which spaned robot is match the this client PlayerId
     * @return true if playerId matches robot spawn playerId
     */
    bool IsAutorizedInThisClient();

    //!ROSSpawn parameters which is passed to ROS2Interface
    UPROPERTY(VisibleAnywhere, Replicated)
    UROS2Spawnable* ROSSpawnParameters = nullptr;

    /**
     * @brief Pointer to the robot's server-owned version
     * @note Owner can't be used since non-player pawn don't have that.
     */
    UPROPERTY(VisibleAnywhere, Replicated)
    ARRBaseRobot* ServerRobot = nullptr;

    /**
     * @brief Instantiate ROS2 Interface without initializing yet
     * @note can't use rpc since this is not controlled by Paleyr. should add (Server, Reliable)
     */
    UFUNCTION(BlueprintCallable)
    void CreateROS2Interface();

    /**
     * @brief Initialize ROS2 Interface
     * @note This is meant to be called in server only, but can't be an RPC since the robot is not always owned by the same [connection](https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Networking/Actors/OwningConnections) with the client's PlayerController.
     */
    UFUNCTION(BlueprintCallable)
    void InitROS2Interface();

    /**
     * @brief Stop ROS2 Interface
     * @note can't use rpc since this is not controlled by Paleyr. should add (Client, Reliable)
     */
    UFUNCTION(BlueprintCallable)
    void StopROS2Interface();

    /**
     * @brief
     * Actually Object's Name is also unique as noted by UE, but we just do not want to rely on it.
     * Instead, WE USE [RobotUniqueName] TO MAKE THE ROBOT ID CONTROL MORE INDPENDENT of UE INTERNAL NAME HANDLING.
     * Reasons:
     * + An Actor's Name could get updated as its Label is updated
     * + In pending-kill state, GetName() goes to [None]
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString RobotUniqueName;

    /**
     * @brief Get robot unique name
     */
    FString GetRobotName() const
    {
        return RobotUniqueName;
    }

    /**
     * @brief Set robot unique name
     */
    void SetRobotName(const FString& InRobotName)
    {
        RobotUniqueName = InRobotName;
    }

    //! Robot Model Name (loaded from URDF/SDF)
    UPROPERTY(EditAnyWhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString RobotModelName;

    /**
     * @brief Get robot model name
     */
    FString GetModelName() const
    {
        return RobotModelName;
    }

    /**
     * @brief Is static built robot model by the Editor
     */
    FORCEINLINE bool IsBuiltInRobotModel() const
    {
        return RobotModelName.StartsWith(TEXT("UE"), ESearchCase::IgnoreCase);
    }

    //! Robot ID No
    UPROPERTY(EditAnyWhere, Replicated)
    uint64 RobotID = 0;

    /**
     * @brief Get robot ID
     */
    uint64 GetRobotID() const
    {
        return RobotID;
    }

    /**
     * @brief Set robot ID
     */
    void SetRobotID(uint64 InRobotID)
    {
        RobotID = InRobotID;
    }

    /**
     * @brief Manually built links
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite /*, Replicated*/)
    TMap<FString, UStaticMeshComponent*> Links;

    /**
     * @brief Manually built joints
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite /*, Replicated*/)
    TMap<FString, URRJointComponent*> Joints;

    /**
     * @brief Initialize sensors components which are child class of #URRROS2BaseSensorComponent.
     *
     * @param InROS2Node ROS2Node which sensor publishers belongs to.
     * @return true
     * @return false
     *
     * @sa [TInlineComponentArray](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/GameFramework/TInlineComponentArray/)
     * @sa [GetComponents](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/GameFramework/AActor/GetComponents/2/)
     */
    bool InitSensors(AROS2Node* InROS2Node);

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    /**
     * @brief Allows a component to replicate other subobject on the actor
     *
     */
    virtual bool ReplicateSubobjects(UActorChannel *Channel, FOutBunch *Bunch, FReplicationFlags *RepFlags) override;

    /**
     * @brief Set Joints state to #Joints
     */
    // UFUNCTION(BlueprintCallable)
    virtual void SetJointState(const TMap<FString, TArray<float>>& InJointState, const ERRJointControlType InJointControlType);


    /**
     * @brief Network Authority.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite /*, Replicated*/)
    ERRNetworkAuthorityType NetworkAuthorityType = ERRNetworkAuthorityType::CLIENT;

protected:
    /**
     * @brief Instantiate default child components
     */
    virtual void PostInitializeComponents() override;
};
