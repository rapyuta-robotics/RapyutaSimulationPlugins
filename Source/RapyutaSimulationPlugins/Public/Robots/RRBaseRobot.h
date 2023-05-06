/**
 * @file RRBaseRobot.h
 * @brief Base Robot class. Other robot class can inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/StaticMeshComponent.h"
#include "Components/WidgetComponent.h"
#include "CoreMinimal.h"

// rclUE
#include "ROS2NodeComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Drives/RRJointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Sensors/RRROS2BaseSensorComponent.h"
#include "Tools/ROS2Spawnable.h"

#include "RRBaseRobot.generated.h"

#define RR_VERIFY_DYNAMIC_ROBOT(InRobot)          \
    if (InRobot)                                  \
    {                                             \
        verify(InRobot->IsDynamicRuntimeRobot()); \
    }
#define RR_VERIFY_STATIC_BP_ROBOT(InRobot)  \
    if (InRobot)                            \
    {                                       \
        verify(InRobot->IsStaticBPRobot()); \
    }
#define RR_VERIFY_DYNAMIC_OR_STATIC_BP_ROBOT(InRobot) \
    {                                                 \
    }

class ARRNetworkGameState;
class URRRobotROS2Interface;
class ARRNetworkPlayerController;
class URRUserWidget;

/**
 * @brief Which server or client has robot movement authority.
 * @todo Implement Server authority.
 */
UENUM(BlueprintType)
enum class ERRNetworkAuthorityType : uint8
{
    SERVER UMETA(DisplayName = "Server", ToolTip = "robot moves in server first and movement replicates to clients."),
    CLIENT UMETA(DisplayName = "Client", ToolTip = "robot moves in client first and use rpc to apply movement to server.")
};

DECLARE_MULTICAST_DELEGATE_OneParam(FOnRobotCreationDone, bool /* bCreationResult */);

/**
 * @brief Base Robot class. Other robot class should inherit from this class.
 * This actor use #URRRobotROS2Interface as the main ROS2 communication tool.
 * This actor has basic functionality to use with client-server, e.g. replication setting
 * - Moves kinematically with #URobotVehicleMovementComponent.
 * - Is possessed by #ARRRobotVehicleROSController to be control from ROS2.
 * You can find example at #ATurtlebotBurger.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRBaseRobot : public ARRBaseActor
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ARRBaseRobot object
     *
     */
    ARRBaseRobot();

    /**
     * @brief Construct a new ARRBaseRobot object
     *
     * @param ObjectInitializer
     */
    ARRBaseRobot(const FObjectInitializer& ObjectInitializer);

    /**
     * @brief BeginPlay
     */
    virtual void BeginPlay() override;

    /**
     * @brief Wake rigid body in addition to Super::Tick()
     *
     * @param DeltaSeconds
     */
    virtual void Tick(float DeltaSeconds) override;

    /**
     * @brief Initialize default components being configurable in child BP classes.
     * Could only be called in constructor.
     */
    void SetupDefault();

    UPROPERTY(VisibleAnywhere, Replicated)
    USceneComponent* DefaultRoot = nullptr;
    /**
     * @brief Set the root offset for #RobotVehicleMoveComponent
     * This will be added to the odometry data published in ros topic /odom
     * It is used, for example, to allow the robot root pose to remain constant even if we move the skeletal mesh root component for
     * collisions
     *
     * @param InRootOffset
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetRootOffset(const FTransform& InRootOffset);

    /**
     * @brief DynamicRuntime robot: Implemented purely in cpp, built & loaded up at runtime from raw CAD + metadata (URDF/SDF)
     */
    FORCEINLINE bool IsDynamicRuntimeRobot() const
    {
        return (false == IsStaticBPRobot());
    }

    /**
     * @brief Static BP robot: implemented in BP, possibly inheriting from #ARBaseRobot or its children classes,
     * built from pre-designed static UE assets (StaticMesh, SkeletalMesh, Skeleton, Physics Asset, etc.)
     */
    FORCEINLINE bool IsStaticBPRobot() const
    {
        const FString className = GetClass()->GetName();
        return className.StartsWith(TEXT("BP"))             // In-Editor
               || className.StartsWith(TEXT("SKEL_BP"));    // In-Package auto prefixed [SKEL_]
    }

    //! Robot creation done delegate
    FOnRobotCreationDone OnRobotCreationDone;

    //! Default class to use when ROS2 Interface is setup for robot
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "ROS2 Interface Class"), Replicated)
    TSubclassOf<URRRobotROS2Interface> ROS2InterfaceClass;

    /**
     * Robot's ROS2 Interface.
     * With the client-server setup, this is created in the server and replicated to the client and initialized only in the client.
     */
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Replicated, ReplicatedUsing = OnRep_ROS2Interface)
    URRRobotROS2Interface* ROS2Interface = nullptr;

    /**
     * @brief Function called with #ROS2Interface replication. Start ROS2Interface if bStartStopROS2Interface=true.
     */
    UFUNCTION(BlueprintCallable)
    virtual void OnRep_ROS2Interface();

    /**
     * @brief Flag to start/stop ROS2Interfaces. Since RPC can't be used, use replication to trigger initialization.
     */
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Replicated, ReplicatedUsing = OnRep_bStartStopROS2Interface)
    bool bStartStopROS2Interface = false;

    /**
     * @brief Function called with #bStartStopROS2Interface replication. Start/stop ROS2Interface if it is ready.
     */
    UFUNCTION(BlueprintCallable)
    virtual void OnRep_bStartStopROS2Interface();

    /**
     * @brief Check necessary variables has initialized and PlayerId which spawned robot is match to this client PlayerId

     * @return true if playerId matches robot spawn playerId
     */
    bool IsAuthorizedInThisClient();

    //! ROSSpawn parameters which is passed to ROS2Interface
    //! You can change paramter in BP for manually placed robot but
    //! Paramerter will be overwirten if you spawn from /SpawnEntity srv.
    UPROPERTY(BlueprintReadWrite, Replicated)
    UROS2Spawnable* ROSSpawnParameters = nullptr;

    /**
     * @brief Pointer to the robot's server-owned version
     * @note Owner can't be used since non-player pawn don't have that.
     */
    UPROPERTY(VisibleAnywhere, Replicated)
    ARRBaseRobot* ServerRobot = nullptr;

    /**
     * @brief Instantiate ROS2 Interface without initializing yet
     * @note Not uses RPC but replication since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa #ARRBaseRobotROSController::OnPossess
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     *
     */
    UFUNCTION(BlueprintCallable)
    void CreateROS2Interface();

    /**
     * @brief Initialize ROS2 Interface. Directly call #URRRobotROS2Interface::Initialize or execute in client via #OnRep_bStartStopROS2Interface.
     * @note Not uses RPC but replication since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa #ARRBaseRobotROSController::OnPossess
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     */
    UFUNCTION(BlueprintCallable)
    void InitROS2Interface();

    UFUNCTION(BlueprintCallable)
    bool InitROS2InterfaceImpl();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle ROS2InitTimer;

    /**
     * @brief Stop ROS2 Interface. Directly call #URRRobotROS2Interface::DeInitialize or execute in client via #OnRep_bStartStopROS2Interface.
     * @note Not uses RPC but replication since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     */
    UFUNCTION(BlueprintCallable)
    void DeInitROS2Interface();

    /**
     * @brief
     * Actually Object's Name is also unique as noted by UE, but we just do not want to rely on it.
     * Instead, we use [RobotUniqueName] to make the robot id control more indpendent of ue internal name handling.
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
    UPROPERTY(VisibleAnyWhere, BlueprintReadOnly, meta = (ExposeOnSpawn = "true"), Replicated)
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
     * Robot Links
     * @todo adopt to client-server.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, UStaticMeshComponent*> Links;

    /**
     * Robot Joints
     * @todo adopt to client-server.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, URRJointComponent*> Joints;

    /**
     * @brief Initialize sensors components which are child class of #URRROS2BaseSensorComponent.
     *
     * @param InROS2Node ROS2Node which sensor publishers belongs to.
     * @return true
     * @return false Given ROS2Node is invalid.
     *
     * @sa [TInlineComponentArray](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/GameFramework/TInlineComponentArray/)
     * @sa [GetComponents](https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/GameFramework/AActor/GetComponents/2/)
     */
    virtual bool InitSensors(UROS2NodeComponent* InROS2Node);

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
    virtual bool ReplicateSubobjects(UActorChannel* Channel, FOutBunch* Bunch, FReplicationFlags* RepFlags) override;

    /**
     * @brief Set Joints state to #Joints
     * @todo Provide a simillar method which can be used from Blueprint
     */
    // UFUNCTION(BlueprintCallable)
    virtual void SetJointState(const TMap<FString, TArray<float>>& InJointState, const ERRJointControlType InJointControlType);

    /**
     * @brief Network Authority Type.
     * @todo Server is not supported yet.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    ERRNetworkAuthorityType NetworkAuthorityType = ERRNetworkAuthorityType::CLIENT;

    // MOVEMENT --
    //
    //! Local target linear vel
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector TargetLinearVel = FVector::ZeroVector;

    //! Local target angular vel
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector TargetAngularVel = FVector::ZeroVector;

    /**
     * @brief Stop robot movement, resetting all vel inputs
     */
    UFUNCTION(BlueprintCallable)
    virtual void StopMovement();

    //! Main robot movement component (kinematics/diff-drive or wheels-drive comp)
    //! #MovementComponent and #RobotVehicleMoveComponent should point to same pointer.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    UMovementComponent* MovementComponent = nullptr;

    //! Movecomponent casted to #URobotVehicleMovementComponent for utility.
    //! This should be pointing same thing as #MovementComponent
    //! This should be set from #SetMoveComponent
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Replicated)
    URobotVehicleMovementComponent* RobotVehicleMoveComponent = nullptr;

    //! Class of the main robot movement component, configurable in child class
    //! If VehicleMoveComponentClass == nullptr, it is expected that MovementComponent is set from BP or user code.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TSubclassOf<UMovementComponent> VehicleMoveComponentClass;

    UFUNCTION(BlueprintCallable)
    virtual void SetMoveComponent(UMovementComponent* InMoveComponent);

    /**
     * @brief Set velocity to #RobotVehicleMoveComponent.
     * Calls #SetLocalLinearVel for setting velocity to #RobotVehicleMoveComponent and
     * #SyncServerLinearMovement to sync movement of the robot in the server.
     *
     * @param InLinearVel
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetLinearVel(const FVector& InLinearVel);

    /**
     * @brief Set angular velocity to #RobotVehicleMoveComponent
     * Calls #SetLocalAngularVel for setting velocity to #RobotVehicleMoveComponent and
     * #SyncServerAngularMovement to sync movement of the robot in the server.
     * @param InAngularVel
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetAngularVel(const FVector& InAngularVel);

    /**
     * @brief Set position and linear velocity to the robot in the server.
     * @param InClientTimeStamp
     * @param InClientRobotPosition
     * @param InLinearVel
     * @note Not uses RPC since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     */
    UFUNCTION(BlueprintCallable)
    virtual void SyncServerLinearMovement(float InClientTimeStamp,
                                          const FTransform& InClientRobotTransform,
                                          const FVector& InLinearVel);

    /**
     * @brief Set  rotation and angular velocity to the robot in the server.
     * @param InClientTimeStamp
     * @param InClientRobotRotation
     * @param InAngularVel
     * @note Not uses RPC since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     */
    UFUNCTION(BlueprintCallable)
    virtual void SyncServerAngularMovement(float InClientTimeStamp,
                                           const FRotator& InClientRobotRotation,
                                           const FVector& InAngularVel);

    /**
     * @brief Set linear velocity to #RobotVehicleMoveComponent in the client.
     * @note Not uses RPC since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetLocalLinearVel(const FVector& InLinearVel);

    /**
     * @brief Set angular velocity to #RobotVehicleMoveComponent in the client.
     * @note Not uses RPC since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetLocalAngularVel(const FVector& InAngularVel);

    //! Offset transform between the Actor  root component and the pose that will be published in /odom topic
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform RootOffset = FTransform::Identity;

    //! Call ConfigureMovecomponent and RobotVehicleMoveComponent::Initialize() in InitMoveComponent in PostInitializeComponents or not.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bInitRobotVehicleMoveComponent = true;

    /**
     * @brief This method is called inside #PostInitializeComponents.
     * Custom initialization of child BP class can be done by overwritting this method.
     *
     */
    UFUNCTION(BlueprintImplementableEvent, BlueprintCallable)
    void BPConfigureMovementComponent();

    // TOOLTIP --
    UPROPERTY()
    uint8 bTooltipEnabled : 1;
    //! Tooltip widget component
    UPROPERTY()
    TObjectPtr<UWidgetComponent> TooltipComp = nullptr;
    //! Relative pose of the tooltip from the owner robot
    UPROPERTY()
    FTransform TooltipOffset = FTransform(FVector(0.f, 0.f, 100.f));
    /**
     * @brief Set text for #Tooltip
     * @param InTooltip
     */
    void SetTooltipText(const FString& InTooltip);

    /**
     * @brief Set visibility of #Tooltip
     * @param bInVisible
     */
    void SetTooltipVisible(bool bInVisible);

protected:
    /**
     * @brief Instantiate default child components
     */
    virtual void PreInitializeComponents() override;

    /**
     * @brief Post Initialization process of actor. Initialize #RobotVehicleMoveComponent by calling #InitMoveComponent.
     * @sa[ActorLifecycle](https://docs.unrealengine.com/5.1/en-US/ProgrammingAndScripting/ProgrammingWithCPP/UnrealArchitecture/Actors/ActorLifecycle/)
     * @sa[PostInitializeComponents](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/AActor/PostInitializeComponents/)
     */
    virtual void PostInitializeComponents() override;

    /**
     * @brief Create and Initialize #MovementComponent if #VehicleMoveComponentClass != nullptr.
     * If VehicleMoveComponentClass == nullptr, it is expected that MovementComponent is set from BP or user code.
     *
     * @return true #MovementComponent is created and initialized.
     * @return false #VehicleMoveComponentClass == nullptr.
     */
    virtual bool InitMoveComponent();

    /**
     * @brief This method is called inside #PostInitializeComponents.
     * Custom initialization of child class can be done by overwritting this method.
     *
     */
    virtual void ConfigureMovementComponent();

    /**
     * @brief Create & init #TooltipComp
     */
    virtual void InitTooltip();
};
