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
#include "Core/RRObjectCommon.h"
#include "Drives/RRJointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Sensors/RRROS2BaseSensorComponent.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/RRUIWidgetComponent.h"

#include "RRBaseRobot.generated.h"

#define RR_VERIFY_DYNAMIC_ROBOT(InRobot)                                                                                  \
    if (InRobot && (false == InRobot->IsDynamicRuntimeRobot()))                                                           \
    {                                                                                                                     \
        UE_LOG_WITH_INFO(LogTemp,                                                                                         \
                         Error,                                                                                           \
                         TEXT("[ClassName:%s] is required to be NOT prefixed with [BP_] or [SKEL_BP] if this is purely a" \
                              "cpp-based robot class!"),                                                                  \
                         *InRobot->GetClass()->GetName());                                                                \
    }

#define RR_VERIFY_STATIC_BP_ROBOT(InRobot)                                                                                 \
    if (InRobot && (false == InRobot->IsStaticBPRobot()))                                                                  \
    {                                                                                                                      \
        UE_LOG_WITH_INFO(LogTemp,                                                                                          \
                         Error,                                                                                            \
                         TEXT("[ClassName:%s] is required to be prefixed with [BP_] if this is a blueprint robot class!"), \
                         *InRobot->GetClass()->GetName());                                                                 \
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
 * This actor use #URRRobotROS2Interface as the main ROS 2 communication tool.
 * This actor has basic functionality to use with client-server, e.g. replication setting
 * - Moves kinematically with #URobotVehicleMovementComponent.
 * - Is possessed by #ARRBaseRobotROSController to be control from ROS2.
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
    TObjectPtr<USceneComponent> DefaultRoot = nullptr;

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
     * @note Experimental
     */
    FORCEINLINE bool IsDynamicRuntimeRobot() const
    {
        return (false == IsStaticBPRobot());
    }

    /**
     * @brief Static BP robot: implemented in BP, possibly inheriting from #ARBaseRobot or its children classes,
     * built from pre-designed static UE assets (StaticMesh, SkeletalMesh, Skeleton, Physics Asset, etc.)
     * @note Experimental
     */
    FORCEINLINE bool IsStaticBPRobot() const
    {
        // NOTE: (GetClass()->ClassGeneratedBy != nullptr) is also useful but WITH_EDITORONLY_DATA
        const FString className = GetClass()->GetName();
        return className.StartsWith(TEXT("BP"))             // In-Editor
               || className.StartsWith(TEXT("SKEL_BP"));    // In-Package auto prefixed [SKEL_]
    }

    /**
     * @brief Get name of an already-stored dynamic robot resource (mesh, skeleton, physics asset, etc.)
     * @param InDataType
     * @return FString
     */
    FString GetDynamicResourceName(const ERRResourceDataType InDataType) const;

    /**
     * @brief Get UE asset path of an already-stored dynamic robot resource (mesh, skeleton, physics asset, etc.)
     * @param InDataType
     * @return FString
     */
    FString GetDynamicResourceAssetPath(const ERRResourceDataType InDataType) const;

    //! Robot creation done delegate
    FOnRobotCreationDone OnRobotCreationDone;

    //! Default class to use when ROS 2 Interface is setup for robot
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "ROS 2 Interface Class"), Replicated)
    TSubclassOf<URRRobotROS2Interface> ROS2InterfaceClass = nullptr;

    /**
     * Robot's ROS 2 Interface.
     * With the client-server setup, this is created in the server and replicated to the client and initialized only in the client.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Instanced, Replicated, ReplicatedUsing = OnRep_ROS2Interface)
    TObjectPtr<URRRobotROS2Interface> ROS2Interface = nullptr;

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
    TObjectPtr<UROS2Spawnable> ROSSpawnParameters = nullptr;

    /**
     * @brief Pointer to the robot's server-owned version
     * @note Owner can't be used since non-player pawn don't have that.
     */
    UPROPERTY(VisibleAnywhere, Replicated)
    TObjectPtr<ARRBaseRobot> ServerRobot = nullptr;

    /**
     * @brief Instantiate ROS 2 Interface without initializing yet
     * @note Not uses RPC but replication since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa #ARRBaseRobotROSController::OnPossess
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     *
     */
    UFUNCTION(BlueprintCallable)
    void CreateROS2Interface();

    /**
     * @brief Initialize ROS 2 Interface. Directly call #URRRobotROS2Interface::Initialize or execute in client via #OnRep_bStartStopROS2Interface.
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
     * @brief Stop ROS 2 Interface. Directly call #URRRobotROS2Interface::DeInitialize or execute in client via #OnRep_bStartStopROS2Interface.
     * @note Not uses RPC but replication since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/5.1/en-US/InteractiveExperiences/Networking/Actors/OwningConnections)
     */
    UFUNCTION(BlueprintCallable)
    void DeInitROS2Interface();

    /**
     * @brief
     * @deprecated Please use EntityUniqueName
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString RobotUniqueName;

    /**
     * @brief Set robot unique name
     */
    void SetRobotName(const FString& InRobotName)
    {
        RobotUniqueName = InRobotName;
        EntityUniqueName = InRobotName;
    }

    //! Robot Model Name (loaded from URDF/SDF)
    //! should be same as EntityModelName in #ARRBaseActor
    //! @todo move to protected member
    UPROPERTY(VisibleAnyWhere, BlueprintReadOnly, meta = (ExposeOnSpawn = "true"), Replicated)
    FString RobotModelName;

    UFUNCTION(BlueprintCallable)
    virtual void SetRobotModelName(const FString InName)
    {
        RobotModelName = InName;
        EntityModelName = InName;
        if (ActorInfo.IsValid())
        {
            ActorInfo->EntityModelName = InName;
        }
    }

    UFUNCTION(BlueprintCallable)
    virtual FString GetRobotModelName()
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
     * @brief Add UStaticMeshComponent to #Links
     * InLinkName is used for frame name of tf.
     *
     * @param InLinkName
     * @param InMesh
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable)
    virtual bool AddLink(const FString& InLinkName, UStaticMeshComponent* InMesh);

    //! Base mesh comp, normally also as the root comp
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TObjectPtr<UMeshComponent> BaseMeshComp = nullptr;

    /**
     * @brief Set #BaseMeshComp, optionally making it the new Root, replacing #DefaultRoot
     * @param InBaseMeshComp
     * @param bInMakeAsRoot
     * @param bInDestroyDefaultRoot Whether or not destroying #DefaultRoot upon (bInMakeAsRoot == true), in which case if kept it is only to support compatibility in users' child-BP class
     */
    void SetBaseMeshComp(UMeshComponent* InBaseMeshComp, bool bInMakeAsRoot = true, bool bInDestroyDefaultRoot = true);

    /**
     * Robot Joints
     * @todo adopt to client-server.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, URRJointComponent*> Joints;

    /**
     * @brief Add URRJointComponent to #Joints and set Joint's parent and child link from name in #Links
     * InParentLinkName and InChildLinkName need to be in #Links beforehand.
     * InJointName is used for joint name of joint_states topic
     *
     * @param InParentLinkName
     * @param InChildLinkName
     * @param InJointName
     * @param InJoint
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable)
    virtual bool AddJoint(const FString& InParentLinkName,
                          const FString& InChildLinkName,
                          const FString& InJointName,
                          URRJointComponent* InJoint);

    /**
     * Initialize #Joints or not. Initial pose are set in each joint.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bInitializeJoints = true;

    UFUNCTION(BlueprintCallable)
    virtual void StartJointsInitialization();

    UFUNCTION(BlueprintCallable)
    virtual void CheckJointsInitialization();

    /**
     * @brief Set the ChildComponents Collision Enabled
     * if IsEnabled=false Set all children UPrimitiveComponents collision profile to "OverlapAll". Original profiles are saved in #OriginalCollisionProfiles.
     * if IsEnabled=false Revert all children UPrimitiveComponents collision profile to back to original.
     * @param IsEnable
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetChildComponentsCollisionEnabled(const bool IsEnable);

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
    //! [cm/s] Local target linear vel
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector TargetLinearVel = FVector::ZeroVector;

    //! [deg/s] Local target angular vel [X:Roll - Y:Pitch - Z: Yaw]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector TargetAngularVel = FVector::ZeroVector;

    /**
     * @brief Stop robot movement, resetting all vel inputs
     */
    UFUNCTION(BlueprintCallable)
    virtual void StopMovement();

    //! Main robot movement component (kinematics/diff-drive or wheels-drive comp)
    //! #MovementComponent and #RobotVehicleMoveComponent should point to same pointer.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Instanced)
    TObjectPtr<UMovementComponent> MovementComponent = nullptr;

    //! Movecomponent casted to #URobotVehicleMovementComponent for utility.
    //! This should be pointing same thing as #MovementComponent
    //! This should be set from #SetMoveComponent
    // UPROPERTY(EditAnywhere, BlueprintReadWrite, Instanced, Replicated)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Instanced, Replicated)
    TObjectPtr<URobotVehicleMovementComponent> RobotVehicleMoveComponent;

    //! Class of the main robot movement component, configurable in child class
    //! If VehicleMoveComponentClass == nullptr, it is expected that MovementComponent is set from BP or user code.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TSubclassOf<UMovementComponent> VehicleMoveComponentClass;

    UFUNCTION(BlueprintCallable)
    virtual void SetMoveComponent(UMovementComponent* InMoveComponent);

    /**
     * @brief Set linear and angular velocity to #RobotVehicleMoveComponent.
     * Calls #SetLinearVel and #SetAngularVel
     * @param InLinearVel
     * @param InAngularVel
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetVel(const FVector& InLinearVel, const FVector& InAngularVel);

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

    //! Mobile robot or not. If this is false, movecomponent=nullptr and ROS 2 odom and cmd_vel interface are disabled.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bMobileRobot = true;

    //! Control Joint by ROS2 o rnot
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bJointControl = true;

    // UI WIDGET --
    //! Enable widget or not
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 bUIWidgetEnabled : 1;

    //! UI widget component
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TObjectPtr<URRUIWidgetComponent> UIWidgetComp = nullptr;

    //! Relative pose of the UI widget from the owner robot
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform UIWidgetOffset = FTransform(FVector(0.f, 0.f, 100.f));

    //! Widget class
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Classes)
    TSubclassOf<UUserWidget> UIUserWidgetClass;

    /**
     * @brief Check whether #UIUserWidget is valid
     */
    bool CheckUIUserWidget() const;

    //! Allowed period between two successive velocity commands. After this delay, a zero speed command will be set.
    //! If this is <= 0, time out won't happen.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float CmdVelTimeout = 0.5;

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
     * @brief This method is called inside #PreInitializeComponents.
     * Custom initialization of child BP class can be done by overwritting this method.
     * @note BlueprintImplementableEvent can't return value
     */
    UFUNCTION(BlueprintImplementableEvent, BlueprintCallable)
    void BPPreInitializeComponents();

    /**
     * @brief This method is called inside #PreInitializeComponents.
     * Custom initialization of child BP class can be done by overwritting this method.
     * @note BlueprintImplementableEvent can't return value
     */
    UFUNCTION(BlueprintImplementableEvent, BlueprintCallable)
    void BPPostInitializeComponents();

    //! last time when SetVel is called.
    float LastCmdVelUpdateTime = 0;

public:
    /**
     * @brief Parse Json parameters in #ROSSpawnParameters
     * This function is called in #PreInitializeComponents
     * Please overwrite this function to parse your custom parameters
     *
     * \code{Example Implementation of Json parser:}
     * if (false == Super::InitPropertiesFromJSON())
     * {
     *     return false;
     * }

     * TSharedRef<TJsonReader<TCHAR>> jsonReader = TJsonReaderFactory<TCHAR>::Create(ROSSpawnParameters->ActorJsonConfigs);
     * TSharedPtr<FJsonObject> jsonObj = MakeShareable(new FJsonObject());
     * if (!FJsonSerializer::Deserialize(jsonReader, jsonObj) && jsonObj.IsValid())
     * {
     *     UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("Failed to deserialize json to object"));
     *     return false;
     * }

     * // Parse single value
     * bool bParam = false;
     * if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("bool_value"), bParam))
     * {
     *     UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("bool value: %d"), bParam);
     * }
     * else
     * {
     *     UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("%s [bool_value] not found in json config"));
     * }

     * // Parse array
     * const TArray<TSharedPtr<FJsonValue>>* paramArray;
     * if (!jsonObj->TryGetArrayField(TEXT("array_value"), paramArray))
     * {
     *     return false;
     * }

     * for (const auto& param : *paramArray)
     * {
     *     const TSharedPtr<FJsonObject>* jObj;
     *     if (!param->TryGetObject(jObj))
     *     {
     *         UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("%s Not an object !!"));
     *         continue;
     *     }

     *     // Parse value in array
     *     float valueInParam = 0.0;
     *     if (URRGeneralUtils::GetJsonField(*jObj, TEXT("value_in_array"), valueInParam, 0.0f))
     *     {
     *         UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("value_in_array : %f"), valueInParam);
     *     }
     *     else
     *     {
     *         UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("%s [value_in_array] not found in json config"));
     *     }
     * }
     * return true;
     * \endcode
     *
     * @return true/false
    */
    UFUNCTION(BlueprintCallable)
    virtual bool InitPropertiesFromJSON();

    /**
     * @brief This method is called inside #PreInitializeComponents.
     * Custom initialization of child BP class can be done by overwritting this method.
     * @note BlueprintImplementableEvent can't return value
     */
    UFUNCTION(BlueprintImplementableEvent, BlueprintCallable)
    void BPInitPropertiesFromJSON();

    /**
     * @brief Calls both #InitPropertiesFromJSON and #BPInitPropertiesFromJSON
     *
     */
    virtual void ConfigureMovementComponent();

    /**
     * @brief Create & init #UIWidgetComp
     */
    virtual void InitUIWidget();

    /**
     * Initialize #Joints or not. Initial pose are set in each joint.
     */
    UPROPERTY(BlueprintReadWrite)
    bool bInitializingJoints = false;

    /**
     * @brief Children UPrimitives components orignal collision profiles.
     * Retrive profiles to this variable when SetChildComponentsCollisionEnabled(false) is called
     * and this values are used when SetChildComponentsCollisionEnabled(true) are called.
     *
     */
    UPROPERTY()
    TMap<UPrimitiveComponent*, FName> OriginalCollisionProfiles;

    UFUNCTION(BlueprintCallable)
    virtual bool InitPropertiesFromJSONAll()
    {
        bool res = InitPropertiesFromJSON();
        BPInitPropertiesFromJSON();
        return res;
    };
};
