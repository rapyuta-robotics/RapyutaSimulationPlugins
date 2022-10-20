/**
 * @file RRRobotBaseVehicle.h
 * @brief Base robot vehicle class, inherited by other robot vehicle classes.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Drives/RRJointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RRBaseRobot.h"
#include "Sensors/RRROS2BaseSensorComponent.h"

// rclUE
#include "ROS2Node.h"

#include "RRRobotBaseVehicle.generated.h"

class URobotVehicleMovementComponent;

/**
 * @brief Base robot vehicle class, inherited by other robot vehicle classes.
 * - Moves kinematically with #URobotVehicleMovementComponent.
 * - Is possessed by #ARRRobotVehicleROSController to be control from ROS2.
 * You can find example at #ATurtlebotBurger.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRRobotBaseVehicle : public ARRBaseRobot
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ARRRobotBaseVehicle object
     *
     */
    ARRRobotBaseVehicle();

    /**
     * @brief Construct a new ARRRobotBaseVehicle object
     *
     * @param ObjectInitializer
     */
    ARRRobotBaseVehicle(const FObjectInitializer& ObjectInitializer);

    /**
     * @brief Initialize vehicle default
     *
     */
    void SetupDefaultVehicle();

    //! reference actor for odometry.
    //! @todo is this still necessary?
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    AActor* Map = nullptr;

    // MOVEMENT --
    //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector TargetLinearVel = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector TargetAngularVel = FVector::ZeroVector;

    //! Main robot movement component (kinematics/diff-drive or wheels-drive comp)
    UPROPERTY(VisibleAnywhere)
    UMovementComponent* MovementComponent = nullptr;

    //! Main robot vehicle movement component
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    URobotVehicleMovementComponent* RobotVehicleMoveComponent = nullptr;

    //! Class of the main robot movement component, configurable in child class
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TSubclassOf<UMovementComponent> VehicleMoveComponentClass;

    /**
     * @brief Create and Initialize #MovementComponent if #VehicleMoveComponentClass != nullptr.
     * If VehicleMoveComponentClass == nullptr, it is expected that MovementComponent is set from BP or user code.
     *
     * @return true #MovementComponent is created and initialized.
     * @return false #VehicleMoveComponentClass == nullptr.
     */
    virtual bool InitMoveComponent();

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const;

    /**
     * @brief Allows a component to replicate other subobject on the actor
     *
     */
    virtual bool ReplicateSubobjects(UActorChannel* Channel, FOutBunch* Bunch, FReplicationFlags* RepFlags) override;

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
     * @sa [Connection](https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Networking/Actors/OwningConnections) 
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
     * @sa [Connection](https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Networking/Actors/OwningConnections) 
     */
    UFUNCTION(BlueprintCallable)
    virtual void SyncServerAngularMovement(float InClientTimeStamp,
                                           const FRotator& InClientRobotRotation,
                                           const FVector& InAngularVel);

    /**
     * @brief Set linear velocity to #RobotVehicleMoveComponent in the client.
     * @note Not uses RPC since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Networking/Actors/OwningConnections) 
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetLocalLinearVel(const FVector& InLinearVel);

    /**
     * @brief Set angular velocity to #RobotVehicleMoveComponent in the client.
     * @note Not uses RPC since the robot is not always owned by the same connection with the client's PlayerController.
     * @sa [Connection](https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Networking/Actors/OwningConnections) 
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetLocalAngularVel(const FVector& InAngularVel);

protected:
    /**
     * @brief Post Initialization process of actor. Initialize #RobotVehicleMoveComponent by calling #InitMoveComponent.
     * @sa[ActorLifecycle](https://docs.unrealengine.com/4.27/en-US/ProgrammingAndScripting/ProgrammingWithCPP/UnrealArchitecture/Actors/ActorLifecycle/)
     * @sa[PostInitializeComponents](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AActor/PostInitializeComponents/)
     */
    virtual void PostInitializeComponents() override;
    /**
     * @brief This method is called inside #PostInitializeComponents.
     * Custom initialization of child class can be done by overwritting this method.
     *
     */
    virtual void ConfigureMovementComponent()
    {
    }
};
