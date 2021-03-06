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
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"))
    AActor* Map = nullptr;

    // KINEMATIC MOVEMENT --
    //
    //! Main robot vehicle move component
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URobotVehicleMovementComponent* RobotVehicleMoveComponent = nullptr;

    //! Class of the main robot vehicle move component, configurable in child class
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URobotVehicleMovementComponent> VehicleMoveComponentClass;

    /**
     * @brief Initialize #RobotVehicleMoveComponent
     *
     * @return true
     * @return false
     */
    virtual bool InitMoveComponent();

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
     * @brief Set velocity to #RobotVehicleMoveComponent
     *
     * @param InLinearVelocity
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetLinearVel(const FVector& InLinearVelocity);

    /**
     * @brief Set angular velocity to #RobotVehicleMoveComponent
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetAngularVel(const FVector& InAngularVelocity);

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
    virtual void ConfigureVehicleMoveComponent()
    {
    }
};
