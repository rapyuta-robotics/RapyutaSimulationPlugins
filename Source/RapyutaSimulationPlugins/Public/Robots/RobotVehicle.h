/**
 * @file RobotVehicle.h
 * @brief Base RobotVehicle class. Other robot class should inherit from this class. Example is #ATurtlebotBurger.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"
#include "Engine/TargetPoint.h"
#include "GameFramework/Pawn.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Drives/JointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Sensors/RRROS2BaseSensorComponent.h"

// rclUE
#include "ROS2Node.h"

#include "RobotVehicle.generated.h"

class URobotVehicleMovementComponent;

/**
 * @brief Base RobotVehicle class. Other robot class should inherit from this class.
 * This actor moves with #URobotVehicleMovementComponent.
 * This actor is possessed by #ARRRobotVehicleROSController to be control from ROS2.
 * You can find example at #ATurtlebotBurger.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public ARRBaseActor
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ARobotVehicle object
     *
     */
    ARobotVehicle();

    /**
     * @brief Construct a new ARobotVehicle object
     *
     * @param ObjectInitializer
     */
    ARobotVehicle(const FObjectInitializer& ObjectInitializer);

    /**
     * @brief
     * Actually Object's Name is also unique as noted by UE, but we just do not want to rely on it.
     * Instead, WE USE [RobotUniqueName] TO MAKE THE ROBOT ID CONTROL MORE INDPENDENT of UE INTERNAL NAME HANDLING.
     * Reasons:
     * + An Actor's Name could get updated as its Label is updated
     * + In pending-kill state, GetName() goes to [None]
     *
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"))
    FString RobotUniqueName;

    //! reference actor for odometry.
    //! @todo is this still necessary?
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"))
    AActor* Map = nullptr;

    //! Robot Mesh
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URobotVehicleMovementComponent* RobotVehicleMoveComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URobotVehicleMovementComponent> VehicleMoveComponentClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, UStaticMeshComponent*> Links;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, UJointComponent*> Joints;

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
     * @brief Initialize #RobotVehicleMoveComponent
     *
     * @return true
     * @return false
     */
    virtual bool InitMoveComponent();

    /**
     * @brief Initialize #SkeletalMeshComp.
     *
     */
    void SetupDefault();

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

    // UFUNCTION(BlueprintCallable)
    virtual void SetJointState(const TMap<FString, TArray<float>>& InJointState, EJointControlType InJointControlType);

protected:
    /**
     * @brief Post Initialization process of actor. Initialize #RobotVehicleMoveComponent by calling #InitMoveComponent.
     * @sa
     * [ActorLifecycle](https://docs.unrealengine.com/4.27/en-US/ProgrammingAndScripting/ProgrammingWithCPP/UnrealArchitecture/Actors/ActorLifecycle/)
     * @sa
     * [PostInitializeComponents](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AActor/PostInitializeComponents/)
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
