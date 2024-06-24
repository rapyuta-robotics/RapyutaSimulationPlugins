/**
 * @file RobotVehicleMovementComponent.h
 * @brief Base Robot vehicle movement class which is used as part of #ARobotVehicle.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// System
#include <random>

// UE
#include "CoreMinimal.h"
#include "GameFramework/PawnMovementComponent.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseOdomComponent.h"

//Thirdparty
#include "filter.hpp"

#include "RobotVehicleMovementComponent.generated.h"

class ARRBaseRobot;
class URRFloatingMovementComponent;

/**
 * @brief Base Robot vehicle movement class which is used as part of #ARobotVehicle.
 * Robot moves based on the Velocity(member of UMovementComponent) and #AngularVelocity without considering physics.
 * If Robot bumped into something, try to slide along it.
 *
 * If #bAdaptToSurfaceBelow is true, robot will follow the pawn movement under the robot which has been defined as the
 #MovingPlatform (e.g. elevators), it will also adapt its pose to the floor surface configuration (e.g. slopes)

 *
 * Publish odometry from world origin or initial pose.
 *
 * @sa [UPawnMovementComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UPawnMovementComponent/)
 *
 * @todo Support 3D movement.
 * @todo Expose odom covariance parameter.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URobotVehicleMovementComponent : public UPawnMovementComponent
{
    GENERATED_BODY()

private:
    // For Elevator management
    //! The platform below the robot, e.g. elevator.
    UPROPERTY(VisibleAnywhere)
    AActor* MovingPlatform = nullptr;

    UPROPERTY(VisibleAnywhere)
    FVector LastPlatformLocation = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere)
    FQuat LastPlatformRotation = FQuat::Identity;

    //! List all scene components on the pawn. that have the tag "ContactPoint". This is used to adapt the robot pose based on the
    //! floor surface configuration.

    UPROPERTY(VisibleAnywhere)
    TArray<USceneComponent*> ContactPoints;

public:
    UPROPERTY(VisibleAnywhere, Replicated)
    ARRBaseRobot* OwnerVehicle = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TObjectPtr<URRFloatingMovementComponent> AIMovementComp = nullptr;

    /**
     * @brief Init #AIMovementComp, which drives the robot kinematically using UE AI Navigation
     */
    UFUNCTION(BlueprintCallable)
    void InitAIMovementComp();

    /**
     * @brief Assign a #USceneComponent as #UpdatedComponent
     * @param InNewUpdatedComponent
     * @sa
     * [SetUpdatedComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/SetUpdatedComponent)
     */
    virtual void SetUpdatedComponent(USceneComponent* InNewUpdatedComponent) override;

    //! [deg/s] AngularVelocity control input for [UpdatedComponent]
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Velocity)
    FVector AngularVelocity = FVector::ZeroVector;

    //! Desired position calculated from deltatime and UpdatedComponent::ComponentVelocity
    //! @sa
    //! [UpdatedComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/UpdatedComponent/)
    UPROPERTY(Transient)
    FVector DesiredMovement = FVector::ZeroVector;

    //! Desired orientation calculated from deltatime and #AngularVelocity
    UPROPERTY(Transient)
    FQuat DesiredRotation = FQuat::Identity;

    // For slopes, complex floors, free fall

    //! Ray start Z offset. Value must be > possible penetration of objects in contact point, in one tick
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RayOffsetUp = 10.f;

    //! Ray end Z offset.
    //! Rays go from ContactPoint+RayOffsetUp to ContactPoint-RayOffsetDown
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RayOffsetDown = 20.f;

    //! to activate/deactivate floor checks to stick the robot on its surface below
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bAdaptToSurfaceBelow = true;

    /**
     * @brief Initialize noise and odometry.
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void Initialize();

    /**
     * @brief Initialize Velocity filters.
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitVelFilters();

    //! @todo is this necessary?
    UPROPERTY()
    int8 InversionFactor = 1;

    /**
     * @brief Call #InitOdom, Calculate #MinDistanceToFloor, and
     * Add all actors in PawnOwner which has tag "ContactPoints" to #ContactPoints
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitData();

    UFUNCTION(BlueprintCallable)
    void SetMovingPlatform(AActor* platform);

    UFUNCTION(BlueprintCallable)
    bool IsOnMovingPlatform();

    UFUNCTION(BlueprintCallable)
    void RemoveMovingPlatform();

    // For slopes, complex floors, free fall

    //! [cm] Z distance between the robot root location and the floor, used when less than 3 contact points are defined
    UPROPERTY(VisibleAnywhere)
    float MinDistanceToFloor = 0.f;

    //![cm/s] How much the robot falls if no floor beneath ( FallingSpeed * DeltaTime )
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float FallingSpeed = 100.f;

    //! Odometry source
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRBaseOdomComponent* OdomComponent = nullptr;

    //! Low Pass filter Time constant of linear velocity commands
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelFilterTau = FVector::ZeroVector;

    //! Low Pass filter Time constant of angular velocity commands
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngVelFilterTau = FVector::ZeroVector;

protected:
    virtual bool IsSupportedForNetworking() const override
    {
        return true;
    }

    /**
     * @brief Call #UpdateMovement, and UpdateComponentVelocity
     *
     * @param DeltaTime
     * @param TickType
     * @param ThisTickFunction
     *
     * @sa
     * [UpdateComponentVelocity](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/UpdateComponentVelocity/)
     */
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief Move actor by using SafeMoveUpdatedComponent and SlideAlongSurface.
     * Calculate #DesiredMovement and #DesiredRotation from deltatime, UpdatedComponent and #AngularVelocity.
     *
     * If #bAdaptToSurfaceBelow is true, robot will follow the pawn movement under the robot with tag "ContactPoints".
     * Please check the .cpp file for detailed implementation to follow platform.
     *
     *
     * @param InDeltaTime
     * @sa
     * [SafeMoveUpdatedComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/SafeMoveUpdatedComponent/1/)
     * @sa
     * [SlideAlongSurface](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/SlideAlongSurface/)
     * @sa
     * [UpdatedComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/UpdatedComponent/)
     */
    virtual void UpdateMovement(float InDeltaTime);

    //! internal property used to log throttle.
    UPROPERTY()
    float LogLastHit = 0.f;

    //! Linear Velocity Filter
    TStaticArray<FirstOrderSystem, 3> LinearVelFilter;

    //! Angular Velocity Filter
    TStaticArray<FirstOrderSystem, 3> AngVelFilter;
};
