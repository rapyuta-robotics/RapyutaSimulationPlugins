// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Kismet/GameplayStatics.h"

#include "Msgs/ROS2EntityStateMsg.h"
#include <Msgs/ROS2OdometryMsg.h>

#include "RobotVehicleMovementComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RCLUE_API URobotVehicleMovementComponent : public UPawnMovementComponent
{
    GENERATED_BODY()

private:
    UPROPERTY(Transient)
    FVector DesiredMovement;

    UPROPERTY(Transient)
    FQuat DesiredRotation;

    // for manual computation of linear and angular velocities for odometry 
    FQuat LastOrientation;
    FVector LastLocation;

    // For Elevator management
    AActor* MovingPlatform = nullptr;    // The platform below the robot
    FVector LastPlatformLocation;
    FQuat LastPlatformRotation;

    // For slopes, complex floors, free fall
    float MinDistanceToFloor =
        0.f;    // Z distance between the robot root location and the floor, used when less than 3 contact points are defined
    const float FallingSpeed = 100.;           // How much the robot falls if no floor beneath ( FallingSpeed * DeltaTime )
    TArray<USceneComponent*> ContactPoints;    // List all scene components on the pawn. that have the tag "ContactPoint"

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Velocity)
    FVector AngularVelocity;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSOdometry OdomData;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSEntityState EntityState;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId = TEXT("");

    UPROPERTY(EditAnywhere)
    FTransform InitialTransform;

    // For slopes, complex floors, free fall
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RayOffsetUp = 10.;    // Ray start Z offset. Value must be > possible penetration of objects in contact point, in one tick

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RayOffsetDown = 20.;  // Ray end Z offset
    // Rays go from ContactPoint+RayOffsetUp to ContactPoint-RayOffsetDown

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool FollowFloor = true;    // to activate/deactivate floor checks to stick the robot on its surface below

    UFUNCTION(BlueprintCallable)
    FTransform GetOdomTF();

    UFUNCTION(BlueprintCallable)
    void UpdateEntityState( FString AgentName );

    UFUNCTION(BlueprintCallable)
    virtual void InitMovementComponent();

    UFUNCTION(BlueprintCallable)
    void SetMovingPlatform(AActor* platform);

    UFUNCTION(BlueprintCallable)
    bool IsOnMovingPlatform();

    UFUNCTION(BlueprintCallable)
    void RemoveMovingPlatform();

protected:
    virtual void InitOdom();
    virtual void UpdateMovement(float DeltaTime);
    virtual void UpdateOdom(float DeltaTime);
    bool IsOdomInitialized = false;

public:
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
