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
#include "GameFramework/Actor.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Kismet/GameplayStatics.h"

// rclUE
#include "Msgs/ROS2OdometryMsg.h"

#include "RobotVehicleMovementComponent.generated.h"

/**
 * @brief Type of odometry frame origin.
 * World provide odometry from world origin and Encoder provide odometry from initial pose.
 */
UENUM(BlueprintType)
enum class EOdomSource : uint8
{
    WORLD UMETA(DisplayName = "World"),
    ENCODER UMETA(DisplayName = "Encoder")
};

/**
 * @brief Base Robot vehicle movement class which is used as part of #ARobotVehicle. 
 * Robot moves based on the Velocity(member of UMovementComponent) and #AngularVelocity without considering physics.
 * If Robot bumped into something, try to slide along it.
 * 
 * If #bFollowPlatform is true, robot will follow the pawn movement under the robot which has been defined as the #MovingPlatform (e.g. elevators), it will also adapt its pose to the floor surface configuration (e.g. slopes)

 * 
 * Publish odometry from world origin or initial pose.
 *
 * @sa [UPawnMovementComponent](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/UPawnMovementComponent/)
 * 
 * @todo Support 3D movement.
 * @todo Expose odom covariance parameter.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RCLUE_API URobotVehicleMovementComponent : public UPawnMovementComponent
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

    //! List all scene components on the pawn. that have the tag "ContactPoint". This is used to adapt the robot pose based on the floor surface configuration.

    UPROPERTY(VisibleAnywhere)
    TArray<USceneComponent*> ContactPoints;    

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Velocity)
    FVector AngularVelocity = FVector::ZeroVector;
 
    //! Desired position calculated from deltatime and UpdatedComponent::ComponentVelocity
    //! @sa [UpdatedComponent](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/UpdatedComponent/)
    UPROPERTY(Transient)
    FVector DesiredMovement = FVector::ZeroVector;

    //! Desired orientation calculated from deltatime and #AngularVelocity
    UPROPERTY(Transient)
    FQuat DesiredRotation = FQuat::Identity;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSOdometry OdomData;

    //! Frame id of odometry
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("odom");

    //! Child frame id of odometry
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId = TEXT("base_footprint");

    /**
     * @brief Set the Frame Id and child frame id of odometry
     * 
     * @param InFrameId 
     * @param InChildFrameId 
     */
    void SetFrameIds(const FString& InFrameId, const FString& InChildFrameId);

    UPROPERTY(EditAnywhere)
    FTransform InitialTransform = FTransform::Identity;

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
    bool bFollowPlatform = true;   

    UFUNCTION(BlueprintCallable)
    FTransform GetOdomTF() const;

    /**
     * @brief Initialize noise and odometry.
     * 
     */
    UFUNCTION(BlueprintCallable)
    virtual void Initialize();

    UFUNCTION(BlueprintCallable)
    virtual void InitOdom();

    //! @todo is this necessary?
    UPROPERTY()
    int8 InversionFactor = 1;

    UPROPERTY(EditAnywhere)
    EOdomSource OdomSource = EOdomSource::WORLD;

    /**
     * @brief Call #InitOdom, Calculate #MinDistanceToFloor, and
     * Add all actors in PawnOwner which has tag "ContactPoints" to #ContactPoints
     * 
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitMovementComponent();

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

protected:
    /**
     * @brief Move actor by using SafeMoveUpdatedComponent and SlideAlongSurface.
     * Calculate #DesiredMovement and #DesiredRotation from deltatime, UpdatedComponent and #AngularVelocity.
     * 
     * If #bFollowPlatform is true, robot will follow the pawn movement under the robot with tag "ContactPoints".
     * Please check the .cpp file for detailed implementation to follow platform.
     * 
     *
     * @param InDeltaTime 
     * @sa [SafeMoveUpdatedComponent](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/SafeMoveUpdatedComponent/1/)
     * @sa [SlideAlongSurface](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/SlideAlongSurface/)
     * @sa [UpdatedComponent](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/UpdatedComponent/)
     */
    virtual void UpdateMovement(float InDeltaTime);

    /**
     * @brief Update odom.
     * Noise is integral of gaussian noise.
     * @param InDeltaTime 
     */
    virtual void UpdateOdom(float InDeltaTime);

    bool IsOdomInitialized = false;

    UPROPERTY()
    FTransform PreviousTransform = FTransform::Identity;

    //! C++11 RNG for odometry noise
    std::random_device Rng;

    //! C++11 RNG for odometry noise
    std::mt19937 Gen = std::mt19937{Rng()};
    
    std::normal_distribution<> GaussianRNGPosition;
    std::normal_distribution<> GaussianRNGRotation;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseMeanPos = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseVariancePos = 0.01f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseMeanRot = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float NoiseVarianceRot = 0.05f;

    //! Add noise or not
    UPROPERTY(EditAnywhere, Category = "Noise")
    bool WithNoise = true;

public:
    /**
     * @brief Call #UpdateMovement, #UpdateOdom, and UpdateComponentVelocity
     * 
     * @param DeltaTime 
     * @param TickType 
     * @param ThisTickFunction 
     *
     * @sa [UpdateComponentVelocity](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/UMovementComponent/UpdateComponentVelocity/)
     */
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
