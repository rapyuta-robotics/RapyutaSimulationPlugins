// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

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

UENUM(BlueprintType)
enum class EOdomSource : uint8
{
    WORLD UMETA(DisplayName = "World"),
    ENCODER UMETA(DisplayName = "Encoder")
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RCLUE_API URobotVehicleMovementComponent : public UPawnMovementComponent
{
    GENERATED_BODY()

private:
    // For Elevator management
    UPROPERTY(VisibleAnywhere)
    AActor* MovingPlatform = nullptr;    // The platform below the robot

    UPROPERTY(VisibleAnywhere)
    FVector LastPlatformLocation = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere)
    FQuat LastPlatformRotation = FQuat::Identity;

    UPROPERTY(VisibleAnywhere)
    TArray<USceneComponent*> ContactPoints;    // List all scene components on the pawn. that have the tag "ContactPoint"

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Velocity)
    FVector AngularVelocity = FVector::ZeroVector;
    FVector DesiredMovement = FVector::ZeroVector;

    UPROPERTY(Transient)
    FQuat DesiredRotation = FQuat::Identity;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSOdometry OdomData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("odom");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId = TEXT("base_footprint");

    void SetFrameIds(const FString& InFrameId, const FString& InChildFrameId);

    UPROPERTY(EditAnywhere)
    FTransform InitialTransform = FTransform::Identity;

    // For slopes, complex floors, free fall
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RayOffsetUp = 10.f;    // Ray start Z offset. Value must be > possible penetration of objects in contact point, in one tick

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RayOffsetDown = 20.f;    // Ray end Z offset
    // Rays go from ContactPoint+RayOffsetUp to ContactPoint-RayOffsetDown

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bFollowPlatform = true;    // to activate/deactivate floor checks to stick the robot on its surface below

    UFUNCTION(BlueprintCallable)
    FTransform GetOdomTF() const;

    UFUNCTION(BlueprintCallable)
    virtual void Initialize();

    UFUNCTION(BlueprintCallable)
    virtual void InitOdom();

    UPROPERTY()
    int8 InversionFactor = 1;

    UPROPERTY(EditAnywhere)
    EOdomSource OdomSource = EOdomSource::WORLD;

    UFUNCTION(BlueprintCallable)
    virtual void InitMovementComponent();

    UFUNCTION(BlueprintCallable)
    void SetMovingPlatform(AActor* platform);

    UFUNCTION(BlueprintCallable)
    bool IsOnMovingPlatform();

    UFUNCTION(BlueprintCallable)
    void RemoveMovingPlatform();

    // For slopes, complex floors, free fall
    UPROPERTY(VisibleAnywhere)
    float MinDistanceToFloor =
        0.f;    // Z distance between the robot root location and the floor, used when less than 3 contact points are defined

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float FallingSpeed = 100.f;    // How much the robot falls if no floor beneath ( FallingSpeed * DeltaTime )

protected:
    virtual void UpdateMovement(float InDeltaTime);
    virtual void UpdateOdom(float InDeltaTime);
    bool IsOdomInitialized = false;

    UPROPERTY()
    FTransform PreviousTransform = FTransform::Identity;

    std::random_device Rng;
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

    UPROPERTY(EditAnywhere, Category = "Noise")
    bool WithNoise = true;

public:
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
