// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// System
#include <random>

// UE
#include "CoreMinimal.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Kismet/GameplayStatics.h"

// rclUE
#include "Msgs/ROS2OdometryMsg.h"

#include "RobotVehicleMovementComponent.generated.h"

UENUM(BlueprintType)
enum class EOdomSource : uint8
{
    WORLD UMETA(DisplayName="World"),
    ENCODER UMETA(DisplayName = "Encoder")
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RCLUE_API URobotVehicleMovementComponent : public UPawnMovementComponent
{
    GENERATED_BODY()

private:
    UPROPERTY(Transient)
    FVector DesiredMovement = FVector::ZeroVector;

    UPROPERTY(Transient)
    FQuat DesiredRotation = FQuat::Identity;

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Velocity)
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSOdometry OdomData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("odom");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId =  TEXT("base_footprint");

    void SetFrameIds(const FString& InFrameId, const FString& InChildFrameId);

    UPROPERTY(EditAnywhere)
    FTransform InitialTransform = FTransform::Identity;

    UFUNCTION(BlueprintCallable)
    FTransform GetOdomTF() const;

    UFUNCTION(BlueprintCallable)
    virtual void Initialize();

    UFUNCTION(BlueprintCallable)
    virtual void InitOdom();

    UPROPERTY()
    int8 InversionFactor = 1;

    UPROPERTY(EditAnywhere)
    EOdomSource OdomSource =  EOdomSource::WORLD;

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
