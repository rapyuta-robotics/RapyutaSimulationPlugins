// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Kismet/GameplayStatics.h"

#include <Msgs/ROS2OdometryMsg.h>

#include <random>

#include "RobotVehicleMovementComponent.generated.h"

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
    URobotVehicleMovementComponent();

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Velocity)
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSOdometry OdomData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId = TEXT("");

    UPROPERTY(EditAnywhere)
    FTransform InitialTransform = FTransform::Identity;

    UFUNCTION(BlueprintCallable)
    FTransform GetOdomTF();

    UFUNCTION(BlueprintCallable)
    virtual void Initialize();

    UPROPERTY()
    int InversionFactor = 1;

protected:
    virtual void BeginPlay() override;
    virtual void InitOdom();
    virtual void UpdateMovement(float DeltaTime);
    virtual void UpdateOdom(float DeltaTime);
    bool IsOdomInitialized = false;

    UPROPERTY()
    FTransform PreviousTransform = FTransform::Identity;

    std::random_device Rng;
    std::mt19937 Gen;
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
