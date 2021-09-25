// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Kismet/GameplayStatics.h"

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

    AActor* MovingPlatform = nullptr;
    FVector LastPlatformLocation;
    FQuat LastPlatformRotation;

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Velocity)
    FVector AngularVelocity;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    FROSOdometry OdomData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FVector> FloorContactPoints;

    UPROPERTY(EditAnywhere)
    FTransform InitialTransform;

    UFUNCTION(BlueprintCallable)
    FTransform GetOdomTF();

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
