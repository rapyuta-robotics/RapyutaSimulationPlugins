// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"
#include "Engine/TargetPoint.h"
#include "GameFramework/Pawn.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "ROS2Node.h"

#include "RobotVehicle.generated.h"

class URobotVehicleMovementComponent;
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public APawn
{
    GENERATED_BODY()

public:
    ARobotVehicle();
    ARobotVehicle(const FObjectInitializer& ObjectInitializer);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"))
    FString RobotUniqueName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"))
    AActor* Map = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URobotVehicleMovementComponent* RobotVehicleMoveComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URobotVehicleMovementComponent> VehicleMoveComponentClass;

    bool InitSensors(AROS2Node* InROS2Node);
    void Initialize();

    UFUNCTION(BlueprintCallable)
    virtual void SetLinearVel(const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable)
    virtual void SetAngularVel(const FVector& InAngularVelocity);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelRadius = 1.f;

    // todo get data from links
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelSeparationHalf = 1.f;

protected:
    virtual void PostInitializeComponents() override;
    virtual void ConfigureVehicleMoveComponent()
    {
    }
};
