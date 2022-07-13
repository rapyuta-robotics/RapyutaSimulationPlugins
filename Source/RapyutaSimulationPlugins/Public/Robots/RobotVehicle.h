// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"
#include "Engine/TargetPoint.h"
#include "GameFramework/Pawn.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Sensors/RRROS2BaseSensorComponent.h"

// rclUE
#include "ROS2Node.h"

#include "RobotVehicle.generated.h"

class URobotVehicleMovementComponent;
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public ARRBaseActor
{
    GENERATED_BODY()

public:
    ARobotVehicle();
    ARobotVehicle(const FObjectInitializer& ObjectInitializer);

    // Actually Object's Name is also unique as noted by UE, but we just do not want to rely on it.
    // Instead, WE USE [RobotUniqueName] TO MAKE THE ROBOT ID CONTROL MORE INDPENDENT of UE INTERNAL NAME HANDLING.
    // Reasons:
    // + An Actor's Name could get updated as its Label is updated
    // + In pending-kill state, GetName() goes to [None]
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString RobotUniqueName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    AActor* Map = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    URobotVehicleMovementComponent* RobotVehicleMoveComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TSubclassOf<URobotVehicleMovementComponent> VehicleMoveComponentClass;

    bool InitSensors(AROS2Node* InROS2Node);
    virtual bool InitMoveComponent();
    void SetupDefault();

    UFUNCTION(BlueprintCallable)
    virtual void SetLinearVel(const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable)
    virtual void SetAngularVel(const FVector& InAngularVelocity);

    UFUNCTION(BlueprintCallable, Server, Reliable)
    virtual void SetServerLinearVel(float TimeStamp, const FVector& InPosition, const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable, Server, Reliable)
    virtual void SetServerAngularVel(float TimeStamp,const FRotator& InRotation, const FVector& InAngularVelocity);

    UFUNCTION(BlueprintCallable, Client, Reliable)
    virtual void SetClientLinearVel(const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable, Client, Reliable)
    virtual void SetClientAngularVel(const FVector& InAngularVelocity);

protected:
    virtual void PostInitializeComponents() override;
    virtual void ConfigureVehicleMoveComponent()
    {
    }
};
