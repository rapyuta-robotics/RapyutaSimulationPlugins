// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"
#include "Engine/TargetPoint.h"
#include "GameFramework/Pawn.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Sensors/RRROS2BaseSensorComponent.h"

// rclUE
#include "Msgs/ROS2JointStateMsg.h"
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

    // Actually Object's Name is also unique as noted by UE, but we just do not want to rely on it.
    // Instead, WE USE [RobotUniqueName] TO MAKE THE ROBOT ID CONTROL MORE INDPENDENT of UE INTERNAL NAME HANDLING.
    // Reasons:
    // + An Actor's Name could get updated as its Label is updated
    // + In pending-kill state, GetName() goes to [None]
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
    void SetupDefault();

    UFUNCTION(BlueprintCallable)
    virtual void SetLinearVel(const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable)
    virtual void SetAngularVel(const FVector& InAngularVelocity);

    UFUNCTION()
    virtual FROSJointState GetJointStates()
    {
        checkNoEntry();
        return FROSJointState();
    }

protected:
    virtual void PostInitializeComponents() override;
    virtual void ConfigureVehicleMoveComponent()
    {
    }
};
