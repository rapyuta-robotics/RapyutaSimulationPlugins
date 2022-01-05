// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Robots/Turtlebot3/Turtlebot.h"
#include "Sensors/RR2DLidarComponent.h"

#include "TurtlebotBurger.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogTurtlebotBurger, Log, All);

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotBurger : public ATurtlebot
{
    GENERATED_BODY()

public:
    ATurtlebotBurger(const FObjectInitializer& ObjectInitializer);

protected:
    virtual void OnConstruction(const FTransform& InTransform) override;
    virtual void BeginPlay() override;

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    // Called every frame
    virtual void Tick(float DeltaTime) override;

    UFUNCTION()
    void SetupBody();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* Base = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* LidarSensor = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URR2DLidarComponent* LidarComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* WheelRight = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* CasterBack = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_LidarSensor = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_WheelRight = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_CasterBack = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxForce = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UMaterial* VehicleMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UMaterial* BallMaterial = nullptr;

    UPROPERTY(VisibleAnywhere)
    uint8 bBodyComponentsCreated : 1;

    UFUNCTION()
    void SetupConstraintsAndPhysics();

    UFUNCTION()
    void SetupWheelDrives();
};
