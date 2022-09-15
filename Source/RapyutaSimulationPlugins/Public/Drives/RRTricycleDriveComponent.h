/**
 * @file RRSkeletalRobotDiffDriveComponent.h
 * @brief Skeletal Robot DiffDriveComponent Drive component class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "PhysicsEngine/ConstraintInstance.h"

// RapyutaSimulationPlugins
#include "Drives/DifferentialDriveComponent.h"
//#include "Robot/RRRobotCommon.h"

#include "RRTricycleDriveComponent.generated.h"

/**
 * @brief Skeletal Robot DiffDriveComponent Drive component class.
 * This inherits from UPawnMovementComponent to:
 * + Allow itself to act as a kinematic movement comp through UpdatedComponent, if required at runtime.
 * + Utilize UPawnMovementComponent::Velocity to store [cmd_vel]'s value, also collectively used for kinematic movement.
 * During dynamics-only mode, UpdatedComponent could be disabled.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRTricycleDriveComponent : public UDifferentialDriveComponent
{
    GENERATED_BODY()
public:
    URRTricycleDriveComponent();
    URRTricycleDriveComponent(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());


    
    UFUNCTION(BlueprintCallable)
    void Setup();
    
    virtual void UpdateMovement(float DeltaTime) override;

    UPROPERTY()
    USkeletalMeshComponent* SkeletalMeshComponent;

    UPROPERTY()
    FVector VelocityCurrent = FVector::ZeroVector;
    
    UPROPERTY()
    TMap<FString, float> JointsStatesCurrent;

    UPROPERTY()
    float MaxEngineTorque = 300.0f;
    
    float SteerInputCurrent = 0.0f;
};
