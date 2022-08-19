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
    UFUNCTION(BlueprintCallable)
    void Setup();
    
    virtual void UpdateMovement(float DeltaTime) override;
    void UpdateFromCurrent();

    //void SetBoneTransformByName(FName BoneName, const FTransform& InTransform, EBoneSpaces::Type BoneSpace);

   // UPROPERTY()
   // ARRSkeletalRobot* SkeletalRobot = nullptr;
    
    //USkeletalMeshComponent* SkeletalMeshComponent;
    //UPoseableMeshComponent* PoseableMeshComponent;
    
    FConstraintInstance* DriveWheelCI = nullptr;
    FConstraintInstance* CasterWheelCI = nullptr;

    UPROPERTY()
    FVector VelocityCurrent = FVector::ZeroVector;
    
    UPROPERTY()
    FVector AngularVelocityCurrent = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString DriveWheelName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString CasterWheelName;

    int32 DriveWheelBoneIndex = INDEX_NONE;
};
