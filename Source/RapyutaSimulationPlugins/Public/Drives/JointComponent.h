/**
 * @file JointComponent.h
 * @brief Base Joint component class which is used as part of #ARobotVehicle.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Components/ActorComponent.h"
#include "Core/RRStaticMeshComponent.h"
#include "CoreMinimal.h"

#include "JointComponent.generated.h"

UENUM(BlueprintType)
enum class EJointControlType : uint8
{
    POSITION UMETA(DisplayName = "Position"),
    VELOCITY UMETA(DisplayName = "Velocity"),
    EFFORT UMETA(DisplayName = "Effort")
};

/**
 * @brief Base Joints class. Other sensors class should inherit from this class.
 * temporary implementation of joints. should be merged with RobotImporter later.
 *
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UJointComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    UJointComponent();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;

public:

    UFUNCTION(BlueprintCallable)
    virtual void SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity);

    UFUNCTION(BlueprintCallable)
    virtual void SetVelocityWithArray(const TArray<float>& InVelocity);

    UFUNCTION(BlueprintCallable)
    virtual void SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation);

    UFUNCTION(BlueprintCallable)
    virtual void SetPoseTargetWithArray(const TArray<float>& InPose);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PositionTarget = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationTarget = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* ParentLink;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* ChildLink;

    //! Linear Degrees Of Freedom
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 LinearDOF = 0;

    //! Rotational Degrees Of Freedom
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 RotationalDOF = 1;

    //! Pose Limitations[cm]
    //! @todo is it possible to set inf?
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PositionMax = FVector(1000, 1000, 1000);

    //! Pose Limitations[cm]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PositionMin = FVector(-1000, -1000, -1000);

    //! Orientation Limitations[deg]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationMax = FRotator(180, 180, 180);

    //! Orientation Limitations[deg]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationMin = FRotator(-180, -180, -180);

    //! Linear Velocity Limitations[cm/s]
    //! @todo is it possible to set inf?
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelMax = FVector(1000, 1000, 1000);

    //! Linear Velocity Limitations[cm/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelMin = FVector(-1000, -1000, -1000);

    //! Angular Velocity Limitations[deg/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelMax = FVector(180, 180, 180);

    //! Angular Velocity Limitations[deg/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelMin = FVector(-180, -180, -180);

};
