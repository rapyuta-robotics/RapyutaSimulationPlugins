// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

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

//! temporary impl. should be merged with RobotImporter later.
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
    // Called every frame
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

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
};
