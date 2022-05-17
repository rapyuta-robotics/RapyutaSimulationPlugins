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
// UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
// class RAPYUTASIMULATIONPLUGINS_API URRRobotLink : public USceneComponent
// {
//     GENERATED_BODY()
// public:
//     UPROPERTY(EditAnywhere, BlueprintReadWrite)
//     URRStaticMeshComponent* MeshComponent = nullptr;

//     // UPROPERTY(EditAnywhere, BlueprintReadWrite)
//     // FString Name;

//     // UPROPERTY(EditAnywhere, BlueprintReadWrite)
//     // FString ParentFrameName;

//     UPROPERTY(EditAnywhere, BlueprintReadWrite)
//     FVector PositionOffset = FVector::ZeroVector;

//     UPROPERTY(EditAnywhere, BlueprintReadWrite)
//     FQuat RotationOffset = FQuat::Identity;

//     URRRobotLink()
//     {
//         MeshComponent = CreateDefaultSubobject<URRStaticMeshComponent>(TEXT("Link"));
//         MeshComponent->SetupAttachment(this);
//     }

//     FTransform GetRelativeTransfomToParent() const
//     {
//         return FTransform(RotationOffset, PositionOffset);
//     }
// };

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
    virtual void SetPose(const FVector& InPosition, const FRotator& InOrientation);

    UFUNCTION(BlueprintCallable)
    virtual void SetPoseWithArray(const TArray<float>& InPose);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelocity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelocity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Position;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator Orientation;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRRobotLink* ParentLink;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRRobotLink* ChildLink;

    //! Joint Position Offset from parent link
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PositionOffset = FVector::ZeroVector;

    //! Joint Rotational Offset from parent link
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FQuat RotationOffset = FQuat::Identity;

    //! Linear Degrees Of Freedom
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 LinearDOF = 0;

    //! Rotational Degrees Of Freedom
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 RotationalDOF = 1;
};
