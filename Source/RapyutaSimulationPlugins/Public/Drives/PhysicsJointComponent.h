// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "Drives/JointComponent.h"

#include "PhysicsJointComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UPhysicsJointComponent : public UJointComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    UPhysicsJointComponent();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;

public:
    virtual void SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity) override;

    virtual void SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation) override;

    UFUNCTION(BlueprintCallable)
    virtual void SetJoint();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Constraint = nullptr;
};
