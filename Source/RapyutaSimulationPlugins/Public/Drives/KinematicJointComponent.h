// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Drives/JointComponent.h"

#include "KinematicJointComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UKinematicJointComponent : public UJointComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    UKinematicJointComponent();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    
    virtual void SetPose(const FVector& InPosition, const FRotator& InOrientation) override;
    
    virtual void UpdatePose();
};
