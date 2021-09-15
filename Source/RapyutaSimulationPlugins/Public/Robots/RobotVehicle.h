// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Drives/RobotVehicleMovementComponent.h"

#include "RobotVehicle.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public APawn
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	URobotVehicleMovementComponent *MoveComponent;

public:
	ARobotVehicle(const FObjectInitializer& ObjectInitializer);

	virtual void Tick(float DeltaSeconds) override;

	UFUNCTION(BlueprintCallable)
	virtual void SetLinearVel(FVector velocity);

	UFUNCTION(BlueprintCallable)
	virtual void SetAngularVel(FVector velocity);

protected:

	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
};
