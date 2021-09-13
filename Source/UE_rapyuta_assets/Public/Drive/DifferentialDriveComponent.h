// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Drive/RobotVehicleMovementComponent.h"

#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include "DifferentialDriveComponent.generated.h"

/**
 * 
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UE_RAPYUTA_ASSETS_API UDifferentialDriveComponent : public URobotVehicleMovementComponent
{
	GENERATED_BODY()

public:
	UDifferentialDriveComponent();
	virtual void UpdateMovement(float DeltaTime) override;
	
	UFUNCTION(BlueprintCallable)
	void SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight);

	virtual void InitMovementComponent() override;

	UFUNCTION(BlueprintCallable)
	void SetPerimeter();

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* WheelLeft;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* WheelRight;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float WheelRadius = 1.0f;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float WheelSeparationHalf = 1.0f; //todo get data from links

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float MaxForce = 1000; //todo get data from physics constraints

private:
	float WheelPerimeter = 6.28f;;


	
};
