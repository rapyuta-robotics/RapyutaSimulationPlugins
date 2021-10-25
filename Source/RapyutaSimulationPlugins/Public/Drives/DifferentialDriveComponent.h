// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include <random>

#include "CoreMinimal.h"
#include "Drives/RobotVehicleMovementComponent.h"

#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include "DifferentialDriveComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogDifferentialDriveComponent, Log, All);

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class RAPYUTASIMULATIONPLUGINS_API UDifferentialDriveComponent : public URobotVehicleMovementComponent
{
	GENERATED_BODY()

public:
	UDifferentialDriveComponent();
	virtual void UpdateMovement(float DeltaTime) override;
	virtual void UpdateOdom(float DeltaTime) override;
	
	UFUNCTION(BlueprintCallable)
	void SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight);

	virtual void Initialize() override;

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
	float WheelPerimeter = 6.28f;

	float PoseEncoderX = 0;
	float PoseEncoderY = 0;
	float PoseEncoderTheta = 0;
};
