// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Sensors/ROS2CameraComponent.h"

#include "ROS2CameraActor.generated.h"

/**
 * 
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API AROS2CameraActor : public AActor
{
	GENERATED_BODY()
	
public:

	AROS2CameraActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	/** The camera component for this camera */
	UPROPERTY(Category = CameraActor, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UROS2CameraComponent* CameraComponent;
};
