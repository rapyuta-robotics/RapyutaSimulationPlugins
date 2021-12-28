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

	/** The camera component for this camera */
	UPROPERTY(Category = CameraActor, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UROS2CameraComponent* CameraComponent;
};
