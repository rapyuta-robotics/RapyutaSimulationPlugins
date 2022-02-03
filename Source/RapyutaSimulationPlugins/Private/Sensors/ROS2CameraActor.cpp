// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/ROS2CameraActor.h"

AROS2CameraActor::AROS2CameraActor()
{
	
	// Setup camera defaults
	CameraComponent = CreateDefaultSubobject<UROS2CameraComponent>(TEXT("CameraComponent"));
	CameraComponent->FieldOfView = 90.0f;
	CameraComponent->bConstrainAspectRatio = true;
	CameraComponent->AspectRatio = 1.777778f;
	CameraComponent->PostProcessBlendWeight = 1.0f;
	
	RootComponent = CameraComponent;
}