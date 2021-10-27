// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotVehicle.h"

#include "Drives/RobotVehicleMovementComponent.h"
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

void ARobotVehicle::InitializeMoveComponent()
{
    RobotVehicleMoveComponent = NewObject<URobotVehicleMovementComponent>(this, TEXT("RobotVehicleMoveComponent"));
    verify(RobotVehicleMoveComponent);
}

void ARobotVehicle::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
}

void ARobotVehicle::SetLinearVel(const FVector& InLinearVelocity)
{
    // We're assuming input is in meters, so convert to centimeters.
    RobotVehicleMoveComponent->Velocity = InLinearVelocity;    // 0
}

void ARobotVehicle::SetAngularVel(const FVector& InAngularVelocity)
{
    RobotVehicleMoveComponent->AngularVelocity = InAngularVelocity;
}

void ARobotVehicle::BeginPlay()
{
    Super::BeginPlay();
    RobotVehicleMoveComponent->Initialize();
}

void ARobotVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
}
