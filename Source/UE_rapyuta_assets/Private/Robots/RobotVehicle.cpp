// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotVehicle.h"

#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

ARobotVehicle::ARobotVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
}

void ARobotVehicle::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
}

void ARobotVehicle::SetLinearVel(FVector Velocity)
{
    // We're assuming input is in meters, so convert to centimeters.
    MoveComponent->Velocity = Velocity;
}

void ARobotVehicle::SetAngularVel(FVector Velocity)
{
    MoveComponent->AngularVelocity = Velocity;
}

void ARobotVehicle::BeginPlay()
{
    Super::BeginPlay();
    MoveComponent->InitMovementComponent();
}

void ARobotVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
}
