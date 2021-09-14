// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/DifferentialDriveComponent.h"

DEFINE_LOG_CATEGORY(LogDifferentialDriveComponent);

UDifferentialDriveComponent::UDifferentialDriveComponent()
{
}

void UDifferentialDriveComponent::SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight)
{
    WheelLeft = InWheelLeft;
    WheelRight = InWheelRight;

    WheelLeft->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
    WheelLeft->SetAngularVelocityDriveTwistAndSwing(true, false);

    WheelRight->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
    WheelRight->SetAngularVelocityDriveTwistAndSwing(true, false);
}

void UDifferentialDriveComponent::SetPerimeter()
{
    if (WheelRadius <= 1e-6)
    {
        UE_LOG(LogDifferentialDriveComponent, Warning, TEXT("Wheel radius is too small. Wheel radius is reset to 1.0"));
        WheelRadius = 1.0f;
    }
    WheelPerimeter = WheelRadius * 2.0 * 3.1416;
}

void UDifferentialDriveComponent::UpdateMovement(float DeltaTime)
{
    if (IsValid(WheelLeft) && IsValid(WheelRight))
    {
        float velL = Velocity.X + AngularVelocity.Z * WheelSeparationHalf;
        float velR = Velocity.X - AngularVelocity.Z * WheelSeparationHalf;

        WheelLeft->SetAngularVelocityTarget(FVector(-velL / WheelPerimeter, 0, 0));
        WheelRight->SetAngularVelocityTarget(FVector(-velR / WheelPerimeter, 0, 0));
        WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
        WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
    }
    else
    {
        UE_LOG(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }
}

void UDifferentialDriveComponent::InitMovementComponent()
{
    Super::InitMovementComponent();

    if (!IsValid(WheelLeft) || !IsValid(WheelRight))
    {
        UE_LOG(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }

    SetPerimeter();
}