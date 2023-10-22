// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/DifferentialDriveComponent.h"

// rclUE
#include "rclcUtilities.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"

void UDifferentialDriveComponent::SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight)
{
    auto fSetWheel = [this](UPhysicsConstraintComponent*& CurWheel, UPhysicsConstraintComponent* NewWheel)
    {
        if (IsValid(NewWheel))
        {
            CurWheel = NewWheel;
            CurWheel->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
            CurWheel->SetAngularDriveParams(0, MaxForce, MaxForce);
            CurWheel->SetAngularVelocityDriveTwistAndSwing(true, false);
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("NewWheel is invalid!"));
        }
    };

    fSetWheel(WheelLeft, InWheelLeft);
    fSetWheel(WheelRight, InWheelRight);
}

void UDifferentialDriveComponent::UpdateMovement(float DeltaTime)
{
    if (IsValid(WheelLeft) && IsValid(WheelRight))
    {
        float velL = GetWheelVelocity(0);
        float velR = GetWheelVelocity(1);

        WheelLeft->SetAngularVelocityTarget(FVector(velL / WheelPerimeter, 0, 0));
        WheelRight->SetAngularVelocityTarget(FVector(-velR / WheelPerimeter, 0, 0));
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }
}

float UDifferentialDriveComponent::GetWheelVelocity(const int index)
{
    const float angularVelRad = FMath::DegreesToRadians(AngularVelocity.Z);
    float out = 0;
    if (index == 0)
    {
        // left wheel
        out = Velocity.X + angularVelRad * WheelSeparationHalf;    //cm
    }
    else if (index == 1)
    {
        // right wheel
        out = Velocity.X - angularVelRad * WheelSeparationHalf;    //cm
    }

    return out;
}
