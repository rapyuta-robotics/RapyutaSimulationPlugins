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
        const float angularVelRad = FMath::DegreesToRadians(AngularVelocity.Z);
        float velL = Velocity.X + angularVelRad * WheelSeparationHalf;
        float velR = Velocity.X - angularVelRad * WheelSeparationHalf;

        WheelLeft->SetAngularVelocityTarget(FVector(velL / WheelPerimeter, 0, 0));
        WheelRight->SetAngularVelocityTarget(FVector(-velR / WheelPerimeter, 0, 0));
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }
}

float UDifferentialDriveComponent::GetWheelVelocity(const EDiffDriveWheel WheelIndex)
{
    // todo calculate from wheel pose
    const float angularVelRad = FMath::DegreesToRadians(AngularVelocity.Z);
    float out = 0;
    if (WheelIndex == EDiffDriveWheel::LEFT)
    {
        // left wheel
        out = Velocity.X + angularVelRad * WheelSeparationHalf;    //cm
    }
    else if (WheelIndex == EDiffDriveWheel::RIGHT)
    {
        // right wheel
        out = Velocity.X - angularVelRad * WheelSeparationHalf;    //cm
    }

    return out;
}
