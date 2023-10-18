// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/RRDifferentialDriveComponent.h"

// rclUE
#include "rclcUtilities.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"

void URRDifferentialDriveComponent::SetWheels(URRPhysicsJointComponent* InWheelLeft, URRPhysicsJointComponent* InWheelRight)
{
    auto fSetWheel = [this](URRPhysicsJointComponent*& CurWheel, URRPhysicsJointComponent* NewWheel)
    {
        if (IsValid(NewWheel))
        {
            CurWheel = NewWheel;
            CurWheel->AngularSpring = MaxForce;
            CurWheel->AngularDamper = MaxForce;
            CurWheel->AngularForceLimit = MaxForce;
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("NewWheel is invalid!"));
        }
    };

    fSetWheel(WheelLeft, InWheelLeft);
    fSetWheel(WheelRight, InWheelRight);
}

void URRDifferentialDriveComponent::UpdateMovement(float DeltaTime)
{
    if (IsValid(WheelLeft) && IsValid(WheelRight))
    {
        const float angularVelRad = FMath::DegreesToRadians(AngularVelocity.Z);
        float velL = Velocity.X + angularVelRad * WheelSeparationHalf;
        float velR = Velocity.X - angularVelRad * WheelSeparationHalf;
        WheelLeft->SetVelocityTarget(FVector::ZeroVector, FVector(FMath::RadiansToDegrees(velL / WheelRadius), 0, 0));
        WheelRight->SetVelocityTarget(FVector::ZeroVector, FVector(FMath::RadiansToDegrees(-velR / WheelRadius), 0, 0));
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }
}
