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
            // CurWheel->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
            // CurWheel->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
            // CurWheel->SetAngularVelocityDriveTwistAndSwing(true, false);

            CurWheel->AngularSpring = MaxForce;
            CurWheel->AngularDamper = MaxForce;
            CurWheel->AngularForceLimit = MaxForce;

            // CurWheel->Constraint->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("NewWheel is invalid!"));
        }
    };

    fSetWheel(WheelLeft, InWheelLeft);
    fSetWheel(WheelRight, InWheelRight);

    // WheelLeft = InWheelLeft;
    // WheelLeft->AngularSpring = 0;
    // WheelLeft->AngularDamper = 0;
    // WheelLeft->AngularForceLimit = MaxForce;
    // WheelRight = InWheelRight;
    // WheelRight->AngularSpring = 0;
    // WheelRight->AngularDamper = 0;
    // WheelRight->AngularForceLimit = MaxForce;

    // WheelRight->Constraint->SetAngularDriveParams(0, 0, MaxForce);
}

void URRDifferentialDriveComponent::UpdateMovement(float DeltaTime)
{
    if (IsValid(WheelLeft) && IsValid(WheelRight))
    {
        const float angularVelRad = FMath::DegreesToRadians(AngularVelocity.Z);
        float velL = Velocity.X + angularVelRad * WheelSeparationHalf;
        float velR = Velocity.X - angularVelRad * WheelSeparationHalf;
        WheelLeft->SetVelocityTarget(FVector::ZeroVector, FVector(-velL / WheelPerimeter, 0, 0));
        WheelRight->SetVelocityTarget(FVector::ZeroVector, FVector(-velR / WheelPerimeter, 0, 0));
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }
}
