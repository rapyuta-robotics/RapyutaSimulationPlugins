// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/DifferentialDriveComponent.h"

// rclUE
#include "rclcUtilities.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

void UDifferentialDriveComponent::SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight, UStaticMeshComponent* InWheelLeftLink, UStaticMeshComponent* InWheelRightLink)
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
    if (!IsValid(InWheelLeftLink))
    {
        UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("Wheel Left Link is invalid! Ensure it is properly initialized and assigned."));
        return;
    }
    if (!IsValid(InWheelRightLink))
    {
        UE_LOG_WITH_INFO_NAMED(LogDifferentialDriveComponent, Error, TEXT("Wheel Right Link is invalid! Ensure it is properly initialized and assigned."));
        return;
    }
    else
    {
        WheelLeftLink = InWheelLeftLink;
        WheelRightLink = InWheelRightLink;
        LeftJointToChildLink =
                URRGeneralUtils::GetRelativeTransform(WheelLeft->GetComponentTransform(), WheelLeftLink->GetComponentTransform());
        RightJointToChildLink = 
                URRGeneralUtils::GetRelativeTransform(WheelRight->GetComponentTransform(), WheelRightLink->GetComponentTransform());
    }
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

float UDifferentialDriveComponent::GetWheelVelocity(const EDiffDriveWheel WheelIndex, float DeltaTime)
{
    if(!IsValid(WheelLeftLink) || !IsValid(WheelRightLink))
    {
        return 0;
    }
    // todo calculate from wheel pose
    // const float angularVelRad = FMath::DegreesToRadians(AngularVelocity.Z);
    float out = 0;

    FVector dummyPosition = FVector::ZeroVector;
    FVector angularVelocity = FVector::ZeroVector;
    FRotator prevOrientation = FRotator::ZeroRotator;
    FVector prevOrientationEuler = FVector::ZeroVector;
    FVector OrientationEuler = FVector::ZeroVector;
     
    if (WheelIndex == EDiffDriveWheel::LEFT)
    {
        // left wheel
        // out = Velocity.X + angularVelRad * WheelSeparationHalf;    //cm
        prevOrientation = LeftWheelOrientation;
        prevOrientationEuler = prevOrientation.Euler();
        URRGeneralUtils::GetPhysicsConstraintTransform(WheelLeft, LeftJointToChildLink, dummyPosition, LeftWheelOrientation, WheelLeftLink);
        OrientationEuler = LeftWheelOrientation.Euler();
        for (uint8 i = 0; i < 3; i++)
        {
            angularVelocity[i] =
                 UKismetMathLibrary::SafeDivide(FRotator::NormalizeAxis(OrientationEuler[i] - prevOrientationEuler[i]), DeltaTime);
        }
    }
    else if (WheelIndex == EDiffDriveWheel::RIGHT)
    {
        // right wheel
        // out = Velocity.X - angularVelRad * WheelSeparationHalf;    //cm
        prevOrientation = RightWheelOrientation;
        prevOrientationEuler = prevOrientation.Euler();
        URRGeneralUtils::GetPhysicsConstraintTransform(WheelRight, RightJointToChildLink, dummyPosition, RightWheelOrientation, WheelRightLink);
        OrientationEuler = RightWheelOrientation.Euler();
        for (uint8 i = 0; i < 3; i++)
        {
            angularVelocity[i] =
                - UKismetMathLibrary::SafeDivide(FRotator::NormalizeAxis(OrientationEuler[i] - prevOrientationEuler[i]), DeltaTime);
        }
    }
    // Only the first element of angularVelocity is used because it represents the rotation around the axis
    // relevant to the wheel's movement (typically the X-axis in this context).
    return FMath::DegreesToRadians(angularVelocity[0]) * WheelRadius;
}
