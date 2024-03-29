// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/TurtlebotBurger.h"

ATurtlebotBurger::ATurtlebotBurger(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    VehicleMoveComponentClass = UDifferentialDriveComponent::StaticClass();
    bBodyComponentsCreated = false;
    SetupBody();
    SetupConstraintsAndPhysics();
}

bool ATurtlebotBurger::SetupBody()
{
    if (bBodyComponentsCreated)
    {
        return false;
    }

    // Constraints
    Base_WheelLeft = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelLeft"));

    Base_WheelRight = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelRight"));

    bBodyComponentsCreated = true;

    return true;
}

void ATurtlebotBurger::SetupWheelDrives()
{
    if (bBodyComponentsCreated && IsValid(MovementComponent))
    {
        UDifferentialDriveComponent* diffDriveComponent = CastChecked<UDifferentialDriveComponent>(MovementComponent);
        diffDriveComponent->SetWheels(Base_WheelLeft, Base_WheelRight);
        diffDriveComponent->WheelRadius = WheelRadius;
        diffDriveComponent->WheelSeparationHalf = WheelSeparationHalf;
        diffDriveComponent->SetPerimeter();
    }
}

bool ATurtlebotBurger::SetupConstraintsAndPhysics()
{
    if (bBodyComponentsCreated)
    {
        Base_WheelLeft->ComponentName1.ComponentName = TEXT("Base");
        Base_WheelLeft->ComponentName2.ComponentName = TEXT("WheelLeft");
        Base_WheelLeft->SetDisableCollision(true);
        Base_WheelLeft->SetRelativeLocation(FVector(3.2, -8, 2.3));
        Base_WheelLeft->SetRelativeRotation(FRotator(0, -90, 0));
        Base_WheelLeft->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
        Base_WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
        Base_WheelLeft->SetAngularVelocityDriveTwistAndSwing(true, false);
        Base_WheelLeft->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
        Base_WheelLeft->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
        Base_WheelLeft->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_WheelLeft->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_WheelLeft->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

        Base_WheelRight->ComponentName1.ComponentName = TEXT("Base");
        Base_WheelRight->ComponentName2.ComponentName = TEXT("WheelRight");
        Base_WheelRight->SetDisableCollision(true);
        Base_WheelRight->SetRelativeLocation(FVector(3.2, 8, 2.3));
        Base_WheelRight->SetRelativeRotation(FRotator(0, 90, 0));
        Base_WheelRight->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
        Base_WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
        Base_WheelRight->SetAngularVelocityDriveTwistAndSwing(true, false);
        Base_WheelRight->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
        Base_WheelRight->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
        Base_WheelRight->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_WheelRight->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_WheelRight->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

        // need to attach child to physics constraint first before attaching physics constraint to parent.
        WheelLeft->SetupAttachment(Base_WheelLeft);
        WheelLeft->SetRelativeRotation(FRotator(0, -90, 0));
        WheelRight->SetupAttachment(Base_WheelRight);
        WheelRight->SetRelativeRotation(FRotator(0, 90, 0));

        Base_WheelRight->SetupAttachment(Base);
        Base_WheelLeft->SetupAttachment(Base);

        return true;
    }
    else
    {
        return false;
    }
}
