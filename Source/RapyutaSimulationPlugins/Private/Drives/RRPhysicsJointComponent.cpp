// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRPhysicsJointComponent.h"

// Sets default values for this component's properties
URRPhysicsJointComponent::URRPhysicsJointComponent()
{
    // todo initializing physicsconstaints here does not work somehow.
    // Constraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("%sPhysicsConstraint"), *GetName());
    // Constraint->SetupAttachment(this);
}

// Called when the game starts
void URRPhysicsJointComponent::BeginPlay()
{
    SetJoint();
    Super::BeginPlay();
    // todo add initialization
}

void URRPhysicsJointComponent::SetJoint()
{
    if (Constraint)
    {
        FName dummy;
        UPrimitiveComponent* component1;
        UPrimitiveComponent* component2;
        Constraint->GetConstrainedComponents(component1, dummy, component2, dummy);

        if (component1 && component2)
        {
            ChildLink = Cast<UStaticMeshComponent>(component2);
            ParentLink = Cast<UStaticMeshComponent>(component1);
        }
        else {
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("PhysicsConstraints do not have components."));
        }

        // todo initializing physicsconstaints here does not work somehow.
        // Constraint->SetConstrainedComponents(ParentLink, TEXT("parent"), ChildLink, TEXT("child"));
        // Constraint->ComponentName2.ComponentName = ChildLink->GetFName();
        // Constraint->ComponentName1.ComponentName = ParentLink->GetFName();    
    }
    else {
         UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("PhysicsConstraints are not set."));
    }
    
    // todo add param to constraints, pose
}

void URRPhysicsJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
     UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("Can't set velocity directly to physics joint"));
}

void URRPhysicsJointComponent::SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    //todo disable position control
    Super::SetVelocityTarget(InLinearVelocity, InAngularVelocity);
    if (!bSmoothing)
    {
        Constraint->SetLinearVelocityTarget(InLinearVelocity);
        Constraint->SetAngularVelocityTarget(InAngularVelocity);
    }
    else
    {
        MidLinearVelocityTarget = FVector::ZeroVector;
        MidAngularVelocityTarget = FVector::ZeroVector;
    }
}

void URRPhysicsJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
     UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("Can't set pose directly to physics joint"));
}

void URRPhysicsJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::SetPoseTarget(InPosition, InOrientation);
    if (!bSmoothing)
    {       
        FVector poseDiff = PositionTarget - Position;
        FVector orientDiff = OrientationTarget.Euler() - Orientation.Euler();
        uint8 i;
        for (i = 0; i < 3; i++)
        {
            LinearVelocityTarget[i] = FMath::IsNearlyZero(poseDiff[i]) ? 0 : poseDiff[i] < 0 ? LinearVelMin[i] : LinearVelMax[i];
        }
        for (i = 0; i < 3; i++)
        {
            AngularVelocityTarget[i] = FMath::IsNearlyZero(orientDiff[i]) ? 0 : orientDiff[i] < 0 ? AngularVelMin[i] : AngularVelMax[i];
        }
        Constraint->SetLinearVelocityTarget(LinearVelocityTarget);
        Constraint->SetAngularVelocityTarget(AngularVelocityTarget);
        Constraint->SetLinearPositionTarget(InPosition);
        Constraint->SetAngularOrientationTarget(InOrientation);
    }
    else
    {
        MidLinearVelocityTarget = FVector::ZeroVector;
        MidAngularVelocityTarget = FVector::ZeroVector;
        MidPositionTarget = FVector::ZeroVector;
        MidOrientationTarget = FRotator::ZeroRotator;
    }
}

void URRPhysicsJointComponent::StepUpdateFloat(double& current, const double target, const double step, const double tolerance)
{
    if (FMath::IsNearlyEqual(target,  current, tolerance))
    {
        current = target;
    }
    else 
    {
        double diff = target - current;
        current += step * diff/FMath::Abs(diff);
        // cap velocity
        if(diff > 0 && current >= target)
        {
            current = target;
        }
        else if(diff < 0 &&  current <= target)
        {
            current = target;
        }
    }
}

void URRPhysicsJointComponent::StepUpdateRotator(FRotator& current, const FRotator target, const FVector step, const double tolerance)
{
    //convert to quatnion(axis, angle)
    //setp update angle
    //convert to rotator
    StepUpdateFloat(current.Roll, target.Roll, step[0], tolerance);
    StepUpdateFloat(current.Pitch, target.Pitch, step[1], tolerance);
    StepUpdateFloat(current.Yaw, target.Yaw, step[2], tolerance);
}

void URRPhysicsJointComponent::StepUpdateVector(FVector& current, const FVector target, const FVector step, const double tolerance)
{
    uint8 i;
    for (i = 0; i < 3; i++)
    {
        StepUpdateFloat(current[i], target[i], step[i], tolerance);
    }
}

void URRPhysicsJointComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
     
    // Pose = 

    //update mid velocity
    if(bSmoothing)
    {
        StepUpdateVector(MidLinearVelocityTarget, LinearVelocityTarget, FVector::OneVector*LinearVelocitySmoothingAcc*DeltaTime, LinearVelocityTolerance);
        StepUpdateVector(MidAngularVelocityTarget, AngularVelocityTarget, FVector::OneVector*AngularVelocitySmoothingAcc*DeltaTime, AngularVelocityTolerance);
    }

    if( ControlType == ERRJointControlType::POSITION )
    {
        if ( HasReachedPoseTarget(PositionTolerance, OrientationTolerance) )
        {
            Constraint->SetLinearVelocityTarget(FVector::ZeroVector);
            Constraint->SetAngularVelocityTarget(FVector::ZeroVector);    
        }
        else if (bSmoothing)
        {

            StepUpdateVector(MidPositionTarget, PositionTarget, MidLinearVelocityTarget*DeltaTime, LinearVelocityTolerance);
            Constraint->SetLinearVelocityTarget(MidLinearVelocityTarget);
            Constraint->SetLinearPositionTarget(MidPositionTarget);
            
            // todo calculate rotar update            
            StepUpdateRotator(MidOrientationTarget, OrientationTarget, MidAngularVelocityTarget*DeltaTime, AngularVelocityTolerance);
            Constraint->SetAngularVelocityTarget(MidAngularVelocityTarget); 
            Constraint->SetAngularOrientationTarget(MidOrientationTarget);
        }
    }
    else if (ControlType == ERRJointControlType::VELOCITY)
    {
        if(!HasReachedVelocityTarget(LinearVelocityTolerance, AngularVelocityTolerance) && bSmoothing)
        {
            Constraint->SetLinearVelocityTarget(MidLinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(MidAngularVelocityTarget); 
        }
    }
}