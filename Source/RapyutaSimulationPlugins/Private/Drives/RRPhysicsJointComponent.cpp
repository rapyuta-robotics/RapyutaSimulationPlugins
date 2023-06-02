// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRPhysicsJointComponent.h"

// Sets default values for this component's properties
URRPhysicsJointComponent::URRPhysicsJointComponent()
{
    // todo initializing physicsconstaints here does not work somehow.
    // Constraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("%sPhysicsConstraint"), *GetName());
    // Constraint->SetupAttachment(this);
}

void URRPhysicsJointComponent::Initialize()
{
    SetJoint();

    // set joints relations and save initial parent to joint transformation.
    JointToChildLink = URRGeneralUtils::GetRelativeTransform(
            Constraint->GetComponentTransform(),
            ChildLink->GetComponentTransform());
    ParentLinkToJoint = URRGeneralUtils::GetRelativeTransform(
            ParentLink->GetComponentTransform(),
            Constraint->GetComponentTransform());
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
            UE_LOG(LogRapyutaCore, Error, TEXT("PhysicsConstraints do not have components."));
        }

        // todo initializing physicsconstaints here does not work somehow.
        // Constraint->SetConstrainedComponents(ParentLink, TEXT("parent"), ChildLink, TEXT("child"));
        // Constraint->ComponentName2.ComponentName = ChildLink->GetFName();
        // Constraint->ComponentName1.ComponentName = ParentLink->GetFName();    


        JointToChildLink = URRGeneralUtils::GetRelativeTransform(
            Constraint->GetComponentTransform(),
            ChildLink->GetComponentTransform());

    }
    else {
         UE_LOG(LogRapyutaCore, Error, TEXT("PhysicsConstraints are not set."));
    }
    
    // todo add param to constraints, pose
}

void URRPhysicsJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
     UE_LOG(LogRapyutaCore, Error, TEXT("Can't set velocity directly to physics joint"));
}

void URRPhysicsJointComponent::SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    Constraint->SetLinearVelocityDrive(true, true, true);
    Constraint->SetAngularVelocityDrive(true, true);
    Constraint->SetLinearPositionDrive(false, false, false);
    Constraint->SetAngularOrientationDrive(false, false);
    Super::SetVelocityTarget(InLinearVelocity, InAngularVelocity);
    if (!bSmoothing)
    {
        Constraint->SetLinearVelocityTarget(InLinearVelocity);
        Constraint->SetAngularVelocityTarget(InAngularVelocity/360.0);

#if RAPYUTA_JOINT_DEBUG
        if(RotationalDOF>0)
        {
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *InAngularVelocity.ToString());
        }
        if(LinearDOF>0)
        {
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *InLinearVelocity.ToString());
        }
#endif

    }
    else
    {
        MidLinearVelocityTarget = LinearVelocity;
        MidAngularVelocityTarget = AngularVelocity;
    }
}

void URRPhysicsJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
     UE_LOG(LogRapyutaCore, Error, TEXT("Can't set pose directly to physics joint"));
}

void URRPhysicsJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    Constraint->SetLinearPositionDrive(true, true, true);
    Constraint->SetAngularOrientationDrive(true, true);
    Super::SetPoseTarget(InPosition, InOrientation);
    
    FVector poseDiff = PositionTarget - Position;
    uint8 i;
    for (i = 0; i < 3; i++)
    {
        LinearVelocityTarget[i] = FMath::IsNearlyZero(poseDiff[i], 5.0) ? 0 : poseDiff[i] < 0 ? LinearVelMin[i] : LinearVelMax[i];
    }
    for (i = 0; i < 3; i++)
    {
        float diff = FRotator::NormalizeAxis(OrientationTarget.Euler()[i] - Orientation.Euler()[i]);
        AngularVelocityTarget[i] = FMath::IsNearlyZero(diff, 1.0) ? 0 : diff < 0 ? AngularVelMin[i] : AngularVelMax[i];
    }
    
    if (!bSmoothing)
    { 
        Constraint->SetLinearVelocityTarget(LinearVelocityTarget);
        Constraint->SetAngularVelocityTarget(AngularVelocityTarget/360.0);
        Constraint->SetLinearPositionTarget(InPosition);
        // not sure why this inverse conversion is required.
        // Constraint->SetAngularOrientationTarget(InOrientation);
        Constraint->SetAngularOrientationTarget(FRotator(-InOrientation.Pitch, -InOrientation.Yaw, -InOrientation.Roll));

#if RAPYUTA_JOINT_DEBUG
        if(RotationalDOF>0)
        {
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s | %s"),
                *AngularVelocityTarget.ToString(),
                *InOrientation.Euler().ToString()
            );
        }
        if(LinearDOF>0)
        {
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s | %s"),
                *LinearVelocityTarget.ToString(),
                *InPosition.ToString()
            );
        }
#endif

    }
    else
    {
        MidLinearVelocityTarget = LinearVelocity;
        MidAngularVelocityTarget = AngularVelocity;
        MidPositionTarget = Position;
        MidOrientationTarget = Orientation;

#if RAPYUTA_JOINT_DEBUG
        if(RotationalDOF>0)
        {
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s | %s"),
                *MidAngularVelocityTarget.ToString(),
                *MidOrientationTarget.Euler().ToString()
            );
        }
        if(LinearDOF>0)
        {
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s | %s"),
                *MidLinearVelocityTarget.ToString(),
                *MidOrientationTarget.ToString()
            );
        }
#endif

    }
}

bool URRPhysicsJointComponent::StepUpdateFloat(double& current, const double target, const double step, const double tolerance)
{
    bool reached = false;
    double diff = target - current;
    double absStep = FMath::Abs(step);
    double signedStep = diff > 0 ? absStep : -absStep;
    if(FMath::Abs(diff)<=absStep || //can reach target in step
        FMath::Abs(diff)<=FMath::Abs(tolerance) || //with in tolerance
        (signedStep > 0 && diff < 0) || (signedStep < 0 && diff > 0) //overshoot
    )
    {
        current = target;
        reached = true;
    }
    else 
    {
        current += signedStep;
    }

    return reached;
}

bool URRPhysicsJointComponent::StepUpdateAngle(double& current, const double target, const double step, const double tolerance)
{
    bool reached = false;
    double currentNormalized = FRotator::NormalizeAxis(current);
    const double targetNormalized = FRotator::NormalizeAxis(target);
    double diff = FRotator::NormalizeAxis(targetNormalized  - currentNormalized);
    double absStep = FMath::Abs(step);
    double signedStep = diff > 0 ? absStep : -absStep;
    if(FMath::Abs(diff)<=absStep || //can reach target in step
        FMath::Abs(diff)<=FMath::Abs(tolerance) || //with in tolerance
        (signedStep > 0 && diff < 0) || (signedStep < 0 && diff > 0) //overshoot
    )
    {
        currentNormalized = targetNormalized;
        reached = true;
    }
    else
    {
        currentNormalized += signedStep;
        currentNormalized = FRotator::NormalizeAxis(currentNormalized);
    }

    current = currentNormalized;
    return reached;
}

void URRPhysicsJointComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    uint8 i = 0;

    // update status
    FVector prevPosition = Position;
    FRotator prevOrientation = Orientation;

    FTransform relativeTrans = URRGeneralUtils::GetRelativeTransform(
            Constraint->GetComponentTransform(),
            ChildLink->GetComponentTransform());

    Position = relativeTrans.GetLocation() - JointToChildLink.GetLocation();
    Orientation = (relativeTrans.GetRotation()*JointToChildLink.GetRotation().Inverse()).Rotator();

    FVector prevOrientationEuler = prevOrientation.Euler();
    FVector OrientationEuler = Orientation.Euler();
    for (i = 0; i < 3; i++)
    {
        LinearVelocity[i] = UKismetMathLibrary::SafeDivide(
                Position[i] - prevPosition[i], 
                DeltaTime
            );
        //todo update with quat?
        AngularVelocity[i] = UKismetMathLibrary::SafeDivide(
                FRotator::NormalizeAxis(OrientationEuler[i] - prevOrientationEuler[i]), 
                DeltaTime
            );
    }

#if RAPYUTA_JOINT_DEBUG
    if(RotationalDOF>0)
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("Target: %s | %s"),
                *AngularVelocityTarget.ToString(),
                *OrientationTarget.Euler().ToString()
            );
        UE_LOG(LogRapyutaCore, Error, TEXT("Status: %s | %s"),
                *AngularVelocity.ToString(),
                *Orientation.Euler().ToString()
            );
    }
    if(LinearDOF>0)
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("Target: %s | %s"),
                *LinearVelocityTarget.ToString(),
                *PositionTarget.ToString()
            );
        UE_LOG(LogRapyutaCore, Error, TEXT("Status: %s | %s"),
                *LinearVelocity.ToString(),
                *Position.ToString()
            );
    }
#endif

    //update mid velocity
    if(bSmoothing)
    {

        for (i = 0; i < 3; i++)
        {
            StepUpdateFloat(MidLinearVelocityTarget[i], LinearVelocityTarget[i], LinearVelocitySmoothingAcc*DeltaTime, LinearVelocityTolerance);
            StepUpdateFloat(MidAngularVelocityTarget[i], AngularVelocityTarget[i], AngularVelocitySmoothingAcc*DeltaTime, AngularVelocityTolerance);
        }
    }

    if( ControlType == ERRJointControlType::POSITION )
    {       
        FVector OrientationTargetEuler = OrientationTarget.Euler();
        FVector MidOrientationTargetEuler = MidOrientationTarget.Euler();
        if (bSmoothing)
        {
            for (i = 0; i < 3; i++)
            {
                if(StepUpdateFloat(MidPositionTarget[i], PositionTarget[i], MidLinearVelocityTarget[i]*DeltaTime, PositionTolerance))
                {
                    MidLinearVelocityTarget[i] = 0.0;
                }

                if(StepUpdateAngle(MidOrientationTargetEuler[i], OrientationTargetEuler[i], MidAngularVelocityTarget[i]*DeltaTime, OrientationTolerance))
                {
                    MidAngularVelocityTarget[i] = 0.0;
                }
                MidOrientationTarget = FRotator::MakeFromEuler(MidOrientationTargetEuler);
            }
            Constraint->SetLinearVelocityTarget(MidLinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(MidAngularVelocityTarget/360.0); 
            Constraint->SetLinearPositionTarget(MidPositionTarget);            
            // Constraint->SetAngularOrientationTarget(MidOrientationTarget);
            // not sure why this inverse conversion is required.
            Constraint->SetAngularOrientationTarget(FRotator(-MidOrientationTarget.Pitch, -MidOrientationTarget.Yaw, -MidOrientationTarget.Roll));
        }
        else
        {
            // Set velocity=0 if it reached or overshoot
            for (i = 0; i < 3; i++)
            {
                double dummy = Position[i];
                if(StepUpdateFloat(dummy, PositionTarget[i], LinearVelocityTarget[i], PositionTolerance))
                {
                    LinearVelocityTarget[i] = 0.0;
                }
                dummy = OrientationEuler[i];
                if(StepUpdateAngle(OrientationEuler[i], OrientationTargetEuler[i], AngularVelocityTarget[i], OrientationTolerance))
                {
                    AngularVelocityTarget[i] = 0.0;
                }
            }
            Constraint->SetLinearVelocityTarget(LinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(AngularVelocityTarget/360.0); //rev/s    
        }
    }
    else if (ControlType == ERRJointControlType::VELOCITY)
    {
        if(!HasReachedVelocityTarget(LinearVelocityTolerance, AngularVelocityTolerance) && bSmoothing)
        {
            Constraint->SetLinearVelocityTarget(MidLinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(MidAngularVelocityTarget/360.0); 
        }
    }

#if RAPYUTA_JOINT_DEBUG
    if(bSmoothing)
    {
        if(RotationalDOF>0)
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("Smoothing mid: %s | %s"),
                *MidAngularVelocityTarget.ToString(),
                *MidOrientationTarget.Euler().ToString()   
            );
        }
        if(LinearDOF>0)
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("Smoothing mid: %s | %s"),
                *MidLinearVelocityTarget.ToString(),
                *MidPositionTarget.ToString()
            );
        }
    }
#endif

}