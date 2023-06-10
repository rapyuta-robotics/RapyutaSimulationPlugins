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
    
    for (uint8 i = 0; i < 3; i++)
    {
        PositionTPI[i] = TwoPointInterpolation(true);
        OrientationTPI[i] = TwoAngleInterpolation(true);
    }
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
    // change to velocity control mode
    Constraint->SetLinearVelocityDrive(true, true, true);
    Constraint->SetAngularVelocityDrive(true, true);
    Constraint->SetLinearPositionDrive(false, false, false);
    Constraint->SetAngularOrientationDrive(false, false);

    // set velocity target
    Super::SetVelocityTarget(InLinearVelocity, InAngularVelocity);
    if (!bSmoothing)
    {
        Constraint->SetLinearVelocityTarget(InLinearVelocity);
        Constraint->SetAngularVelocityTarget(InAngularVelocity/360.0);
    }
    else
    {
        MidLinearVelocityTarget = LinearVelocity;
        MidAngularVelocityTarget = AngularVelocity;
    }

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

void URRPhysicsJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
     UE_LOG(LogRapyutaCore, Error, TEXT("Can't set pose directly to physics joint"));
}

void URRPhysicsJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    // if target is within tolerance, ignore it.
    if (PositionTarget.Equals(InPosition, PositionTolerance) && 
        OrientationTarget.Equals(InOrientation, OrientationTolerance))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Verbose, TEXT("New target is within tolerance."));
        return;
    }

    // change to position control mode
    Constraint->SetLinearPositionDrive(true, true, true);
    Constraint->SetAngularOrientationDrive(true, true);
    Super::SetPoseTarget(InPosition, InOrientation);

    FVector OrientationEuler = Orientation.Euler();
    FVector OrientationTargetEuler = OrientationTarget.Euler();
    uint8 i; 
    if (!bSmoothing)
    { 
        // set maximum velocity for target direction.
        FVector poseDiff = PositionTarget - Position;
        FVector orientationDiff = OrientationTargetEuler - OrientationEuler;
        for (i = 0; i < 3; i++)
        {
            LinearVelocityTarget[i] = FMath::IsNearlyZero(poseDiff[i], PositionTolerance) ? 0 : poseDiff[i] < 0 ? -LinearVelMax[i] : LinearVelMax[i];
        }
        for (i = 0; i < 3; i++)
        {
            float diff = FRotator::NormalizeAxis(orientationDiff[i]);
            AngularVelocityTarget[i] = FMath::IsNearlyZero(diff, OrientationTolerance) ? 0 : diff < 0 ? -AngularVelMax[i] : AngularVelMax[i];
        }
    
        Constraint->SetLinearVelocityTarget(LinearVelocityTarget);
        Constraint->SetAngularVelocityTarget(AngularVelocityTarget/360.0); // rev/s
        Constraint->SetLinearPositionTarget(InPosition);
        // not sure why this inverse conversion is required.
        // Constraint->SetAngularOrientationTarget(InOrientation);
        Constraint->SetAngularOrientationTarget(FRotator(-InOrientation.Pitch, -InOrientation.Yaw, -InOrientation.Roll));

    }
    else
    {
        //Initalize interpolation
        MidPositionTarget = Position;
        MidOrientationTarget = Orientation;
        float t0 = GetWorld()->GetTimeSeconds();
        for (i = 0; i < 3; i++)
        {    
            if (!FMath::IsNearlyEqual(Position[i], PositionTarget[i], PositionTolerance))
            {
                PositionTPI[i].calcTrajectory(
                    Position[i], PositionTarget[i], //pose
                    LinearVelocitySmoothingAcc, LinearVelMax[i], //max
                    t0, 
                    LinearVelocity[i], 0.0 //velocity
                );
            }

            if (!FMath::IsNearlyEqual(OrientationEuler[i], OrientationTargetEuler[i], OrientationTolerance))
            {
                OrientationTPI[i].calcTrajectory(
                    FMath::DegreesToRadians(OrientationEuler[i]), 
                    FMath::DegreesToRadians(OrientationTargetEuler[i]), //pose
                    FMath::DegreesToRadians(AngularVelocitySmoothingAcc), 
                    FMath::DegreesToRadians(AngularVelMax[i]), //max
                    t0, 
                    FMath::DegreesToRadians(AngularVelocity[i]), 0.0 //velocity
                );
            }
        }
    }
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

void URRPhysicsJointComponent::UpdateState(const float DeltaTime)
{
    FVector prevPosition = Position;
    FRotator prevOrientation = Orientation;

    FTransform relativeTrans = URRGeneralUtils::GetRelativeTransform(
            Constraint->GetComponentTransform(),
            ChildLink->GetComponentTransform());

    Position = relativeTrans.GetLocation() - JointToChildLink.GetLocation();
    Orientation = (relativeTrans.GetRotation()*JointToChildLink.GetRotation().Inverse()).Rotator();

    FVector prevOrientationEuler = prevOrientation.Euler();
    FVector OrientationEuler = Orientation.Euler();
    for (uint8 i = 0; i < 3; i++)
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
}

void URRPhysicsJointComponent::UpdateControl(const float DeltaTime)
{
    uint8 i = 0;
    if( ControlType == ERRJointControlType::POSITION )
    {       
        FVector OrientationTargetEuler = OrientationTarget.Euler();
        FVector MidOrientationTargetEuler = MidOrientationTarget.Euler();
        if (bSmoothing)
        {
            // input
            float t = GetWorld()->GetTimeSeconds();
            
            // output
            bool initialized = true;
            for (i = 0; i < 3; i++)
            {
                if (PositionTPI[i].isInitialized())
                {
                    std::vector<double> resPos = PositionTPI[i].getPoint(t);
                    MidPositionTarget[i] = resPos[0];
                    MidLinearVelocityTarget[i] = resPos[1];
                }
                if (OrientationTPI[i].isInitialized())
                {
                    std::vector<double> resOri = OrientationTPI[i].getPoint(t);
                    MidOrientationTargetEuler[i] = FMath::RadiansToDegrees(resOri[0]);
                    MidAngularVelocityTarget[i] = FMath::RadiansToDegrees(resOri[1]);   
                }
            }
            MidOrientationTarget = FRotator::MakeFromEuler(MidOrientationTargetEuler);

            Constraint->SetLinearVelocityTarget(MidLinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(MidAngularVelocityTarget/360.0); 
            Constraint->SetLinearPositionTarget(MidPositionTarget);           
            // not sure why this inverse conversion is required.
            Constraint->SetAngularOrientationTarget(FRotator(-MidOrientationTarget.Pitch, -MidOrientationTarget.Yaw, -MidOrientationTarget.Roll));
        }
        else
        {
            // todo support linear update in two_points_interpolation_constant_acc.hpp
            // Set velocity=0 if it reached or overshoot
            // Use URRMathUtils::URRMathUtils::StepUpdate to check current value is within tolerance.
            FVector OrientationEuler = Orientation.Euler();
            for (i = 0; i < 3; i++)
            {
                double dummy = Position[i];
                if(URRMathUtils::StepUpdate(dummy, PositionTarget[i], LinearVelocityTarget[i], PositionTolerance))
                {
                    LinearVelocityTarget[i] = 0.0;
                }
                dummy = OrientationEuler[i];
                if(URRMathUtils::StepUpdateAngle(dummy, OrientationTargetEuler[i], AngularVelocityTarget[i], OrientationTolerance))
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
        // Linear update of velocity
        for (i = 0; i < 3; i++)
        {
            URRMathUtils::StepUpdate(MidLinearVelocityTarget[i], LinearVelocityTarget[i], LinearVelocitySmoothingAcc*DeltaTime, LinearVelocityTolerance);
            URRMathUtils::StepUpdate(MidAngularVelocityTarget[i], AngularVelocityTarget[i], AngularVelocitySmoothingAcc*DeltaTime, AngularVelocityTolerance);
        }

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

void URRPhysicsJointComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UpdateState(DeltaTime);
    UpdateControl(DeltaTime);
}