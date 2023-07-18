// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/RRPhysicsJointComponent.h"

// Sets default values for this component's properties
URRPhysicsJointComponent::URRPhysicsJointComponent()
{
    // todo initializing physicsconstaints here does not work somehow.
    Constraint = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("%sPhysicsConstraint"), *GetName());
    Constraint->AttachToComponent(this, FAttachmentTransformRules::KeepRelativeTransform);
}

bool URRPhysicsJointComponent::IsValid()
{
    return Super::IsValid() && Constraint;
}

void URRPhysicsJointComponent::Initialize()
{
    if (IsValid())
    {
        if (!bManualConstraintSetting)
        {
            SetJoint();
        }

        // set joints relations and save initial parent to joint transformation.
        JointToChildLink =
            URRGeneralUtils::GetRelativeTransform(Constraint->GetComponentTransform(), ChildLink->GetComponentTransform());
        ParentLinkToJoint =
            URRGeneralUtils::GetRelativeTransform(ParentLink->GetComponentTransform(), Constraint->GetComponentTransform());
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogTemp, Error, TEXT("JointComponent must have Physics Constraints"));
    }
}

void URRPhysicsJointComponent::SetJoint()
{
    Constraint->SetConstrainedComponents(ParentLink, NAME_None, ChildLink, NAME_None);

    // set linear drive
    // LinearDOF = 1 => x, 2 => x,y, 3 => x,y,z
    bool linearEnabled[3] = {false, false, false};
    for (uint8 i = 0; i < 3; i++)
    {
        if (LinearDOF > i)
        {
            linearEnabled[i] = true;
        }
    }
    Constraint->SetLinearPositionDrive(linearEnabled[0], linearEnabled[1], linearEnabled[2]);
    Constraint->SetLinearVelocityDrive(linearEnabled[0], linearEnabled[1], linearEnabled[2]);
    Constraint->SetLinearXLimit(linearEnabled[0] ? ELinearConstraintMotion::LCM_Free : ELinearConstraintMotion::LCM_Locked, 0);
    Constraint->SetLinearYLimit(linearEnabled[1] ? ELinearConstraintMotion::LCM_Free : ELinearConstraintMotion::LCM_Locked, 0);
    Constraint->SetLinearZLimit(linearEnabled[2] ? ELinearConstraintMotion::LCM_Free : ELinearConstraintMotion::LCM_Locked, 0);
    Constraint->SetLinearDriveParams(LinearSpring, LinearDamper, LinearForceLimit);

    // set angular drive
    // RotationalDOF = 1 => Twist, 2 => Swing1, 3 => Swing2
    bool angleEnabled[3] = {false, false, false};
    for (uint8 i = 0; i < 3; i++)
    {
        if (RotationalDOF > i)
        {
            angleEnabled[i] = true;
        }
    }
    Constraint->SetAngularDriveMode(angleEnabled[2] ? EAngularDriveMode::SLERP : EAngularDriveMode::TwistAndSwing);
    if (angleEnabled[2])    // if DOF==3, it uses SLERP
    {
        Constraint->SetOrientationDriveSLERP(true);
        Constraint->SetAngularVelocityDriveSLERP(true);
    }
    else
    {
        Constraint->SetAngularOrientationDrive(angleEnabled[1], angleEnabled[0]);
        Constraint->SetAngularVelocityDrive(angleEnabled[1], angleEnabled[0]);
    }
    Constraint->SetAngularTwistLimit(angleEnabled[0] ? EAngularConstraintMotion::ACM_Free : EAngularConstraintMotion::ACM_Locked,
                                     0);
    Constraint->SetAngularSwing1Limit(angleEnabled[1] ? EAngularConstraintMotion::ACM_Free : EAngularConstraintMotion::ACM_Locked,
                                      0);
    Constraint->SetAngularSwing2Limit(angleEnabled[2] ? EAngularConstraintMotion::ACM_Free : EAngularConstraintMotion::ACM_Locked,
                                      0);
    Constraint->SetAngularDriveParams(AngularSpring, AngularDamper, AngularForceLimit);
    Constraint->SetDisableCollision(true);
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
        Constraint->SetAngularVelocityTarget(InAngularVelocity / 360.0);
    }
    else
    {
        MidLinearVelocityTarget = LinearVelocity;
        MidAngularVelocityTarget = AngularVelocity;
    }

#if RAPYUTA_JOINT_DEBUG
    if (RotationalDOF > 0)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *InAngularVelocity.ToString());
    }
    if (LinearDOF > 0)
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
    if (PositionTarget.Equals(InPosition, PositionTolerance) && OrientationTarget.Equals(InOrientation, OrientationTolerance))
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
            LinearVelocityTarget[i] = FMath::Clamp(Kvp * poseDiff[i], -LinearVelMax[i], LinearVelMax[i]);
        }
        for (i = 0; i < 3; i++)
        {
            float diff = FRotator::NormalizeAxis(orientationDiff[i]);
            AngularVelocityTarget[i] = FMath::Clamp(Kva * poseDiff[i], -AngularVelMax[i], AngularVelMax[i]);
        }

        Constraint->SetLinearVelocityTarget(LinearVelocityTarget);
        Constraint->SetAngularVelocityTarget(AngularVelocityTarget / 360.0);    // rev/s
        Constraint->SetLinearPositionTarget(InPosition + PErrInt);
        Constraint->SetAngularOrientationTarget(GetOrientationTargetFromEuler(InOrientation.Euler() + AErrInt));
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
                PositionTPI[i].calcTrajectory(Position[i],
                                              PositionTarget[i],    //pose
                                              LinearVelocitySmoothingAcc,
                                              LinearVelMax[i],    //max
                                              t0,
                                              LinearVelocity[i],
                                              0.0    //velocity
                );
            }

            if (!FMath::IsNearlyEqual(OrientationEuler[i], OrientationTargetEuler[i], OrientationTolerance))
            {
                OrientationTPI[i].calcTrajectory(FMath::DegreesToRadians(OrientationEuler[i]),
                                                 FMath::DegreesToRadians(OrientationTargetEuler[i]),    //pose
                                                 FMath::DegreesToRadians(AngularVelocitySmoothingAcc),
                                                 FMath::DegreesToRadians(AngularVelMax[i]),    //max
                                                 t0,
                                                 FMath::DegreesToRadians(AngularVelocity[i]),
                                                 0.0    //velocity
                );
            }
        }
    }
#if RAPYUTA_JOINT_DEBUG
    if (RotationalDOF > 0)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("%s | %s"), *AngularVelocityTarget.ToString(), *InOrientation.Euler().ToString());
    }
    if (LinearDOF > 0)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("%s | %s"), *LinearVelocityTarget.ToString(), *InPosition.ToString());
    }
#endif
}

void URRPhysicsJointComponent::UpdateState(const float DeltaTime)
{
    FVector prevPosition = Position;
    FRotator prevOrientation = Orientation;

    FTransform relativeTrans =
        URRGeneralUtils::GetRelativeTransform(Constraint->GetComponentTransform(), ChildLink->GetComponentTransform());

    Position = relativeTrans.GetLocation() - JointToChildLink.GetLocation();
    Orientation = (relativeTrans.GetRotation() * JointToChildLink.GetRotation().Inverse()).Rotator();

    FVector prevOrientationEuler = prevOrientation.Euler();
    FVector OrientationEuler = Orientation.Euler();

    for (uint8 i = 0; i < 3; i++)
    {
        LinearVelocity[i] = UKismetMathLibrary::SafeDivide(Position[i] - prevPosition[i], DeltaTime);
        //todo update with quat?
        AngularVelocity[i] =
            UKismetMathLibrary::SafeDivide(FRotator::NormalizeAxis(OrientationEuler[i] - prevOrientationEuler[i]), DeltaTime);
    }
#if RAPYUTA_JOINT_DEBUG
    if (RotationalDOF > 0)
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("Target: %s | %s"),
               *AngularVelocityTarget.ToString(),
               *OrientationTarget.Euler().ToString());
        UE_LOG(LogRapyutaCore, Error, TEXT("Status: %s | %s"), *AngularVelocity.ToString(), *Orientation.Euler().ToString());
    }
    if (LinearDOF > 0)
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("Target: %s | %s"), *LinearVelocityTarget.ToString(), *PositionTarget.ToString());
        UE_LOG(LogRapyutaCore, Error, TEXT("Status: %s | %s"), *LinearVelocity.ToString(), *Position.ToString());
    }
#endif
}

void URRPhysicsJointComponent::UpdateVelocityTargetFromPose(const FVector InPositionDiff, const FVector InOrientationDiff)
{
    uint8 i;
    for (i = 0; i < 3; i++)
    {
        LinearVelocityTarget[i] = FMath::Clamp(Kvp * InPositionDiff[i], -LinearVelMax[i], LinearVelMax[i]);

        float diff = FRotator::NormalizeAxis(InOrientationDiff[i]);
        AngularVelocityTarget[i] = FMath::Clamp(Kva * diff, -AngularVelMax[i], AngularVelMax[i]);
    }
}

void URRPhysicsJointComponent::UpdateIntegral(const FVector& InPositionDiff,
                                              const FVector& InOrientationDiff,
                                              const float DeltaTime)
{
    uint8 i;
    for (i = 0; i < 3; i++)
    {
        PErrInt[i] += Kip * DeltaTime * InPositionDiff[i];

        float diff = FRotator::NormalizeAxis(InOrientationDiff[i]);
        AErrInt[i] += Kia * DeltaTime * diff;
    }
}

FRotator URRPhysicsJointComponent::GetOrientationTargetFromEuler(const FVector& InOrientationTarget)
{
    // not sure why this inverse conversion is required.
    FRotator orientationTarget = FRotator::MakeFromEuler(InOrientationTarget);
    return FRotator(-orientationTarget.Pitch, -orientationTarget.Yaw, -orientationTarget.Roll);
}

void URRPhysicsJointComponent::UpdateControl(const float DeltaTime)
{
    uint8 i = 0;
    if (ControlType == ERRJointControlType::POSITION)
    {
        if (bSmoothing)
        {
            // input
            float t = GetWorld()->GetTimeSeconds();

            // output
            FVector MidOrientationTargetEuler = MidOrientationTarget.Euler();
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
            Constraint->SetLinearVelocityTarget(MidLinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(MidAngularVelocityTarget / 360.0);
            Constraint->SetLinearPositionTarget(MidPositionTarget);
            Constraint->SetAngularOrientationTarget(GetOrientationTargetFromEuler(MidOrientationTargetEuler));
        }
        else
        {
            FVector poseDiff = PositionTarget - Position;
            FVector orientationEuler = Orientation.Euler();
            FVector orientationTargetEuler = OrientationTarget.Euler();
            FVector orientationDiff = orientationTargetEuler - orientationEuler;
            UpdateVelocityTargetFromPose(poseDiff, orientationDiff);
            UpdateIntegral(poseDiff, orientationDiff, DeltaTime);

            Constraint->SetLinearVelocityTarget(LinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(AngularVelocityTarget / 360.0);    //rev/s
            Constraint->SetLinearPositionTarget(PositionTarget + PErrInt);
            Constraint->SetAngularOrientationTarget(GetOrientationTargetFromEuler(orientationTargetEuler + AErrInt));
        }
    }
    else if (ControlType == ERRJointControlType::VELOCITY)
    {
        // Linear update of velocity
        for (i = 0; i < 3; i++)
        {
            URRMathUtils::StepUpdate(MidLinearVelocityTarget[i],
                                     LinearVelocityTarget[i],
                                     LinearVelocitySmoothingAcc * DeltaTime,
                                     LinearVelocityTolerance);
            URRMathUtils::StepUpdate(MidAngularVelocityTarget[i],
                                     AngularVelocityTarget[i],
                                     AngularVelocitySmoothingAcc * DeltaTime,
                                     AngularVelocityTolerance);
        }

        if (!HasReachedVelocityTarget(LinearVelocityTolerance, AngularVelocityTolerance) && bSmoothing)
        {
            Constraint->SetLinearVelocityTarget(MidLinearVelocityTarget);
            Constraint->SetAngularVelocityTarget(MidAngularVelocityTarget / 360.0);
        }
    }

#if RAPYUTA_JOINT_DEBUG
    if (bSmoothing)
    {
        if (RotationalDOF > 0)
        {
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("Smoothing mid: %s | %s"),
                   *MidAngularVelocityTarget.ToString(),
                   *MidOrientationTarget.Euler().ToString());
        }
        if (LinearDOF > 0)
        {
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("Smoothing mid: %s | %s"),
                   *MidLinearVelocityTarget.ToString(),
                   *MidPositionTarget.ToString());
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

void URRPhysicsJointComponent::Teleport(const FVector& InPosition, const FRotator& InOrientation)
{
    Super::Teleport(InPosition, InOrientation);

    //! @note K2_SetWorldTransform not works with physics constraints
    // FHitResult SweepHitResult;
    // K2_SetWorldTransform(FTransform(Orientation, Position) *  // joint changes
    //                      Constraint->GetComponentTransform(),             // world orogin to parent
    //                      true,                                // bSweep
    //                      SweepHitResult,
    //                      false    // bTeleport
    UE_LOG_WITH_INFO_SHORT_NAMED(LogRapyutaCore, Warning, TEXT("Teleport is not supported with Physics joint"));
};

void URRPhysicsJointComponent::MoveToInitPose()
{
    SetPoseTarget(InitialPosition, InitialOrientation);
}
