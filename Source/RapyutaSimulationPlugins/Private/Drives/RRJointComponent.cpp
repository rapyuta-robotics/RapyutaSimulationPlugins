// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/RRJointComponent.h"

// Sets default values for this component's properties
URRJointComponent::URRJointComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    bWantsInitializeComponent = true;
}

bool URRJointComponent::IsValid()
{
    return ChildLink && ParentLink;
}

void URRJointComponent::InitializeComponent()
{
    ResetControl();
}

void URRJointComponent::ResetControl()
{
    bMovingToTargetPose = false;
    bMovingToTargetVelocity = false;
    OnControlSuccessDelegate.Unbind();
    OnControlFailDelegate.Unbind();
}

void URRJointComponent::SetDelegates(const FJointCallback& InOnControlSuccessDelegate,
                                     const FJointCallback& InOnControlFailDelegate,
                                     const float InPositionTolerance,
                                     const float InOrientationTolerance,
                                     const float InLinearTolerance,
                                     const float InAngularTolerance,
                                     const float InTimeOut)
{
    if (!InOnControlSuccessDelegate.IsBound())
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Control Success Delegate is not set - is this on purpose? "));
    }
    else
    {
        OnControlSuccessDelegate.Unbind();
        OnControlSuccessDelegate = InOnControlSuccessDelegate;
    }

    if (!InOnControlFailDelegate.IsBound())
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Control Fail Delegate is not set - is this on purpose? "));
    }
    else
    {
        OnControlFailDelegate.Unbind();
        OnControlFailDelegate = InOnControlFailDelegate;
    }

    PositionTolerance = (InPositionTolerance >= 0.f) ? InPositionTolerance : PositionTolerance;
    OrientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;

    LinearVelocityTolerance = (InLinearTolerance >= 0.f) ? InLinearTolerance : LinearVelocityTolerance;
    AngularVelocityTolerance = (InAngularTolerance >= 0.f) ? InAngularTolerance : AngularVelocityTolerance;

    ControlStartTime = GetWorld()->GetTimeSeconds();
    ControlTimeout = InTimeOut;
};

// velocity
void URRJointComponent::SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    ControlType = ERRJointControlType::VELOCITY;
    LinearVelocityTarget = InLinearVelocity.BoundToBox(-LinearVelMax, LinearVelMax);
    AngularVelocityTarget = InAngularVelocity.BoundToBox(-AngularVelMax, AngularVelMax);
    ResetControl();
    bMovingToTargetVelocity = true;
};

void URRJointComponent::SetSingleLinearVelocityTarget(const float Input)
{
    SetVelocityTarget(FVector(Input, LinearVelocity.Y, LinearVelocity.Z), AngularVelocity);
};

void URRJointComponent::SetSingleAngularVelocityTarget(const float Input)
{
    SetVelocityTarget(LinearVelocity, FVector(Input, AngularVelocity.Y, AngularVelocity.Z));
};

void URRJointComponent::SetVelocityTargetWithDelegates(const FVector& InLinearVelocity,
                                                       const FVector& InAngularVelocity,
                                                       const FJointCallback& InOnControlSuccessDelegate,
                                                       const FJointCallback& InOnControlFailDelegate,
                                                       const float InLinearTolerance,
                                                       const float InAngularTolerance,
                                                       const float InTimeOut)
{
    SetVelocityTarget(InLinearVelocity, InAngularVelocity);
    SetDelegates(InOnControlSuccessDelegate, InOnControlFailDelegate, -1.0, -1.0, InLinearTolerance, InAngularTolerance, InTimeOut);
}

void URRJointComponent::SetSingleLinearVelocityTargetWithDelegates(const float Input,
                                                                   const FJointCallback& InOnControlSuccessDelegate,
                                                                   const FJointCallback& InOnControlFailDelegate,
                                                                   const float InLinearTolerance,
                                                                   const float InTimeOut)
{
    SetVelocityTargetWithDelegates(FVector(Input, LinearVelocity.Y, LinearVelocity.Z),
                                   AngularVelocity,
                                   InOnControlSuccessDelegate,
                                   InOnControlFailDelegate,
                                   InLinearTolerance,
                                   -1.0,
                                   InTimeOut);
}

void URRJointComponent::SetSingleAngularVelocityTargetWithDelegates(const float Input,
                                                                    const FJointCallback& InOnControlSuccessDelegate,
                                                                    const FJointCallback& InOnControlFailDelegate,
                                                                    const float InAngularTolerance,
                                                                    const float InTimeOut)
{
    SetVelocityTargetWithDelegates(LinearVelocity,
                                   FVector(Input, AngularVelocity.Y, AngularVelocity.Z),
                                   InOnControlSuccessDelegate,
                                   InOnControlFailDelegate,
                                   -1.0,
                                   InAngularTolerance,
                                   InTimeOut);
}

bool URRJointComponent::HasReachedVelocityTarget(const float InLinearTolerance, const float InAngularTolerance)
{
    if (!bMovingToTargetVelocity)
    {
        return true;
    }

    const float linearTolerance = (InLinearTolerance >= 0.f) ? InLinearTolerance : LinearVelocityTolerance;
    const float angularTolerance = (InAngularTolerance >= 0.f) ? InAngularTolerance : AngularVelocityTolerance;

    bool res = LinearVelocityTarget.Equals(LinearVelocity, linearTolerance) &&
               AngularVelocityTarget.Equals(AngularVelocity, angularTolerance);

    bMovingToTargetVelocity = !res;
    return res;
};

void URRJointComponent::SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    LinearVelocity = InLinearVelocity.BoundToBox(-LinearVelMax, LinearVelMax);
    AngularVelocity = InAngularVelocity.BoundToBox(-AngularVelMax, AngularVelMax);
};

void URRJointComponent::VelocityFromArray(const TArray<float>& InVelocity, FVector& OutLinearVelocity, FVector& OutAngularVelocity)
{
    if (InVelocity.Num() != LinearDOF + RotationalDOF)
    {
        UE_LOG_WITH_INFO_NAMED(LogTemp,
                               Warning,
                               TEXT("Given joint command num is not much with joint DOF. Linear DOF %i and Rotational DOF %i"),
                               LinearDOF,
                               RotationalDOF);
        return;
    }

    uint8 i;
    FVector LinearInput = FVector::ZeroVector;
    for (i = 0; i < LinearDOF; i++)
    {
        LinearInput[i] = InVelocity[i];
    }

    FVector AngularInput = FVector::ZeroVector;
    for (i = 0; i < RotationalDOF; i++)
    {
        AngularInput[i] = InVelocity[LinearDOF + i];
    }

    OutLinearVelocity = LinearInput;
    OutAngularVelocity = AngularInput;
};

void URRJointComponent::SetVelocityTargetWithArray(const TArray<float>& InVelocity)
{
    FVector OutLinearVelocity;
    FVector OutAngularVelocity;
    VelocityFromArray(InVelocity, OutLinearVelocity, OutAngularVelocity);
    SetVelocityTarget(OutLinearVelocity, OutAngularVelocity);
}

void URRJointComponent::SetVelocityTargetWithArrayWithDelegates(const TArray<float>& InVelocity,
                                                                const FJointCallback& InOnControlSuccessDelegate,
                                                                const FJointCallback& InOnControlFailDelegate,
                                                                const float InLinearTolerance,
                                                                const float InAngularTolerance,
                                                                const float InTimeOut)
{
    FVector OutLinearVelocity;
    FVector OutAngularVelocity;
    VelocityFromArray(InVelocity, OutLinearVelocity, OutAngularVelocity);
    SetVelocityTargetWithDelegates(OutLinearVelocity,
                                   OutAngularVelocity,
                                   InOnControlSuccessDelegate,
                                   InOnControlFailDelegate,
                                   InLinearTolerance,
                                   InAngularTolerance,
                                   InTimeOut);
}

void URRJointComponent::SetVelocityWithArray(const TArray<float>& InVelocity)
{
    FVector OutLinearVelocity;
    FVector OutAngularVelocity;
    VelocityFromArray(InVelocity, OutLinearVelocity, OutAngularVelocity);
    SetVelocity(OutLinearVelocity, OutAngularVelocity);
}

//Pose
void URRJointComponent::SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation)
{
    ControlType = ERRJointControlType::POSITION;
    PositionTarget = InPosition.BoundToBox(PositionMin, PositionMax);
    OrientationTarget =
        FRotator(bLimitPitch ? FMath::Clamp(InOrientation.Pitch, OrientationMin.Pitch, OrientationMax.Pitch) : InOrientation.Pitch,
                 bLimitYaw ? FMath::Clamp(InOrientation.Yaw, OrientationMin.Yaw, OrientationMax.Yaw) : InOrientation.Yaw,
                 bLimitRoll ? FMath::Clamp(InOrientation.Roll, OrientationMin.Roll, OrientationMax.Roll) : InOrientation.Roll);
    ResetControl();
    bMovingToTargetPose = true;
};

void URRJointComponent::SetSinglePositionTarget(const float Input)
{
    SetPoseTarget(FVector(Input, Position.Y, Position.Z), Orientation);
}

void URRJointComponent::SetSingleOrientationTarget(const float Input)
{
    SetPoseTarget(Position, FRotator(Orientation.Pitch, Orientation.Yaw, Input));
}

void URRJointComponent::SetPoseTargetWithDelegates(const FVector& InPosition,
                                                   const FRotator& InOrientation,
                                                   const FJointCallback& InOnControlSuccessDelegate,
                                                   const FJointCallback& InOnControlFailDelegate,
                                                   const float InPositionTolerance,
                                                   const float InOrientationTolerance,
                                                   const float InTimeOut)
{
    SetPoseTarget(InPosition, InOrientation);
    SetDelegates(
        InOnControlSuccessDelegate, InOnControlFailDelegate, InPositionTolerance, InOrientationTolerance, -1.0, -1.0, InTimeOut);
}

void URRJointComponent::SetSinglePositionTargetWithDelegates(const float Input,
                                                             const FJointCallback& InOnControlSuccessDelegate,
                                                             const FJointCallback& InOnControlFailDelegate,
                                                             const float InPositionTolerance,
                                                             const float InTimeOut)
{
    SetPoseTargetWithDelegates(FVector(Input, Position.Y, Position.Z),
                               Orientation,
                               InOnControlSuccessDelegate,
                               InOnControlFailDelegate,
                               InPositionTolerance,
                               -1.0,
                               InTimeOut);
}

void URRJointComponent::SetSingleOrientationTargetWithDelegates(const float Input,
                                                                const FJointCallback& InOnControlSuccessDelegate,
                                                                const FJointCallback& InOnControlFailDelegate,
                                                                const float InOrientationTolerance,
                                                                const float InTimeOut)
{
    SetPoseTargetWithDelegates(Position,
                               FRotator(Orientation.Pitch, Orientation.Yaw, Input),
                               InOnControlSuccessDelegate,
                               InOnControlFailDelegate,
                               -1.0,
                               InOrientationTolerance,
                               InTimeOut);
}

bool URRJointComponent::HasReachedPoseTarget(const float InPositionTolerance, const float InOrientationTolerance)
{
    if (!bMovingToTargetPose)
    {
        return true;
    }

    const float positionTolerance = (InPositionTolerance >= 0.f) ? InPositionTolerance : PositionTolerance;
    const float orientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;
    bool res = PositionTarget.Equals(Position, positionTolerance) && OrientationTarget.Equals(Orientation, orientationTolerance);

#if RAPYUTA_JOINT_DEBUG
    UE_LOG_WITH_INFO_SHORT_NAMED(
        LogTemp, Warning, TEXT("%d %f %s %s"), res, positionTolerance, *PositionTarget.ToString(), *Position.ToString());
    UE_LOG_WITH_INFO_SHORT_NAMED(
        LogTemp, Warning, TEXT("%d %f %s %s"), res, orientationTolerance, *OrientationTarget.ToString(), *Orientation.ToString());
#endif

    if (res)
    {
        bMovingToTargetPose = false;
        if (OnControlSuccessDelegate.IsBound())
        {
            OnControlSuccessDelegate.ExecuteIfBound();
        }
    }
    else
    {
        // if target is set from SetPoseTargetWithDelegate
        if (OnControlFailDelegate.IsBound())
        {
            float currentTime = GetWorld()->GetTimeSeconds();
            if (ControlTimeout >= 0 && currentTime - ControlStartTime > ControlTimeout)
            {
                bMovingToTargetPose = false;
                OnControlFailDelegate.ExecuteIfBound();
            }
        }
    }
    return res;
};

void URRJointComponent::SetPose(const FVector& InPosition, const FRotator& InOrientation)
{
    Position = InPosition.BoundToBox(PositionMin, PositionMax);
    Orientation =
        FRotator(bLimitPitch ? FMath::Clamp(InOrientation.Pitch, OrientationMin.Pitch, OrientationMax.Pitch) : InOrientation.Pitch,
                 bLimitYaw ? FMath::Clamp(InOrientation.Yaw, OrientationMin.Yaw, OrientationMax.Yaw) : InOrientation.Yaw,
                 bLimitRoll ? FMath::Clamp(InOrientation.Roll, OrientationMin.Roll, OrientationMax.Roll) : InOrientation.Roll);
};

void URRJointComponent::PoseFromArray(const TArray<float>& InPose, FVector& OutPosition, FRotator& OutOrientation)
{
    if (InPose.Num() != LinearDOF + RotationalDOF)
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore,
            Warning,
            TEXT("Given joint pose values num (%u) does not match joint total DOF (Linear DOF %i & Rotational DOF %i)"),
            InPose.Num(),
            LinearDOF,
            RotationalDOF);
        return;
    }

    uint8 i;
    FVector LinearInput = FVector(0, 0, 0);
    for (i = 0; i < LinearDOF; i++)
    {
        LinearInput[i] = InPose[i];
    }

    FVector RotationalInput = FVector(0, 0, 0);
    for (i = 0; i < RotationalDOF; i++)
    {
        RotationalInput[i] = InPose[LinearDOF + i];
    }

    OutPosition = LinearInput;
    OutOrientation = FRotator::MakeFromEuler(RotationalInput);
};

void URRJointComponent::SetPoseTargetWithArray(const TArray<float>& InPose)
{
    FVector OutPosition;
    FRotator OutOrientation;
    PoseFromArray(InPose, OutPosition, OutOrientation);
    SetPoseTarget(OutPosition, OutOrientation);
}

void URRJointComponent::SetPoseTargetWithArrayWithDelegates(const TArray<float>& InPose,
                                                            const FJointCallback& InOnControlSuccessDelegate,
                                                            const FJointCallback& InOnControlFailDelegate,
                                                            const float InPositionTolerance,
                                                            const float InOrientationTolerance,
                                                            const float InTimeOut)
{
    FVector OutPosition;
    FRotator OutOrientation;
    PoseFromArray(InPose, OutPosition, OutOrientation);
    SetPoseTargetWithDelegates(OutPosition,
                               OutOrientation,
                               InOnControlSuccessDelegate,
                               InOnControlFailDelegate,
                               InPositionTolerance,
                               InOrientationTolerance,
                               InTimeOut);
}

void URRJointComponent::SetPoseWithArray(const TArray<float>& InPose)
{
    FVector OutPosition;
    FRotator OutOrientation;
    PoseFromArray(InPose, OutPosition, OutOrientation);
    SetPose(OutPosition, OutOrientation);
}

void URRJointComponent::Teleport(const FVector& InPosition, const FRotator& InOrientation)
{
}

void URRJointComponent::MoveToInitPose()
{
}

void URRJointComponent::UpdateState(const float DeltaTime)
{
}

void URRJointComponent::UpdateControl(const float DeltaTime)
{
}

// Called every frame
void URRJointComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (IsValid())
    {
        UpdateState(DeltaTime);
        UpdateControl(DeltaTime);

        HasReachedVelocityTarget();
        HasReachedPoseTarget();
    }
}
