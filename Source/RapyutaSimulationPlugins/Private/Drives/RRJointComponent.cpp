// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/RRJointComponent.h"

// Sets default values for this component's properties
URRJointComponent::URRJointComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void URRJointComponent::BeginPlay()
{
    Initialize();
    Super::BeginPlay();
}

bool URRJointComponent::IsValid()
{
    return ChildLink && ParentLink;
}

void URRJointComponent::Initialize()
{
    InitTF();
}

// velocity
void URRJointComponent::SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity)
{
    ControlType = ERRJointControlType::VELOCITY;
    LinearVelocityTarget = InLinearVelocity.BoundToBox(-LinearVelMax, LinearVelMax);
    AngularVelocityTarget = InAngularVelocity.BoundToBox(-AngularVelMax, AngularVelMax);
};

bool URRJointComponent::HasReachedVelocityTarget(const float InLinearTolerance, const float InAngularTolerance)
{
    const float linearTolerance = (InLinearTolerance >= 0.f) ? InLinearTolerance : LinearVelocityTolerance;
    const float angularTolerance = (InAngularTolerance >= 0.f) ? InAngularTolerance : AngularVelocityTolerance;

    return LinearVelocityTarget.Equals(LinearVelocity, linearTolerance) &&
           AngularVelocityTarget.Equals(AngularVelocity, angularTolerance);
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
};

bool URRJointComponent::HasReachedPoseTarget(const float InPositionTolerance, const float InOrientationTolerance)
{
    const float positionTolerance = (InPositionTolerance >= 0.f) ? InPositionTolerance : PositionTolerance;
    const float orientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;
    return PositionTarget.Equals(Position, positionTolerance) && OrientationTarget.Equals(Orientation, orientationTolerance);
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

void URRJointComponent::InitTF()
{
    if (bPublishTF)
    {
        TFPublisher = NewObject<URRROS2JointTFPublisher>(this);
        TFPublisher->JointToChildLink = JointToChildLink;
        TFPublisher->ParentLinkToJoint = ParentLinkToJoint;
    }
}

void URRJointComponent::UpdateTF()
{
    if (bPublishTF)
    {
        TFPublisher->JointTF = FTransform(Orientation, Position, FVector::OneVector);
    }
}
