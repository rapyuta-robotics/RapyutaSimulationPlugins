// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RRKinematicTricycleDriveComponent.h"

DEFINE_LOG_CATEGORY(LogRRKinematicTricycleDriveComponent);
void URRKinematicTricycleDriveComponent::TickComponent(float InDeltaTime,
                                                       enum ELevelTick TickType,
                                                       FActorComponentTickFunction* ThisTickFunction)
{
    if (!ShouldSkipUpdate(InDeltaTime))
    {
        Super::TickComponent(InDeltaTime, TickType, ThisTickFunction);

        if (SteeringJoint != nullptr && DriveJoint != nullptr)
        {
            float steerAngle = FMath::DegreesToRadians(SteeringJoint->Orientation.Roll);
            float wheelAngle = FMath::DegreesToRadians(DriveJoint->Orientation.Roll);

            float driveDiff = (wheelAngle - PrevWheeAngleRad) * WheelRadius;
            const FVector linearDiff = FVector(driveDiff * FMath::Cos(steerAngle), 0, 0);
            const FQuat rotationDiff = FQuat(FVector(0, 0, 1), driveDiff * FMath::Sin(steerAngle) / WheelBase);

            FHitResult hit;
            UpdatedComponent->AddLocalTransform(FTransform(rotationDiff, linearDiff), true, &hit);

            PrevWheeAngleRad = wheelAngle;
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(
                LogRRKinematicTricycleDriveComponent, Warning, TEXT("Steering and/or drive joints are not speciied"));
        }
    }
}

void URRKinematicTricycleDriveComponent::SetDriveJoints(URRJointComponent* InSteeringJoint, URRJointComponent* InDriveJoint)
{
    SteeringJoint = InSteeringJoint;
    DriveJoint = InDriveJoint;
}

void URRKinematicTricycleDriveComponent::BeginPlay()
{
    if (SteeringJoint != nullptr && DriveJoint != nullptr)
    {
        PrevWheeAngleRad = FMath::DegreesToRadians(DriveJoint->Orientation.Roll);
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(
            LogRRKinematicTricycleDriveComponent, Warning, TEXT("Steering and/or drive joints are not speciied"));
    }
    Super::BeginPlay();
}
