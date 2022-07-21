// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/KinematicTricycleDriveComponent.h"

DEFINE_LOG_CATEGORY(LogKinematicTricycleDriveComponent);
void UKinematicTricycleDriveComponent::TickComponent(float InDeltaTime,
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

            float driveDiff = (wheelAngle - PrevWheeAngle) * WheelRadius;
            const FVector linearDiff = FVector(driveDiff * FMath::Cos(steerAngle), 0, 0);
            const FQuat rotationDiff = FQuat(FVector(0, 0, 1), driveDiff * FMath::Sin(steerAngle) / WheelBase);

            FHitResult hit;
            UpdatedComponent->AddLocalTransform(FTransform(rotationDiff, linearDiff), true, &hit);

            PrevWheeAngle = wheelAngle;
        }
        else
        {
            UE_LOG(LogKinematicTricycleDriveComponent,
                   Warning,
                   TEXT("[%s] [UKinematicTricycleDriveComponent] Steering and/or drive joints are not speciied"),
                   *GetName());
        }
    }
}

void UKinematicTricycleDriveComponent::SetDriveJoints(URRJointComponent* InSteeringJoint, URRJointComponent* InDriveJoint)
{
    SteeringJoint = InSteeringJoint;
    DriveJoint = InDriveJoint;
}

void UKinematicTricycleDriveComponent::BeginPlay()
{
    if (SteeringJoint != nullptr && DriveJoint != nullptr)
    {
        PrevWheeAngle = FMath::DegreesToRadians(DriveJoint->Orientation.Roll);
    }
    else
    {
        UE_LOG(LogKinematicTricycleDriveComponent,
               Warning,
               TEXT("[%s] [UKinematicTricycleDriveComponent] Steering and/or drive joints are not speciied"),
               *GetName());
    }
    Super::BeginPlay();
}
