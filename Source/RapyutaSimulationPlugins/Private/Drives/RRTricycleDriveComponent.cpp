// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Drives/RRTricycleDriveComponent.h"

// RapyutaRobotImporter
//#include "Core/RRSkeletalMeshComponent.h"
//#include "Robot/RRSkeletalRobot.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/PoseableMeshComponent.h"

void URRTricycleDriveComponent::Setup()
{
    /*
    SkeletalRobot = CastChecked<ARRSkeletalRobot>(GetOwner());
    PoseableMeshComponent = CastChecked<UPoseableMeshComponent>(SkeletalRobot->GetComponentByClass(UPoseableMeshComponent::StaticClass()));
    //int32 DriveWheelBoneIndex = skeletalMeshComponent->GetBoneIndex(FName(DriveWheelName));
    //skeletalMeshComponent->GetBoneSpaceTransforms()[DriveWheelBoneIndex];
    //skeletalMeshComponent->SetBoneRotationByName()
    
    for (UPhysicsConstraintTemplate* t : PoseableMeshComponent->GetPhysicsAsset()->ConstraintSetup)
    {
        if (t->DefaultInstance.JointName.ToString() == DriveWheelName)
        {
            DriveWheelCI = &t->DefaultInstance;
               
            DriveWheelCI->SetAngularDriveMode(EAngularDriveMode::Type::TwistAndSwing);
            DriveWheelCI->SetAngularVelocityDriveTwistAndSwing(true, true);
            DriveWheelCI->SetLinearDriveParams(1000.0f, 1000.0f, 10000000.0f);
           // DriveWheelCI->SetAngularDriveParams(100000.0f, 100000.0f, 10000000.0f);
            //DriveWheelCI->SetAngularVelocityTarget(FVector::ZeroVector);
            //DriveWheelCI->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 180.0f);
            //DriveWheelCI->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 180.0f);
            //DriveWheelCI->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 180.0f);
            //DriveWheelCI->SetAngularDriveMode(EAngularDriveMode::Type::SLERP);
            //DriveWheelCI->SetOrientationDriveSLERP(true);
            //DriveWheelCI->SetAngularVelocityDriveSLERP(true);

            t->SetDefaultProfile(*DriveWheelCI);
            
            break;
        }
    }

    PoseableMeshComponent->RecreatePhysicsState();
    //DriveWheelCI = PoseableMeshComponent->FindConstraintInstance(*DriveWheelName);

    
    //FBodyInstance* bodyInstance = PoseableMeshComponent->GetBodyInstance(*DriveWheelName);
    //bodyInstance->GetUnrealWorldTransform();
    
    UpdateFromCurrent();
    */
}

// void URRTricycleDriveComponent::SetBoneTransformByName(FName BoneName, const FTransform& InTransform, EBoneSpaces::Type BoneSpace)
// {
//     // from UPoseableMeshComponent::SetBoneTransformByName
//     int32 BoneIndex = PoseableMeshComponent->GetBoneIndex(BoneName);
//     PoseableMeshComponent->BoneSpaceTransforms[BoneIndex] = FTransform::Identity; //InTransform;
//     //
//     // if(BoneIndex >=0 && BoneIndex < SkeletalMeshComponent->BoneSpaceTransforms.Num())
//     // {
//     //
//     //     if(BoneSpace == EBoneSpaces::WorldSpace)
//     //     {
//     //         SkeletalMeshComponent->BoneSpaceTransforms[BoneIndex].SetToRelativeTransform(SkeletalMeshComponent->GetComponentToWorld());
//     //     }
//     //
//     //     // int32 ParentIndex = SkeletalMeshComponent->RequiredBones.GetParentBoneIndex(BoneIndex);
//     //     //
//     //     // if(ParentIndex >=0)
//     //     // {
//     //     //     FA2CSPose CSPose;
//     //     //     CSPose.AllocateLocalPoses(SkeletalMeshComponent->RequiredBones, SkeletalMeshComponent->BoneSpaceTransforms);
//     //     //
//     //     //     SkeletalMeshComponent->BoneSpaceTransforms[BoneIndex].SetToRelativeTransform(CSPose.GetComponentSpaceTransform(ParentIndex));
//     //     // }
//     // }
// }

void URRTricycleDriveComponent::UpdateFromCurrent()
{
    if(DriveWheelCI)
    {
        /*
        PoseableMeshComponent->SetBoneRotationByName(*DriveWheelName, FRotator(0.0f, 45.0f * AngularVelocity.Z, 0.0f), EBoneSpaces::ComponentSpace);

        DriveWheelCI->SetAngularVelocityTarget(FVector(-VelocityCurrent.X, 0.f, 0.0f));
*/


        //DriveWheelCI->SetAngularOrientationTarget(FQuat::MakeFromEuler(FVector(0.0f, 45.0f * AngularVelocity.Z, 0.0f)));

        // if(FMath::IsNearlyZero(AngularVelocity.Z))
        // {
        //     if(DriveWheelCI->GetAngularSwing2Motion() == EAngularConstraintMotion::ACM_Limited)
        //     {
        //         DriveWheelCI->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 45.0f);
        //     }
        // }
        // else
        // {
        //     if(DriveWheelCI->GetAngularSwing2Motion() == EAngularConstraintMotion::ACM_Locked)
        //     {
        //         DriveWheelCI->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 45.0f);
        //     }
        // }
        
        DriveWheelCI->SetAngularVelocityDriveTwistAndSwing(true, false);

        
        /*
        USkeletalMeshComponent* skeletalMeshComponent = CastChecked<USkeletalMeshComponent>(SkeletalRobot->GetComponentByClass(USkeletalMeshComponent::StaticClass()));
        
        //int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(*DriveWheelName);
        //SkeletalMeshComponent->BoneSpaceTransforms[BoneIndex] = FTransform::Identity; //InTransform;
        //SkeletalMeshComponent->bRequiredBonesUpToDate = false;
        FBodyInstance* bodyInstance = SkeletalMeshComponent->GetBodyInstance(*DriveWheelName);
       // bodyInstance->GetRelativeBodyTransform();
        FTransform newTransform = FTransform::Identity;
        newTransform.SetRotation(FQuat(FRotator(30.0f, 0.0f, 0.0f)));
        bodyInstance->SetBodyTransform(newTransform, ETeleportType::TeleportPhysics);
        //DriveWheelCI->SetAngularVelocityTarget(FVector(0.0f, 0.f, 0.0f));
        //DriveWheelCI->SetAngularVelocityDriveTwistAndSwing(false, true);
        */
    }
}

void URRTricycleDriveComponent::UpdateMovement(float DeltaTime)
{
    if (!Velocity.Equals(VelocityCurrent) || !AngularVelocityCurrent.Equals(AngularVelocity))
    {
        VelocityCurrent = Velocity;
        AngularVelocityCurrent = AngularVelocity;

        ChaosMovementComponent->SetMaxEngineTorque(VelocityCurrent.X);
        ChaosMovementComponent->SetThrottleInput(FMath::IsNearlyZero(VelocityCurrent.X) ? 0.0f : 1.0f);

        if (!FMath::IsNearlyZero(AngularVelocityCurrent.Z))
        {
            float steeringDeg = FMath::Clamp(FMath::RadiansToDegrees(AngularVelocityCurrent.Z), -90.0f, 90.0f);
            ChaosMovementComponent->SetSteeringInput(steeringDeg/90.0f);
        }
    }
}