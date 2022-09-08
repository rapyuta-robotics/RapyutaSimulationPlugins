// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Drives/RRTricycleDriveComponent.h"

// RapyutaRobotImporter
//#include "Core/RRSkeletalMeshComponent.h"
//#include "Robot/RRSkeletalRobot.h"
#include "WheeledVehiclePawn.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "PhysicsEngine/ConstraintInstanceBlueprintLibrary.h"
#include "Robots/RobotVehicle.h"
#include "Core/RRConversionUtils.h"

void URRTricycleDriveComponent::Setup()
{
    ARobotVehicle* vehicle = Cast<ARobotVehicle>(GetOwner());
    SkeletalMeshComponent = vehicle->GetMesh();

    // TODO: remove after fixing spawn. Change to model info joint names.
    JointsStates.Add("tilt_base_c", -5.0f);
    JointsStates.Add("reach_base_c", -10.0f);
    JointsStates.Add("fork_base_c", 0.0872665f);
        
    for(const auto& joint : vehicle->Joints)
    {
        JointsStates.Add(joint.Key, 0.0f);
    }
}

void URRTricycleDriveComponent::UpdateMovement(float DeltaTime)
{
    bool isVelocityChanged = !Velocity.Equals(VelocityCurrent);
    
    if (isVelocityChanged)
    {
        VelocityCurrent = Velocity;
    }

    bool isTransformChanged = false;
    
    for(const auto& jointState : JointsStates)
    {
        float* stateValue = JointsStatesCurrent.Find(jointState.Key);
        
        if(!stateValue || !FMath::IsNearlyEqual(jointState.Value, *stateValue))
        {
            if(stateValue)
            {
                JointsStatesCurrent[jointState.Key] = jointState.Value;
            }
            else
            {
                JointsStatesCurrent.Add(jointState.Key, jointState.Value);
            }

            isTransformChanged = true;
        }
    }

    bool isBackwardMove = VelocityCurrent.X > 0.0f && ChaosMovementComponent->GetForwardSpeed() <= 0.0f;
    bool isAngularChanged = !FMath::IsNearlyZero(AngularVelocity.Z);
    
    if(isVelocityChanged || isTransformChanged || isBackwardMove || isAngularChanged)
    {
        ChaosMovementComponent->SetBrakeInput(VelocityCurrent.X < 0.0f ? 1.0f : 0.0f);
        bool isSpeedEnoughForThrottle = VelocityCurrent.X > 0.0f && ChaosMovementComponent->GetForwardSpeed() >= -1.0f;
        ChaosMovementComponent->SetThrottleInput(isSpeedEnoughForThrottle ? 1.0f : 0.0f);
        bool IsNeedToStopMoving = FMath::IsNearlyZero(VelocityCurrent.X);
        ChaosMovementComponent->SetMaxEngineTorque(IsNeedToStopMoving ? 0.0f : MaxEngineTorque);
        SteerInputCurrent = FMath::Clamp(SteerInputCurrent - DeltaTime * AngularVelocity.Z, -1.0f, 1.0f);
        ChaosMovementComponent->SetSteeringInput(SteerInputCurrent);
        //float rpmFactor = URRConversionUtils::VectorUEToROS(VelocityCurrent).X;
        //ChaosMovementComponent->EngineSetup.MaxRPM = rpmFactor * MaxRPM;
        
        for(const auto& jointState : JointsStatesCurrent)
        {
            // currently robot joints are always have 1D motion space, so we trying to understand is it translation or rotation and in what axis
            FConstraintInstanceAccessor constraintInstanceAccessor = SkeletalMeshComponent->GetConstraintByName(*jointState.Key, false);

            if(FConstraintInstance* constraintInstance = constraintInstanceAccessor.Get())
            {
                bool isLinearXMotion = constraintInstance->GetLinearXMotion() != ELinearConstraintMotion::LCM_Locked;
                bool isLinearYMotion = constraintInstance->GetLinearYMotion() != ELinearConstraintMotion::LCM_Locked;
                bool isLinearZMotion = constraintInstance->GetLinearZMotion() != ELinearConstraintMotion::LCM_Locked;

                if(isLinearXMotion || isLinearYMotion || isLinearZMotion)
                {
                    FVector linearTarget;

                    if(isLinearXMotion)
                    {
                        linearTarget.X = jointState.Value;
                    }
                    else if(isLinearYMotion)
                    {
                        linearTarget.Y = jointState.Value;
                    }
                    else if(isLinearZMotion)
                    {
                        linearTarget.Z = jointState.Value;
                    }

                    UConstraintInstanceBlueprintLibrary::SetLinearPositionTarget(constraintInstanceAccessor, linearTarget);
                }

                bool isAngularTwistMotion = constraintInstance->GetAngularTwistMotion() != EAngularConstraintMotion::ACM_Locked;
                bool isAngularSwing2Motion = constraintInstance->GetAngularSwing2Motion() != EAngularConstraintMotion::ACM_Locked;
                bool isAngularSwing1Motion = constraintInstance->GetAngularSwing1Motion() != EAngularConstraintMotion::ACM_Locked;

                if(isAngularTwistMotion || isAngularSwing2Motion || isAngularSwing1Motion)
                {
                    FRotator angularTarget;

                    if(isAngularTwistMotion)
                    {
                        angularTarget.Yaw =  FMath::RadiansToDegrees(jointState.Value);
                    }
                    else if(isAngularSwing2Motion)
                    {
                        angularTarget.Pitch =  FMath::RadiansToDegrees(jointState.Value);
                    }
                    else if(isAngularSwing1Motion)
                    {
                        angularTarget.Roll =  FMath::RadiansToDegrees(jointState.Value);
                    }

                    UConstraintInstanceBlueprintLibrary::SetAngularOrientationTarget(constraintInstanceAccessor, angularTarget);
                }
            }
        }

        bool anyInput = ChaosMovementComponent->GetBrakeInput() > 0.0f || ChaosMovementComponent->GetThrottleInput() > 0.0f;
        
        if((isAngularChanged || isTransformChanged) && !anyInput)
        {
            ChaosMovementComponent->SetThrottleInput(1.0f);
            ChaosMovementComponent->SetBrakeInput(1.0f);
            //ChaosMovementComponent->StopMovementImmediately();
        }
    }
}