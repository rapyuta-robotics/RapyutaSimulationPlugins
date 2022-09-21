// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotVehicle.h"

#include "Net/UnrealNetwork.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RRRobotVehicleROSController.h"

ARobotVehicle::ARobotVehicle()
{
    SetupDefaultRootSkeletal();
}

ARobotVehicle::ARobotVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefaultRootSkeletal();
}

void ARobotVehicle::SetupDefaultRootSkeletal()
{
    SkeletalMeshWrapper.Init(GetMesh());
    SkeletalMeshWrapper.SkeletalMeshComponent->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPose;
    SkeletalMeshWrapper.SkeletalMeshComponent->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);

    AIControllerClass = ARRRobotVehicleROSController::StaticClass();
}

void ARobotVehicle::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    
   // DOREPLIFETIME(ARobotVehicle, SkeletalMeshComp);
}
