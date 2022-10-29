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
    // Generally, for sake of dynamic robot type import/creation, child components would be then created on the fly!
    // Besides, a default subobject, upon content changes, also makes the owning actor become vulnerable since one in child BP actor
    // classes will automatically get invalidated.

    // [SkeletalMeshComp], due to the support for being configured in certain [ARobotVehicle]'s BP classes,
    // needs to be created in ctor.
    // Reference: AWheeledVehiclePawn
    SkeletalMeshComp = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("SkeletalMeshComp"));
    SkeletalMeshComp->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPose;
    SkeletalMeshComp->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    SkeletalMeshComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    SkeletalMeshComp->SetCanEverAffectNavigation(true);
    SkeletalMeshComp->SetIsReplicated(true);
    AddOwnedComponent(SkeletalMeshComp);
    RootComponent = SkeletalMeshComp;

    AIControllerClass = ARRRobotVehicleROSController::StaticClass();
}

void ARobotVehicle::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(ARobotVehicle, SkeletalMeshComp);
}
