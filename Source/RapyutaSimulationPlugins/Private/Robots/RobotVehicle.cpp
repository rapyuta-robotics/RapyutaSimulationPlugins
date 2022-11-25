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
    // needs to be created in ctor, thus its name must be also a constant literal.
    // Reference: AWheeledVehiclePawn
    SkeletalMeshComp = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("SkeletalMeshComp"));
    SkeletalMeshComp->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
    SkeletalMeshComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    SkeletalMeshComp->SetCanEverAffectNavigation(true);
    SkeletalMeshComp->SetIsReplicated(true);
    SkeletalMeshComp->SetGenerateOverlapEvents(true);
    AddOwnedComponent(SkeletalMeshComp);
    // [SkeletalMeshComp] -> NEW ROOT
    // This is in ctor, thus no need to use [SetRootComponent()]
    RootComponent = SkeletalMeshComp;

    AIControllerClass = ARRRobotVehicleROSController::StaticClass();
}

void ARobotVehicle::PreInitializeComponents()
{
    Super::PreInitializeComponents();
    if (IsDynamicRuntimeRobot())
    {
        DefaultRoot->DestroyComponent(true);
    }
    // else must keep for static child BP legacy support
}

void ARobotVehicle::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(ARobotVehicle, SkeletalMeshComp);
}
