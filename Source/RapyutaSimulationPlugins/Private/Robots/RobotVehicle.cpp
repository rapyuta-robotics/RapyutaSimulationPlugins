// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotVehicle.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/SimulationState.h"

ARobotVehicle::ARobotVehicle()
{
    Initialize();
}

ARobotVehicle::ARobotVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    Initialize();
}

void ARobotVehicle::Initialize()
{
    SkeletalMeshComp = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("SkeletalMeshComp"));
    SkeletalMeshComp->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPose;
    SkeletalMeshComp->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    AddOwnedComponent(SkeletalMeshComp);
    RootComponent = SkeletalMeshComp;
}

bool ARobotVehicle::InitSensors(AROS2Node* InROS2Node)
{
    if (false == IsValid(InROS2Node))
    {
        return false;
    }

    // (NOTE) Use [ForEachComponent] would cause a fatal log on
    // [Container has changed during ranged-for iteration!]
    TInlineComponentArray<URRBaseLidarComponent*> lidarComponents(this);
    for (auto& lidarComp : lidarComponents)
    {
        lidarComp->InitLidar(InROS2Node);
    }

    return true;
}

void ARobotVehicle::SetLinearVel(const FVector& InLinearVelocity)
{
    // We're assuming input is in meters, so convert to centimeters.
    RobotVehicleMoveComponent->Velocity = InLinearVelocity;
}

void ARobotVehicle::SetAngularVel(const FVector& InAngularVelocity)
{
    RobotVehicleMoveComponent->AngularVelocity = InAngularVelocity;
}

void ARobotVehicle::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    if (VehicleMoveComponentClass)
    {
        // (NOTE) Being created in [OnConstruction], PIE will cause this to be reset anyway, thus requires recreation
        RobotVehicleMoveComponent = NewObject<URobotVehicleMovementComponent>(
            this, VehicleMoveComponentClass, *FString::Printf(TEXT("%s_MoveComp"), *GetName()));
        RobotVehicleMoveComponent->RegisterComponent();

        // Configure custom properties (frameids, etc.)
        ConfigureVehicleMoveComponent();

        // Init
        RobotVehicleMoveComponent->Initialize();

        // (NOTE) With [bAutoRegisterUpdatedComponent] as true by default, UpdatedComponent component will be automatically set
        // to the owner actor's root
    }
    else
    {
        // [OnConstruction] could run in various Editor BP actions, thus could not do Fatal log here
        UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] [VehicleMoveComponentClass] has not been configured!"), *GetName());
    }
}
