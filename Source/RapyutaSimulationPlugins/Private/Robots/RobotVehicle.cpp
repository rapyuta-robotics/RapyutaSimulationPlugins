// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotVehicle.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "RRROS2GameMode.h"
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
    SkeletalMeshComp = CreateDefaultSubobject<USkeletalMeshComponent>(*FString::Printf(TEXT("%s_SkeletalComp"), *GetName()));
    SkeletalMeshComp->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPose;
    SkeletalMeshComp->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    RootComponent = SkeletalMeshComp;

    RobotVehicleMoveComponent =
        CreateDefaultSubobject<URobotVehicleMovementComponent>(*FString::Printf(TEXT("%s_MoveComp"), *GetName()));
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

void ARobotVehicle::BeginPlay()
{
    Super::BeginPlay();
    RobotVehicleMoveComponent->Initialize();
}

void ARobotVehicle::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    UROS2Spawnable* rosSpawnParameters = FindComponentByClass<UROS2Spawnable>();
    if (rosSpawnParameters)
    {
        RobotUniqueName = rosSpawnParameters->GetName();
        FString mapName = RobotUniqueName + TEXT("map");

        ARRROS2GameMode* ros2GameMode = Cast<ARRROS2GameMode>(UGameplayStatics::GetGameMode(GetWorld()));
        ASimulationState* simulationStateActor = ros2GameMode->SimulationState;
        if (simulationStateActor->Entities.Contains(mapName))
        {
            Map = simulationStateActor->Entities[mapName];
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("%s does not exist. Spawning at world origin"), *mapName);

            FTransform mapTransform(FTransform::Identity);

            AActor* mapActor = GetWorld()->SpawnActorDeferred<AActor>(ATargetPoint::StaticClass(), mapTransform);
            mapActor->Rename(*mapName);
#if WITH_EDITOR
            mapActor->SetActorLabel(mapName);
#endif

            UGameplayStatics::FinishSpawningActor(mapActor, mapTransform);
            Map = mapActor;

            simulationStateActor->AddEntity(mapActor);
        }
    }
}
