// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotBaseVehicle.h"

// UE
#include "GameFramework/GameState.h"
#include "Net/UnrealNetwork.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRUObjectUtils.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RRRobotVehicleROSController.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/SimulationState.h"
#include "Drives/RRTricycleDriveComponent.h"

ARRRobotBaseVehicle::ARRRobotBaseVehicle()
{
    SetupDefaultVehicle();
}

ARRRobotBaseVehicle::ARRRobotBaseVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefaultVehicle();
}

void ARRRobotBaseVehicle::SetupDefaultVehicle()
{
    VehicleMoveComponentClass = URRTricycleDriveComponent::StaticClass();
    AIControllerClass = ARRRobotVehicleROSController::StaticClass();
}

void ARRRobotBaseVehicle::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    InitMoveComponent(VehicleMoveComponentClass);
}

void ARRRobotBaseVehicle::SetRootOffset(const FTransform& InRootOffset)
{
    if (RobotVehicleMoveComponent)
    {
        RobotVehicleMoveComponent->RootOffset = InRootOffset;
    }
}

void ARRRobotBaseVehicle::InitMoveComponent(TSubclassOf<URobotVehicleMovementComponent> moveComponentClass)
{
    if(!RobotVehicleMoveComponent || RobotVehicleMoveComponent->GetClass() != moveComponentClass)
    {
        if(RobotVehicleMoveComponent)
        {
            RobotVehicleMoveComponent->DestroyComponent();
        }
    
        RobotVehicleMoveComponent = CastChecked<URobotVehicleMovementComponent>(URRUObjectUtils::CreateSelfSubobject(this, moveComponentClass, FString::Printf(TEXT("%sMoveComp"), *moveComponentClass->GetName())));
        //RobotVehicleMoveComponent = CreateDefaultSubobject<URRSkeletalRobotDiffDriveComponent>(TEXT("URRSkeletalRobotDiffDriveComponent"));
   
        RobotVehicleMoveComponent->PrimaryComponentTick.bCanEverTick = true;
        RobotVehicleMoveComponent->SetComponentTickEnabled(true);
        RobotVehicleMoveComponent->Initialize();
        RobotVehicleMoveComponent->RegisterComponent();
        
        if(moveComponentClass != URRTricycleDriveComponent::StaticClass())
        {
            GetVehicleMovement()->UnregisterComponent();
        }
    }
}

void ARRRobotBaseVehicle::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(ARRRobotBaseVehicle, Map);
    DOREPLIFETIME(ARRRobotBaseVehicle, RobotVehicleMoveComponent);
    DOREPLIFETIME(ARRRobotBaseVehicle, VehicleMoveComponentClass);
}

void ARRRobotBaseVehicle::SetLinearVel(const FVector& InLinearVel)
{
    ServerSetLinearVel(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), GetActorLocation(), InLinearVel);
    ClientSetLinearVel(InLinearVel);
}

void ARRRobotBaseVehicle::SetAngularVel(const FVector& InAngularVel)
{
    ServerSetAngularVel(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), GetActorRotation(), InAngularVel);
    ClientSetAngularVel(InAngularVel);
}

void ARRRobotBaseVehicle::SetJointsStates(const TMap<FString, float>& InJointsStates)
{
    if(RobotVehicleMoveComponent)
    {
        RobotVehicleMoveComponent->JointsStates = InJointsStates;
    }
}

void ARRRobotBaseVehicle::ServerSetLinearVel_Implementation(float InClientTimeStamp,
                                                            const FVector& InClientRobotPosition,
                                                            const FVector& InLinearVel)
{
    if (RobotVehicleMoveComponent)
    {
        float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
        SetActorLocation(InClientRobotPosition + InLinearVel * (serverCurrentTime - InClientTimeStamp));
        RobotVehicleMoveComponent->Velocity = InLinearVel;
    }
}

void ARRRobotBaseVehicle::ServerSetAngularVel_Implementation(float InClientTimeStamp,
                                                             const FRotator& InClientRobotRotation,
                                                             const FVector& InAngularVel)
{
    if (RobotVehicleMoveComponent)
    {
        float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
        SetActorRotation(InClientRobotRotation + InAngularVel.Rotation() * (serverCurrentTime - InClientTimeStamp));
        RobotVehicleMoveComponent->AngularVelocity = InAngularVel;
    }
}

void ARRRobotBaseVehicle::ClientSetLinearVel_Implementation(const FVector& InLinearVel)
{
    if (RobotVehicleMoveComponent)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("PLAYER [%s] ClientSetLinearVel %s"),
               *PlayerController->PlayerState->GetPlayerName(),
               *InLinearVel.ToString());
#endif
        RobotVehicleMoveComponent->Velocity = InLinearVel;
    }
}

void ARRRobotBaseVehicle::ClientSetAngularVel_Implementation(const FVector& InAngularVel)
{
    if (RobotVehicleMoveComponent)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("PLAYER [%s] ClientSetAngularVel %s"),
               *PlayerController->PlayerState->GetPlayerName(),
               *InAngularVel.ToString());
#endif
        RobotVehicleMoveComponent->AngularVelocity = InAngularVel;
    }
}
