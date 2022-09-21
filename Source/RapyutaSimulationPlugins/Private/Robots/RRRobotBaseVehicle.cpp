// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotBaseVehicle.h"

// UE
#include "GameFramework/GameState.h"
#include "Net/UnrealNetwork.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRUObjectUtils.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RRRobotVehicleROSController.h"
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

bool ARRRobotBaseVehicle::ReplicateSubobjects(UActorChannel* Channel, FOutBunch* Bunch, FReplicationFlags* RepFlags)
{
    bool bWroteSomething = Super::ReplicateSubobjects(Channel, Bunch, RepFlags);

    // Single Object
    bWroteSomething |= Channel->ReplicateSubobject(RobotVehicleMoveComponent, *Bunch, *RepFlags);

    return bWroteSomething;
}

void ARRRobotBaseVehicle::SetLinearVel(const FVector& InLinearVel)
{
    SyncServerLinearMovement(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), GetTransform(), InLinearVel);
    SetLocalLinearVel(InLinearVel);
}

void ARRRobotBaseVehicle::SetAngularVel(const FVector& InAngularVel)
{
    SyncServerAngularMovement(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), GetActorRotation(), InAngularVel);
    SetLocalAngularVel(InAngularVel);
}

void ARRRobotBaseVehicle::SetJointsStates(const TMap<FString, float>& InJointsStates)
{
    if(RobotVehicleMoveComponent)
    {
        RobotVehicleMoveComponent->JointsStates = InJointsStates;
    }
}

void ARRRobotBaseVehicle::SyncServerLinearMovement(float InClientTimeStamp,
                                                   const FTransform& InClientRobotTransform,
                                                   const FVector& InLinearVel)
{
    // todo: following block is used for RPC in server, which will be used if RPC from non player can be supported.
    // if (RobotVehicleMoveComponent)
    // {
    //     float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    //     SetActorLocation(InClientRobotPosition + InLinearVel * (serverCurrentTime - InClientTimeStamp));
    //     RobotVehicleMoveComponent->Velocity = InLinearVel;
    // }
    auto* npc = Cast<ARRNetworkPlayerController>(UGameplayStatics::GetPlayerController(GetWorld(), 0));
    if (npc != nullptr)
    {
        npc->ServerSetLinearVel(ServerRobot, InClientTimeStamp, InClientRobotTransform, InLinearVel);
    }
}

void ARRRobotBaseVehicle::SyncServerAngularMovement(float InClientTimeStamp,
                                                    const FRotator& InClientRobotRotation,
                                                    const FVector& InAngularVel)
{
    // todo: following block is used for RPC in server, which will be used if RPC from non player can be supported.
    // if (RobotVehicleMoveComponent)
    // {
    //     // GetPlayerController<APlayerController>(0, InContextObject)
    //     float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    //     SetActorRotation(InClientRobotRotation + InAngularVel.Rotation() * (serverCurrentTime - InClientTimeStamp));
    //     RobotVehicleMoveComponent->AngularVelocity = InAngularVel;
    // }

    // call rpc via NetworkPlayerController since can't call rpc directly from non-player pawn.
    auto* npc = Cast<ARRNetworkPlayerController>(UGameplayStatics::GetPlayerController(GetWorld(), 0));
    if (npc != nullptr)
    {
        npc->ServerSetAngularVel(ServerRobot, InClientTimeStamp, InClientRobotRotation, InAngularVel);
    }
}

void ARRRobotBaseVehicle::SetLocalLinearVel(const FVector& InLinearVel)
{
    if (RobotVehicleMoveComponent)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("PLAYER [%s] SetLocalLinearVel %s"),
               *PlayerController->PlayerState->GetPlayerName(),
               *InLinearVel.ToString());
#endif
        RobotVehicleMoveComponent->Velocity = InLinearVel;
    }
}

void ARRRobotBaseVehicle::SetLocalAngularVel(const FVector& InAngularVel)
{
    if (RobotVehicleMoveComponent)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("PLAYER [%s] SetLocalAngularVel %s"),
               *PlayerController->PlayerState->GetPlayerName(),
               *InAngularVel.ToString());
#endif
        RobotVehicleMoveComponent->AngularVelocity = InAngularVel;
    }
}
