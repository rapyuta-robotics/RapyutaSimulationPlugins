// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotBaseVehicle.h"

// UE
#include "GameFramework/GameState.h"
#include "Net/UnrealNetwork.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRNetworkPlayerController.h"
#include "Core/RRUObjectUtils.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RRRobotVehicleROSController.h"
#include "Tools/SimulationState.h"

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
    // Generally, for sake of dynamic robot type import/creation, child components would be then created on the fly!
    // Besides, a default subobject, upon content changes, also makes the owning actor become vulnerable since one in child BP actor
    // classes will automatically get invalidated.
    AIControllerClass = ARRRobotVehicleROSController::StaticClass();

    // NOTE: Any custom object class (eg ROS2Interface Class, VehicleMoveComponentClass) that is required to be configurable by
    // this class' child BP ones & if its object needs to be created before BeginPlay(),
    // -> The class must be left NULL here, so its object (eg RobotVehicleMoveComponent) is not created by default in
    // [PostInitializeComponents()]
}

void ARRRobotBaseVehicle::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    InitMoveComponent();
}

void ARRRobotBaseVehicle::SetRootOffset(const FTransform& InRootOffset)
{
    if (RobotVehicleMoveComponent)
    {
        RobotVehicleMoveComponent->RootOffset = InRootOffset;
    }
}

bool ARRRobotBaseVehicle::InitMoveComponent()
{
    if (VehicleMoveComponentClass)
    {
        // (NOTE) Being created in [OnConstruction], PIE will cause this to be reset anyway, thus requires recreation
        MovementComponent = CastChecked<UMovementComponent>(
            URRUObjectUtils::CreateSelfSubobject(this, VehicleMoveComponentClass, FString::Printf(TEXT("%sMoveComp"), *GetName())));
        MovementComponent->RegisterComponent();
        MovementComponent->SetIsReplicated(true);

        // NOTE: This could be NULL
        RobotVehicleMoveComponent = Cast<URobotVehicleMovementComponent>(MovementComponent);

        // Customize
        ConfigureMovementComponent();

        // Init
        if (RobotVehicleMoveComponent)
        {
            // Configure custom properties (frameids, etc.)
            RobotVehicleMoveComponent->Initialize();
        }

        // (NOTE) With [bAutoRegisterUpdatedComponent] as true by default, UpdatedComponent component will be automatically set
        // to the owner actor's root

        UE_LOG(LogRapyutaCore,
               Display,
               TEXT("[%s] created from class %s!"),
               *MovementComponent->GetName(),
               *VehicleMoveComponentClass->GetName());
        return true;
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s] [VehicleMoveComponentClass] has not been configured, probably later in child BP class!"),
               *GetName());
        return false;
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
    TargetLinearVel = InLinearVel;
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("PLAYER [%s] SetLocalLinearVel %s"),
           *PlayerController->PlayerState->GetPlayerName(),
           *InLinearVel.ToString());
#endif
}

void ARRRobotBaseVehicle::SetLocalAngularVel(const FVector& InAngularVel)
{
    TargetAngularVel = InAngularVel;
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("PLAYER [%s] SetLocalAngularVel %s"),
           *PlayerController->PlayerState->GetPlayerName(),
           *InAngularVel.ToString());
#endif
}
