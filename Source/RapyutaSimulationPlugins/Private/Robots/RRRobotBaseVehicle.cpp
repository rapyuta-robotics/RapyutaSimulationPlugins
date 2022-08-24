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
        RobotVehicleMoveComponent = CastChecked<URobotVehicleMovementComponent>(
            URRUObjectUtils::CreateSelfSubobject(this, VehicleMoveComponentClass, FString::Printf(TEXT("%sMoveComp"), *GetName())));
        RobotVehicleMoveComponent->RegisterComponent();

        // Configure custom properties (frameids, etc.)
        ConfigureVehicleMoveComponent();

        // Init
        RobotVehicleMoveComponent->Initialize();

        // (NOTE) With [bAutoRegisterUpdatedComponent] as true by default, UpdatedComponent component will be automatically set
        // to the owner actor's root
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

bool ARRRobotBaseVehicle::ReplicateSubobjects(UActorChannel *Channel, FOutBunch *Bunch, FReplicationFlags *RepFlags) 
{
    bool bWroteSomething = Super::ReplicateSubobjects(Channel, Bunch, RepFlags);
    
    // Single Object
    bWroteSomething |= Channel->ReplicateSubobject(RobotVehicleMoveComponent, *Bunch, *RepFlags);
        
    return bWroteSomething;
}

void ARRRobotBaseVehicle::SetLinearVel(const FVector& InLinearVel)
{
    ServerSetLinearVel(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), GetActorLocation(), GetActorQuat(), InLinearVel);
    ClientSetLinearVel(InLinearVel);
}

void ARRRobotBaseVehicle::SetAngularVel(const FVector& InAngularVel)
{
    ServerSetAngularVel(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), GetActorRotation(), InAngularVel);
    ClientSetAngularVel(InAngularVel);
}

void ARRRobotBaseVehicle::ServerSetLinearVel(float InClientTimeStamp,
                                                            const FVector& InClientRobotPosition,
                                                            const FQuat& InClientRobotQuat,
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
        npc->ServerSetLinearVel(ServerRobot, InClientTimeStamp, InClientRobotPosition, InClientRobotQuat, InLinearVel);
    }
}

void ARRRobotBaseVehicle::ServerSetAngularVel(float InClientTimeStamp,
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

void ARRRobotBaseVehicle::ClientSetLinearVel(const FVector& InLinearVel)
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

void ARRRobotBaseVehicle::ClientSetAngularVel(const FVector& InAngularVel)
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
