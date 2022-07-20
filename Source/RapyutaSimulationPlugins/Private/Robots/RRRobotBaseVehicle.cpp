// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotBaseVehicle.h"

// UE
#include "GameFramework/GameStateBase.h"
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
    AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;

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

void ARRRobotBaseVehicle::SetLinearVel(const FVector& InLinearVelocity)
{
    ServerSetLinearVel(GameState->GetServerWorldTimeSeconds(), GetActorLocation(), InLinearVelocity);
    ClientSetLinearVel(InLinearVelocity);
}

void ARRRobotBaseVehicle::SetAngularVel(const FVector& InAngularVelocity)
{
    ServerSetAngularVel(GameState->GetServerWorldTimeSeconds(), GetActorRotation(), InAngularVelocity);
    ClientSetAngularVel(InAngularVelocity);
}

void ARRRobotBaseVehicle::ServerSetLinearVel_Implementation(float InTimeStamp,
                                                            const FVector& InPosition,
                                                            const FVector& InLinearVelocity)
{
    float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    SetActorLocation(InPosition + InLinearVelocity * (serverCurrentTime - InTimeStamp));
    RobotVehicleMoveComponent->Velocity = InLinearVelocity;
}

void ARRRobotBaseVehicle::ServerSetAngularVel_Implementation(float InTimeStamp,
                                                             const FRotator& InRotation,
                                                             const FVector& InAngularVelocity)
{
    float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    SetActorRotation(InRotation + InAngularVelocity.Rotation() * (serverCurrentTime - InTimeStamp));
    RobotVehicleMoveComponent->AngularVelocity = InAngularVelocity;
}

void ARRRobotBaseVehicle::ClientSetLinearVel_Implementation(const FVector& InLinearVelocity)
{
    UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s]"), *PlayerController->PlayerState->GetPlayerName());
    RobotVehicleMoveComponent->Velocity = InLinearVelocity;
}

void ARRRobotBaseVehicle::ClientSetAngularVel_Implementation(const FVector& InAngularVelocity)
{
    RobotVehicleMoveComponent->AngularVelocity = InAngularVelocity;
}
