// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotEmptyVehicle.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RRRobotVehicleROSController.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/SimulationState.h"

ARobotEmptyVehicle::ARobotEmptyVehicle()
{
    SetupDefault();
}

ARobotEmptyVehicle::ARobotEmptyVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefault();
}

void ARobotEmptyVehicle::SetupDefault()
{
    // Generally, for sake of dynamic robot type import/creation, child components would be then created on the fly!
    // Besides, a default subobject, upon content changes, also makes the owning actor become vulnerable since one in child BP actor
    // classes will automatically get invalidated.

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("SceneComp"));

    AIControllerClass = ARRRobotVehicleROSController::StaticClass();
    AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;
}

bool ARobotEmptyVehicle::InitSensors(AROS2Node* InROS2Node)
{
    if (false == IsValid(InROS2Node))
    {
        return false;
    }

    // (NOTE) Use [ForEachComponent] would cause a fatal log on
    // [Container has changed during ranged-for iteration!]
    TInlineComponentArray<URRROS2BaseSensorComponent*> sensorComponents(this);
    for (auto& sensorComp : sensorComponents)
    {
        sensorComp->InitalizeWithROS2(InROS2Node);
    }

    return true;
}

bool ARobotEmptyVehicle::InitMoveComponent()
{
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
        return true;
    }
    else
    {
        // [OnConstruction] could run in various Editor BP actions, thus could not do Fatal log here
        UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] [VehicleMoveComponentClass] has not been configured!"), *GetName());
        return false;
    }
}

void ARobotEmptyVehicle::SetLinearVel(const FVector& InLinearVelocity)
{
    RobotVehicleMoveComponent->Velocity = InLinearVelocity;
}

void ARobotEmptyVehicle::SetAngularVel(const FVector& InAngularVelocity)
{
    RobotVehicleMoveComponent->AngularVelocity = InAngularVelocity;
}

void ARobotEmptyVehicle::SetJointState(const TMap<FString, TArray<float>>& InJointState, EJointControlType InJointControlType)
{
    // SetAngularVelocityTarget
    for (auto& joint : InJointState)
    {
        if (Joints.Contains(joint.Key))
        {
            // switch for types
            switch (InJointControlType)
            {
                case EJointControlType::POSITION:
                    Joints[joint.Key]->SetPoseTargetWithArray(joint.Value);
                    break;
                case EJointControlType::VELOCITY:
                    Joints[joint.Key]->SetVelocityWithArray(joint.Value);
                    break;
                case EJointControlType::EFFORT:
                    UE_LOG(LogRapyutaCore,
                           Warning,
                           TEXT("[%s] [RobotVehicle] [SetJointState] Effort control is not supported."),
                           *GetName());
                    break;
            }
        }
        else
        {
            UE_LOG(
                LogTemp, Warning, TEXT("[%s] [RobotVehicle] [SetJointState] do not have joint named %s "), *GetName(), *joint.Key);
        }
    }
}

void ARobotEmptyVehicle::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    InitMoveComponent();
}