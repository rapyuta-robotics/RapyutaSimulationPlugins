// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRBaseRobot.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRUObjectUtils.h"
#include "Drives/RRJointComponent.h"
#include "Robots/RRRobotROS2Interface.h"
#include "Sensors/RRROS2BaseSensorComponent.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/SimulationState.h"

ARRBaseRobot::ARRBaseRobot()
{
    SetupDefault();
}

ARRBaseRobot::ARRBaseRobot(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefault();
}

void ARRBaseRobot::SetupDefault()
{
    // Generally, for sake of dynamic robot type import/creation, child components would be then created on the fly!
    // Besides, a default subobject, upon content changes, also makes the owning actor become vulnerable since one in child BP actor
    // classes will automatically get invalidated.
    URRUObjectUtils::SetupDefaultRootComponent(this);

    // NOTE: Any custom object class (eg ROS2InterfaceClass) that is required to be configurable by this class' child BP ones
    // & IF its object needs to be created before BeginPlay(),
    // -> They must be left NULL here, so its object (eg ROS2Interface) is not created by default in [PostInitializeComponents()]
}

void ARRBaseRobot::PostInitializeComponents()
{
    if (ROS2InterfaceClass)
    {
        CreateROS2Interface();
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s] [ARRBaseRobot::PostInitializeComponents()] ROS2InterfaceClass has not been configured, "
                    "probably later in child BP class!"),
               *GetName());
    }

    // Super::, for EAutoPossessAI::PlacedInWorldOrSpawned, spawn APawn's default controller,
    // which does the possessing, thus must be called afterwards
    Super::PostInitializeComponents();
}

void ARRBaseRobot::CreateROS2Interface()
{
    ROS2Interface = CastChecked<URRRobotROS2Interface>(
        URRUObjectUtils::CreateSelfSubobject(this, ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));
    ROS2Interface->SetupROSParams();
    // NOTE: NOT call ROS2Interface->Initialize(this) here since robot's ros2-based accessories might not have been fully accessible
    // yet. For sure, that would be done in Controller's OnPossess
}

bool ARRBaseRobot::InitSensors(AROS2Node* InROS2Node)
{
    if (false == IsValid(InROS2Node))
    {
        return false;
    }

    // NOTE:
    // + Sensor comps could have been created either statically in child BPs/SetupDefault()/PostInitializeComponents()
    // OR dynamically afterwards
    // + Use [ForEachComponent] would cause a fatal log on [Container has changed during ranged-for iteration!]
    TInlineComponentArray<URRROS2BaseSensorComponent*> sensorComponents(this);
    for (auto& sensorComp : sensorComponents)
    {
        sensorComp->InitalizeWithROS2(InROS2Node);
    }

    return true;
}

void ARRBaseRobot::SetJointState(const TMap<FString, TArray<float>>& InJointState, const ERRJointControlType InJointControlType)
{
    // SetAngularVelocityTarget
    for (auto& joint : InJointState)
    {
        if (Joints.Contains(joint.Key))
        {
            // switch for types
            switch (InJointControlType)
            {
                case ERRJointControlType::POSITION:
                    Joints[joint.Key]->SetPoseTargetWithArray(joint.Value);
                    break;
                case ERRJointControlType::VELOCITY:
                    Joints[joint.Key]->SetVelocityWithArray(joint.Value);
                    break;
                case ERRJointControlType::EFFORT:
                    UE_LOG(LogRapyutaCore,
                           Warning,
                           TEXT("[%s] [ARRBaseRobot] [SetJointState] Effort control is not supported."),
                           *GetName());
                    break;
            }
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s] [ARRBaseRobot] [SetJointState] do not have joint named %s "),
                   *GetName(),
                   *joint.Key);
        }
    }
}
