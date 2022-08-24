// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRBaseRobot.h"

// UE
#include "Net/UnrealNetwork.h"
#include "Engine/ActorChannel.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRNetworkGameMode.h"
#include "Core/RRNetworkGameState.h"
#include "Core/RRUObjectUtils.h"
#include "Drives/RRJointComponent.h"
#include "Robots/RRRobotROS2Interface.h"
#include "Sensors/RRROS2BaseSensorComponent.h"
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

    AutoPossessPlayer = EAutoReceiveInput::Disabled;
    AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;

    // NOTE: Any custom object class (eg ROS2InterfaceClass) that is required to be configurable by this class' child BP ones
    // & IF its object needs to be created before BeginPlay(),
    // -> They must be left NULL here, so its object (eg ROS2Interface) is not created by default in [PostInitializeComponents()]
}

void ARRBaseRobot::PostInitializeComponents()
{
    if (ROS2InterfaceClass)
    {
        // ROS2Interface is created at server and replicated to client.
        if (!IsNetMode(NM_Client))
        {
            CreateROS2Interface();
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s] [ARRBaseRobot::PostInitializeComponents()] ROS2InterfaceClass has not been configured, "
                    "probably later in child BP class!"),
               *GetName());
    }

    // which does the possessing, thus must be called afterwards
    Super::PostInitializeComponents();
}


void ARRBaseRobot::OnRep_ROS2Interface()
{
#if RAPYUTA_SIM_DEBUG
     UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s] [ARRBaseRobot::OnRep_ROS2Interface]."), *GetName()); 
#endif
    // Since Replication order of ROS2Interface, StartStopROS2Interface, ROSSpawnParameters can be shuffled,
    // Trigger init ROS2 interface in each OnRep function.
    // need to initialize here as well.
    // https://forums.unrealengine.com/t/replication-ordering-guarantees/264974
    if(StartStopROS2Interface)
    {
        InitROS2Interface();
    }

}

void ARRBaseRobot::OnRep_StartStopROS2Interface()
{
#if RAPYUTA_SIM_DEBUG  
    UE_LOG(LogRapyutaCore,
        Warning,
        TEXT("[%s] [ARRBaseRobot::OnRep_StartStopROS2Interface]"), *GetName()); 
#endif
    if(StartStopROS2Interface)
    {
        InitROS2Interface();
    }
    else
    {
        StopROS2Interface();
    }
}

bool ARRBaseRobot::IsAutorizedInThisClient()
{
    // Get networkplayer controller
    auto* npc = Cast<ARRNetworkPlayerController>(UGameplayStatics::GetPlayerController(GetWorld(), 0));

    bool res = false;
    if (nullptr == ROS2Interface)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s] [ARRBaseRobot::IsAutorizedInThisClient] No ROS2Controller found"), *GetName()); 
#endif
        res = false;
    }
    else if (IsNetMode(NM_Standalone)) //Standalone 
    {
        res = true;
    }
    else if (nullptr == ROS2Interface->ROSSpawnParameters)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s] [ARRBaseRobot::IsAutorizedInThisClient] No ROS2Controller->ROSSpawnParameters found"), *GetName()); 
#endif
        res = false;
    }
    else if(nullptr == npc)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s] [ARRBaseRobot::IsAutorizedInThisClient] No ARRNetworkPlayerController found"), *GetName()); 
#endif
        res = false;
    }
    else if (ROS2Interface->ROSSpawnParameters->GetNetworkPlayerId() == npc->GetPlayerState<APlayerState>()->GetPlayerId())
    {

        UE_LOG(LogRapyutaCore,
            Log,
            TEXT("[%s] [ARRBaseRobot::IsAutorizedInThisClient()] PlayerId is matched. PlayerId=%d."), 
            *GetName(),
            npc->GetPlayerState<APlayerState>()->GetPlayerId()
            ); 
        res = true;

    }
    else
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s] [ARRBaseRobot::IsAutorizedInThisClient()] PlayerId is mismatched. This robot spawned by PlaeyrId=%d. This Client's PlayerId=%d. "), 
            *GetName(),
            ROS2Interface->ROSSpawnParameters->GetNetworkPlayerId(),
            npc->GetPlayerState<APlayerState>()->GetPlayerId()
            ); 
#endif
        res = false;
    }

    return res;
}

void ARRBaseRobot::CreateROS2Interface()
{
    UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s][ARRBaseRobot::CreateROS2Interface] %d"), *GetName(), IsNetMode(NM_Client)); 
    ROS2Interface = CastChecked<URRRobotROS2Interface>(
        URRUObjectUtils::CreateSelfSubobject(this, ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));
    ROS2Interface->ROSSpawnParameters = ROSSpawnParameters;
    ROS2Interface->SetupROSParams();

    if(StartStopROS2Interface)
    {
        InitROS2Interface();
    }

    // NOTE: NOT call ROS2Interface->Initialize(this) here since robot's ros2-based accessories might not have been fully accessible
    // yet.
    // Thus, that would be done in Controller's OnPossess for reasons:
    // + Controller, upon posses/unpossess, acts as the pivot to start/stop robot's ROS2Interface
    // + ROS2Interface, due to requirements for also instantiatable in ARRBaseRobot's child BPs, may not
    // have been instantiated yet
    // + Child class' ros2-related accessories (ROS2 node, sensors, publishers/subscribers)
    //  may have not been fully accessible until then.
}

void ARRBaseRobot::InitROS2Interface()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s][ARRBaseRobot::InitROS2Interface] %d"), *GetName(), IsAutorizedInThisClient()); 
#endif

    if (IsNetMode(NM_Standalone) || (IsNetMode(NM_Client) && IsAutorizedInThisClient()))
    {
        ROS2Interface->Initialize(this);
        if (NetworkAuthorityType==ERRNetworkAuthorityType::CLIENT)
        {
            SetReplicatingMovement(false);
        }
    }
    else
    {
        //Use replication to triggerto this function in client.
        //Since RPC can't be used from non-player controller
        StartStopROS2Interface = true;
    }
}

void ARRBaseRobot::StopROS2Interface()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
            Warning,
            TEXT("[%s][ARRBaseRobot::StopROS2Interface] %d"), *GetName(), IsAutorizedInThisClient()); 
#endif

    if (IsNetMode(NM_Standalone) || (IsNetMode(NM_Client) && IsAutorizedInThisClient()))
    {
        ROS2Interface->StopPublishers();
    }
    else
    {
        //Use replication to triggerto this function in client.
        //Since RPC can't be used from non-player controller
        StartStopROS2Interface = false;
    }
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

void ARRBaseRobot::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    UBlueprintGeneratedClass* bpClass = Cast<UBlueprintGeneratedClass>(this->GetClass());
    if (bpClass)
    {
        bpClass->GetLifetimeBlueprintReplicationList(OutLifetimeProps);
    }
    DOREPLIFETIME(ARRBaseRobot, RobotModelName);
    DOREPLIFETIME(ARRBaseRobot, RobotID);
    DOREPLIFETIME(ARRBaseRobot, RobotUniqueName);
    DOREPLIFETIME(ARRBaseRobot, ServerRobot);
    DOREPLIFETIME(ARRBaseRobot, ROS2Interface);
    DOREPLIFETIME(ARRBaseRobot, ROS2InterfaceClass);
    DOREPLIFETIME(ARRBaseRobot, ROSSpawnParameters);
    DOREPLIFETIME(ARRBaseRobot, StartStopROS2Interface);
}

bool ARRBaseRobot::ReplicateSubobjects(UActorChannel *Channel, FOutBunch *Bunch, FReplicationFlags *RepFlags) 
{
    bool bWroteSomething = Super::ReplicateSubobjects(Channel, Bunch, RepFlags);
    
    // Single Object
    bWroteSomething |= Channel->ReplicateSubobject(ROS2Interface, *Bunch, *RepFlags);
        
    return bWroteSomething;
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
