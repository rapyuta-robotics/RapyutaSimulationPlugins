// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRBaseRobot.h"

// UE
#include "Engine/ActorChannel.h"
#include "Net/UnrealNetwork.h"

// rclUE
#include "ROS2NodeComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRGameSingleton.h"
#include "Core/RRNetworkGameMode.h"
#include "Core/RRNetworkGameState.h"
#include "Core/RRNetworkPlayerController.h"
#include "Core/RRUObjectUtils.h"
#include "Drives/RRJointComponent.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RRBaseRobotROSController.h"
#include "Robots/RRRobotROS2Interface.h"
#include "Sensors/RRROS2BaseSensorComponent.h"
#include "Tools/SimulationState.h"
#include "UI/RRUserWidget.h"

// Others
#include "Json.h"

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
    DefaultRoot = URRUObjectUtils::SetupDefaultRootComponent(this);

    AIControllerClass = ARRBaseRobotROSController::StaticClass();
    AutoPossessPlayer = EAutoReceiveInput::Disabled;
    AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;
    bUIWidgetEnabled = false;
    UIUserWidgetClass = URRUserWidget::StaticClass();

    // NOTE: Any custom object class (eg ROS2InterfaceClass, VehicleMoveComponentClass) that is required to be configurable by this class' child BP ones
    // & IF its object needs to be created before BeginPlay(),
    // -> They must be left NULL here, so its object (eg ROS2Interface) is not created by default in [PreInitializeComponents()]
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
    DOREPLIFETIME(ARRBaseRobot, DefaultRoot);
    DOREPLIFETIME(ARRBaseRobot, ROS2Interface);
    DOREPLIFETIME(ARRBaseRobot, ROS2InterfaceClass);
    DOREPLIFETIME(ARRBaseRobot, ROSSpawnParameters);
    DOREPLIFETIME(ARRBaseRobot, bStartStopROS2Interface);
    DOREPLIFETIME(ARRBaseRobot, NetworkAuthorityType);
    DOREPLIFETIME(ARRBaseRobot, RobotVehicleMoveComponent);
    DOREPLIFETIME(ARRBaseRobot, VehicleMoveComponentClass);
}

bool ARRBaseRobot::ReplicateSubobjects(UActorChannel* Channel, FOutBunch* Bunch, FReplicationFlags* RepFlags)
{
    bool bWroteSomething = Super::ReplicateSubobjects(Channel, Bunch, RepFlags);

    // Single Object
    bWroteSomething |= Channel->ReplicateSubobject(ROS2Interface, *Bunch, *RepFlags);
    bWroteSomething |= Channel->ReplicateSubobject(RobotVehicleMoveComponent, *Bunch, *RepFlags);
    return bWroteSomething;
}

bool ARRBaseRobot::IsAuthorizedInThisClient()
{
    // Get networkplayer controller
    auto* npc = Cast<ARRNetworkPlayerController>(UGameplayStatics::GetPlayerController(GetWorld(), 0));

    bool res = false;
    if (nullptr != npc && nullptr != npc->ROS2SimStateClient && npc->ROS2SimStateClient->GetNetworkPlayerId())
    {
        if (nullptr == ROS2Interface)
        {
#if RAPYUTA_SIM_VERBOSE
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("No ROS2Controller found"));
#endif
            res = false;
        }
        else if (nullptr == ROS2Interface->ROSSpawnParameters)
        {
#if RAPYUTA_SIM_VERBOSE
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("No ROS2Controller->ROSSpawnParameters found"));
#endif
            res = false;
        }
        else if (nullptr == npc)
        {
#if RAPYUTA_SIM_VERBOSE
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("No ARRNetworkPlayerController found"));
#endif
            res = false;
        }
        else if (ROS2Interface->ROSSpawnParameters->GetNetworkPlayerId() == npc->ROS2SimStateClient->GetNetworkPlayerId())
        {
            UE_LOG_WITH_INFO_NAMED(
                LogRapyutaCore, Log, TEXT("PlayerId is matched. PlayerId=%d."), npc->GetPlayerState<APlayerState>()->GetPlayerId());
            res = true;
        }
        else
        {
#if RAPYUTA_SIM_VERBOSE
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore,
                                   Warning,
                                   TEXT("PlayerId is mismatched. This robot spawned by PlaeyrId=%d. This Client's PlayerId=%d. "),
                                   ROS2Interface->ROSSpawnParameters->GetNetworkPlayerId(),
                                   npc->GetPlayerState<APlayerState>()->GetPlayerId());
#endif
            res = false;
        }
        GetWorld()->GetTimerManager().ClearTimer(ROS2InitTimer);
    }
    return res;
}

FString ARRBaseRobot::GetDynamicResourceName(const ERRResourceDataType InDataType) const
{
    return URRUObjectUtils::ComposeDynamicResourceName(URRGameSingleton::GetAssetNamePrefix(InDataType), *RobotModelName);
}

FString ARRBaseRobot::GetDynamicResourceAssetPath(const ERRResourceDataType InDataType) const
{
    return RRGameSingleton->GetSimResourceInfo(InDataType).Data.FindRef(GetDynamicResourceName(InDataType)).GetAssetPath();
}

void ARRBaseRobot::PreInitializeComponents()
{
    if (ROSSpawnParameters)
    {
        InitPropertiesFromJSONAll();
        SetRobotModelName(ROSSpawnParameters->ActorModelName);
        SetRobotName(ROSSpawnParameters->ActorName);
    }

    if (ROS2Interface == nullptr)
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
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore,
                                   Warning,
                                   TEXT("ROS2InterfaceClass has not been configured, "
                                        "probably later in child BP class!"));
        }
    }

    if (bStartStopROS2Interface)
    {
        InitROS2Interface();
    }

    BPPreInitializeComponents();

    // Super::, for EAutoPossessAI::PlacedInWorldOrSpawned, spawn APawn's default controller,
    // which does the possessing, thus must be called afterwards
    Super::PreInitializeComponents();
}

void ARRBaseRobot::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    if (bMobileRobot)
    {
        InitMoveComponent();
    }
    BPPostInitializeComponents();
}

void ARRBaseRobot::SetBaseMeshComp(UMeshComponent* InBaseMeshComp, bool bInMakeAsRoot, bool bInDestroyDefaultRoot)
{
    if (!ensure(IsValid(InBaseMeshComp)))
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(LogRapyutaCore, Error, TEXT("InBaseMeshComp null!"));
        return;
    }
    BaseMeshComp = InBaseMeshComp;
    if (bInMakeAsRoot)
    {
        // NOTE: This is critical, otherwise GetTransform() always returns one of [DefaultRoot] as initial one at spawning due to its being just a non-physics scene component
        if ((nullptr == RootComponent) || (DefaultRoot == RootComponent))
        {
            // NOTE: [RootComponent] must be valid for the spawning
            SetRootComponent(InBaseMeshComp);
        }
        // else: If Root is some other component, probably setup in Child class,
        // [BaseMeshComp] will only be promoted later AFTER removing that Root

        // Only possible to remove [DefaultRoot] as outside ctor
        if (bInDestroyDefaultRoot && !URRThreadUtils::IsInsideConstructor())
        {
            DefaultRoot->DestroyComponent();
            DefaultRoot = nullptr;
        }
        else if (IsValid(DefaultRoot))
        {
            // Cannot remove [DefaultRoot] in ctor thus to be done later in [PreInitializeComponents()]
            DefaultRoot->AttachToComponent(InBaseMeshComp, FAttachmentTransformRules::KeepRelativeTransform);
        }
    }
}

void ARRBaseRobot::OnRep_ROS2Interface()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT(""));
#endif
    // Since Replication order of ROS2Interface, bStartStopROS2Interface, ROSSpawnParameters can be shuffled,
    // Trigger init ROS 2 interface in each OnRep function.
    // need to initialize here as well.
    // https://forums.unrealengine.com/t/replication-ordering-guarantees/264974
    if (bStartStopROS2Interface)
    {
        InitROS2Interface();
    }
}

void ARRBaseRobot::OnRep_bStartStopROS2Interface()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("");)
#endif
    if (bStartStopROS2Interface)
    {
        InitROS2Interface();
    }
    else
    {
        DeInitROS2Interface();
    }
}

void ARRBaseRobot::SetRootOffset(const FTransform& InRootOffset)
{
    RootOffset = InRootOffset;
}

void ARRBaseRobot::CreateROS2Interface()
{
    UE_LOG_WITH_INFO_SHORT_NAMED(LogRapyutaCore,
                                 Log,
                                 TEXT("ROS2InterfaceClass: [%s] - IsNetMode: %d"),
                                 *ROS2InterfaceClass->GetName(),
                                 IsNetMode(NM_Client));
    ROS2Interface = CastChecked<URRRobotROS2Interface>(
        URRUObjectUtils::CreateSelfSubobject(this, ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));

    // NOTE: NOT call ROS2Interface->Initialize(this) here since robot's ros2-based accessories might not have been fully accessible
    // yet.
    // Thus, that would be done in Controller's OnPossess for reasons:
    // + Controller, upon posses/unpossess, acts as the pivot to start/stop robot's ROS2Interface
    // + ROS2Interface, due to requirements for also instantiatable in ARRBaseRobot's child BPs, may not
    // have been instantiated yet
    // + Child class' ros2-related accessories (ROS 2 node, sensors, publishers/subscribers)
    //  may have not been fully accessible until then.
}
void ARRBaseRobot::InitROS2Interface()
{
    if (!InitROS2InterfaceImpl())
    {
        GetWorld()->GetTimerManager().SetTimer(
            ROS2InitTimer, FTimerDelegate::CreateLambda([this] { InitROS2InterfaceImpl(); }), 1.0f, true);
    }
}

bool ARRBaseRobot::InitROS2InterfaceImpl()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("%d"), IsAuthorizedInThisClient());
#endif
    if ((IsNetMode(NM_Standalone) && nullptr != ROS2Interface) || (IsNetMode(NM_Client) && IsAuthorizedInThisClient()))
    {
        ROS2Interface->Initialize(this);
        if (NetworkAuthorityType == ERRNetworkAuthorityType::CLIENT)
        {
            SetReplicatingMovement(false);
        }
        GetWorld()->GetTimerManager().ClearTimer(ROS2InitTimer);
        return true;
    }
    else
    {
        // Use replication to triggerto this function in client.
        // Since RPC can't be used from non-player controller
        bStartStopROS2Interface = true;
        return false;
    }
}

void ARRBaseRobot::DeInitROS2Interface()
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("%d"), IsAuthorizedInThisClient());
#endif

    if ((IsNetMode(NM_Standalone) && nullptr != ROS2Interface) || (IsNetMode(NM_Client) && IsAuthorizedInThisClient()))
    {
        ROS2Interface->DeInitialize();
    }
    else
    {
        // Use replication to triggerto this function in client.
        // Since RPC can't be used from non-player controller
        bStartStopROS2Interface = false;
    }
}

void ARRBaseRobot::SetMoveComponent(UMovementComponent* InMoveComponent)
{
    if (InMoveComponent == nullptr)
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("InMoveComponent is nullptr!"));
        return;
    }

    MovementComponent = InMoveComponent;
    MovementComponent->RegisterComponent();
    MovementComponent->SetIsReplicated(true);
    RobotVehicleMoveComponent = Cast<URobotVehicleMovementComponent>(MovementComponent);
}

void ARRBaseRobot::ConfigureMovementComponent()
{
    // Configure custom properties (frameids, etc.)
    if (RobotVehicleMoveComponent && ROS2Interface && ROS2Interface->OdomComponent)
    {
        RobotVehicleMoveComponent->OdomComponent = ROS2Interface->OdomComponent;
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Display,
                         TEXT("Assigned OdomSource to RobotVehicleMoveComponent %s from ROS2Interface %s."),
                         *RobotVehicleMoveComponent->GetName(),
                         *ROS2Interface->GetName());
    }
};

bool ARRBaseRobot::InitMoveComponent()
{
    // if htere is a Movecomponent, use it as the movement component.
    // Mainly targeting use MoveComponent in child BP.
    if (MovementComponent == nullptr)
    {
        SetMoveComponent(FindComponentByClass<UMovementComponent>());
    }

    // Create MovementComponent. If it is already created by BP, this part is skiped.
    if (VehicleMoveComponentClass && MovementComponent == nullptr)
    {
        // (NOTE) Being created in [OnConstruction], PIE will cause this to be reset anyway, thus requires recreation
        SetMoveComponent(CastChecked<UMovementComponent>(URRUObjectUtils::CreateSelfSubobject(
            this, VehicleMoveComponentClass, FString::Printf(TEXT("%sMoveComp"), *GetName()))));

        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Display,
                         TEXT("[%s] created from class %s!"),
                         *MovementComponent->GetName(),
                         *VehicleMoveComponentClass->GetName());
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(
            LogRapyutaCore, Warning, TEXT("VehicleMoveComponentClass has not been configured, probably later in child BP class!"));
    }

    if (bInitRobotVehicleMoveComponent)
    {
        // Customize
        ConfigureMovementComponent();
        BPConfigureMovementComponent();

        // Init
        if (RobotVehicleMoveComponent)
        {
            RobotVehicleMoveComponent->Initialize();
        }

        // (NOTE) With [bAutoRegisterUpdatedComponent] as true by default, UpdatedComponent component will be automatically set
        // to the owner actor's root
    }
    return true;
}

bool ARRBaseRobot::InitSensors(UROS2NodeComponent* InROS2Node)
{
    if (false == IsValid(InROS2Node))
    {
        return false;
    }

    // NOTE:
    // + Sensor comps could have been created either statically in child BPs/SetupDefault()/PreInitializeComponents()
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
                    Joints[joint.Key]->SetVelocityTargetWithArray(joint.Value);
                    break;
                case ERRJointControlType::EFFORT:
                    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("Effort control is not supported."));
                    break;
            }
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(
                LogRapyutaCore, Warning, TEXT("[%s] [ARRBaseRobot] [SetJointState] do not have joint named %s "), *joint.Key);
        }
    }
}

void ARRBaseRobot::StopMovement()
{
    auto* moveComp = GetMovementComponent();
    if (moveComp)
    {
        moveComp->StopMovementImmediately();
    }
    SetLinearVel(FVector::ZeroVector);
    SetAngularVel(FVector::ZeroVector);
}

void ARRBaseRobot::SetLinearVel(const FVector& InLinearVel)
{
    LastCmdVelUpdateTime = GetWorld()->GetGameState()->GetServerWorldTimeSeconds();
    SyncServerLinearMovement(LastCmdVelUpdateTime, GetTransform(), InLinearVel);
    SetLocalLinearVel(InLinearVel);
}

void ARRBaseRobot::SetAngularVel(const FVector& InAngularVel)
{
    LastCmdVelUpdateTime = GetWorld()->GetGameState()->GetServerWorldTimeSeconds();
    SyncServerAngularMovement(LastCmdVelUpdateTime, GetActorRotation(), InAngularVel);
    SetLocalAngularVel(InAngularVel);
}

void ARRBaseRobot::SyncServerLinearMovement(float InClientTimeStamp,
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

void ARRBaseRobot::SyncServerAngularMovement(float InClientTimeStamp,
                                             const FRotator& InClientRobotRotation,
                                             const FVector& InAngularVel)
{
    // todo: following block is used for RPC in server, which will be used if RPC from non player can be supported.
    // if (RobotVehicleMoveComponent)
    // {
    //     // GetPlayerController<APlayerController>(0, InContextObject)
    //     float serverCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    //     SetActorRotation(InClientRobotRotation + FRotator::MakeFromEuler(InAngularVel) * (serverCurrentTime - InClientTimeStamp));
    //     RobotVehicleMoveComponent->AngularVelocity = InAngularVel;
    // }

    // call rpc via NetworkPlayerController since can't call rpc directly from non-player pawn.
    auto* npc = Cast<ARRNetworkPlayerController>(UGameplayStatics::GetPlayerController(GetWorld(), 0));
    if (npc != nullptr)
    {
        npc->ServerSetAngularVel(ServerRobot, InClientTimeStamp, InClientRobotRotation, InAngularVel);
    }
}

void ARRBaseRobot::SetLocalLinearVel(const FVector& InLinearVel)
{
    TargetLinearVel = InLinearVel;
#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Warning,
                     TEXT("PLAYER [%s] SetLocalLinearVel %s"),
                     *PlayerController->PlayerState->GetPlayerName(),
                     *InLinearVel.ToString());
#endif
}

void ARRBaseRobot::SetLocalAngularVel(const FVector& InAngularVel)
{
    TargetAngularVel = InAngularVel;
#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Warning,
                     TEXT("PLAYER [%s] SetLocalAngularVel %s"),
                     *PlayerController->PlayerState->GetPlayerName(),
                     *InAngularVel.ToString());
#endif
}

void ARRBaseRobot::InitUIWidget()
{
    UIWidgetComp = URRUObjectUtils::CreateAndAttachChildComponent<URRUIWidgetComponent>(
        this, *FString::Printf(TEXT("%sUIWidget"), *GetName()), UIWidgetOffset);

    UIWidgetComp->UIUserWidgetClass = UIUserWidgetClass;

    UIWidgetComp->Init();
}

bool ARRBaseRobot::CheckUIUserWidget() const
{
    if (UIWidgetComp->GetRRWidget())
    {
        return true;
    }
    else
    {
        if (bUIWidgetEnabled)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Has [bUIWidgetEnabled] ON but NULL UIUserWidget"));
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Requires [bUIWidgetEnabled] to use UIUserWidget"));
        }
        return false;
    }
}

void ARRBaseRobot::BeginPlay()
{
    Super::BeginPlay();
    if (bUIWidgetEnabled)
    {
        InitUIWidget();
    }
    if (bInitializeJoints)
    {
        StartJointsInitialization();
    }
}

void ARRBaseRobot::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    // why this is required?
    // https://dev.epicgames.com/community/snippets/VP9/keep-chaos-physics-awake
    TInlineComponentArray<UStaticMeshComponent*> staticMeshComponents(this);
    for (auto& staticMeshComp : staticMeshComponents)
    {
        if (staticMeshComp->IsSimulatingPhysics())
        {
            staticMeshComp->WakeAllRigidBodies();
        }
    }

    if (bInitializingJoints)
    {
        CheckJointsInitialization();
    }

    if (CmdVelTimeout > 0 && CmdVelTimeout <= GetWorld()->GetGameState()->GetServerWorldTimeSeconds() - LastCmdVelUpdateTime)
    {
        StopMovement();
    }
}

bool ARRBaseRobot::InitPropertiesFromJSON()
{
    // Verify [ROSSpawnParameters], which houses JSON
    if (nullptr == ROSSpawnParameters)
    {
        return false;
    }

    TSharedRef<TJsonReader<TCHAR>> jsonReader = TJsonReaderFactory<TCHAR>::Create(ROSSpawnParameters->ActorJsonConfigs);
    TSharedPtr<FJsonObject> jsonObj = MakeShareable(new FJsonObject());
    if (!FJsonSerializer::Deserialize(jsonReader, jsonObj) && jsonObj.IsValid())
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("Failed to deserialize json to object"));
        return false;
    }

    // Parse single value
    bool bParam = false;
    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("enable_widget"), bParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("enable_widget value: %d"), bParam);
        bUIWidgetEnabled = bParam;
    }
    return true;
}

void ARRBaseRobot::StartJointsInitialization()
{
    if (bInitializingJoints)
    {
        return;
    }
    bInitializingJoints = true;

    // disable collision
    // note: no collision do not work with PhysicsConstraints.
    SetChildComponentsCollisionEnabled(false);

    // move to initial pose
    for (auto& joint : Joints)
    {
        if (joint.Value)
        {
            joint.Value->MoveToInitPose();
        }
    }
}

void ARRBaseRobot::CheckJointsInitialization()
{
    bool res = true;
    for (auto& joint : Joints)
    {
        if (joint.Value)
        {
            res &= joint.Value->HasReachedPoseTarget();
        }
    }
    bInitializingJoints = !res;

    // restore collision
    if (!bInitializingJoints)
    {
        SetChildComponentsCollisionEnabled(true);
    }
}

void ARRBaseRobot::SetChildComponentsCollisionEnabled(const bool IsEnable)
{
    TInlineComponentArray<USceneComponent*> components(this);
    for (auto& comp : components)
    {
        auto primComp = Cast<UPrimitiveComponent>(comp);
        if (primComp)
        {
            if (IsEnable)
            {
                if (OriginalCollisionProfiles.Contains(primComp))
                {
                    primComp->SetCollisionProfileName(OriginalCollisionProfiles[primComp]);
                }
            }
            else
            {
                OriginalCollisionProfiles.Emplace(primComp, primComp->GetCollisionProfileName());
                primComp->SetCollisionProfileName(TEXT("OverlapAll"));
            }
        }
    }
}

bool ARRBaseRobot::AddLink(const FString& InLinkName, UStaticMeshComponent* InMesh)
{
    if (InLinkName.IsEmpty() || InMesh == nullptr)
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(LogRapyutaCore, Error, TEXT("Mesh must not be nullptr and LinkName smust not be empty."));
        return false;
    }

    Links.Add(InLinkName, InMesh);
    return true;
}

bool ARRBaseRobot::AddJoint(const FString& InParentLinkName,
                            const FString& InChildLinkName,
                            const FString& InJointName,
                            URRJointComponent* InJoint)
{
    if (!Links.Contains(InParentLinkName) || !Links.Contains(InChildLinkName))
    {
        UE_LOG_WITH_INFO_SHORT_NAMED(
            LogRapyutaCore, Error, TEXT("Links don\'t have %s and/or %s."), *InParentLinkName, *InChildLinkName);
        return false;
    }

    InJoint->ParentLink = Links[InParentLinkName];
    InJoint->ChildLink = Links[InChildLinkName];
    Joints.Add(InJointName, InJoint);
    return true;
}
