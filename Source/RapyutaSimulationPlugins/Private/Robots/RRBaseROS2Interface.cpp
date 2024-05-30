// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRBaseROS2Interface.h"

// UE
#include "Net/UnrealNetwork.h"

// rclUE
#include "Msgs/ROS2JointState.h"
#include "Msgs/ROS2Twist.h"
#include "ROS2ActionClient.h"
#include "ROS2ActionServer.h"
#include "ROS2Publisher.h"
#include "ROS2ServiceClient.h"
#include "ROS2ServiceServer.h"
#include "ROS2Subscriber.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

void URRBaseROS2Interface::Initialize(AActor* Owner)
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Verbose, TEXT("InitializeROS2Interface"));
#endif

    if (Owner == nullptr)
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("No pawn is given."));
        return;
    }

    ROSSpawnParameters = Owner->FindComponentByClass<UROS2Spawnable>();

    SetupROSParamsAll();

    // Instantiate a ROS 2 node for InRobot
    InitRobotROS2Node(Owner);

    InitInterfaces();

    // Additional initialization implemented in BP
    BPInitialize();
}

void URRBaseROS2Interface::InitInterfaces()
{
    // Refresh TF, Odom publishers
    InitPublishers();

    // cmd_vel, joint state, and other ROS topic inputs.
    InitSubscriptions();

    // Initialize service clients
    InitServiceClients();

    // Initialize service servers
    InitServiceServers();

    // Initialize action clients
    InitActionClients();

    // Initialize action servers
    InitActionServers();
}

void URRBaseROS2Interface::DeInitialize()
{
    if (nullptr == RobotROS2Node)
    {
        return;
    }

    StopPublishers();
}

void URRBaseROS2Interface::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URRBaseROS2Interface, RobotROS2Node);
    DOREPLIFETIME(URRBaseROS2Interface, ROSSpawnParameters);
}

void URRBaseROS2Interface::InitRobotROS2Node(AActor* Owner)
{
    InitROS2NodeParam(Owner);
    RobotROS2Node->Init();
}

void URRBaseROS2Interface::InitROS2NodeParam(AActor* Owner)
{
#if WITH_EDITOR
    const FString nodeName = URRGeneralUtils::GetNewROS2NodeName(UKismetSystemLibrary::GetDisplayName(Owner));
#else
    const FString nodeName = URRGeneralUtils::GetNewROS2NodeName(Owner->GetName());
#endif
    if (RobotROS2Node == nullptr)
    {
        FActorSpawnParameters spawnParams;
        spawnParams.Name = FName(*nodeName);
        RobotROS2Node = NewObject<UROS2NodeComponent>(Owner);
    }
    RobotROS2Node->Name = nodeName;

    // Set robot's [ROS2Node] namespace from spawn parameters if existing
    if (ROSSpawnParameters)
    {
        RobotROS2Node->Namespace = ROSSpawnParameters->GetNamespace();
    }
    else
    {
        if (bUseActorNameAsNamespace)
        {
#if WITH_EDITOR
            RobotROS2Node->Namespace = UKismetSystemLibrary::GetDisplayName(Owner);
#else
            RobotROS2Node->Namespace = Owner->GetName();
#endif
        }
        else
        {
            RobotROS2Node->Namespace = DefautlROSNamespace;
        }
    }
}

bool URRBaseROS2Interface::InitPublishers()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // Additional publishers by child class or robot
    for (auto& pub : Publishers)
    {
        if (pub.Value != nullptr)
        {
            RobotROS2Node->AddPublisher(pub.Value);
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *pub.Key, *GetName());
        }
    }

    return true;
}

void URRBaseROS2Interface::StopPublishers()
{
    // Additional publishers by child class or robot
    for (auto& pub : Publishers)
    {
        if (pub.Value != nullptr)
        {
            pub.Value->StopPublishTimer();
        }
    }
}

bool URRBaseROS2Interface::InitSubscriptions()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // Additional subscribers by child class or robot
    for (auto& sub : Subscribers)
    {
        if (sub.Value != nullptr)
        {
            RobotROS2Node->AddSubscription(sub.Value);
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *sub.Key, *GetName());
        }
    }
    return true;
}

bool URRBaseROS2Interface::InitServiceClients()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // Additional subscribers by child class or robot
    for (auto& client : ServiceClients)
    {
        if (client.Value != nullptr)
        {
            RobotROS2Node->AddServiceClient(client.Value);
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *client.Key, *GetName());
        }
    }
    return true;
}

bool URRBaseROS2Interface::InitServiceServers()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // Additional subscribers by child class or robot
    for (auto& server : ServiceServers)
    {
        if (server.Value != nullptr)
        {
            RobotROS2Node->AddServiceServer(server.Value);
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *server.Key, *GetName());
        }
    }
    return true;
}

bool URRBaseROS2Interface::InitActionClients()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // Additional subscribers by child class or robot
    for (auto& client : ActionClients)
    {
        if (client.Value != nullptr)
        {
            RobotROS2Node->AddActionClient(client.Value);
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *client.Key, *GetName());
        }
    }
    return true;
}

bool URRBaseROS2Interface::InitActionServers()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // Additional subscribers by child class or robot
    for (auto& server : ActionServers)
    {
        if (server.Value != nullptr)
        {
            RobotROS2Node->AddActionServer(server.Value);
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *server.Key, *GetName());
        }
    }
    return true;
}

void URRBaseROS2InterfaceComponent::BeginPlay()
{
    if (ROS2Interface == nullptr)
    {
        ROS2Interface = CastChecked<URRBaseROS2Interface>(
            URRUObjectUtils::CreateSelfSubobject(this, ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));
    }

    Super::BeginPlay();
}

void URRBaseROS2InterfaceComponent::AddAllSubComponentToROSInterface()
{
    // add all ros components under this class to ROSInterface to make it initialize with ROSInterface initialization.
    TInlineComponentArray<UROS2PublisherComponent*> pubComps(GetOwner());
    for (auto& pubComp : pubComps)
    {
        if (nullptr != pubComp)
        {
            ROS2Interface->Publishers.Add(*pubComp->TopicName, pubComp->Publisher);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("%s is added"), *pubComp->TopicName);
        }
    }
    TInlineComponentArray<UROS2SubscriberComponent*> subComps(GetOwner());
    for (auto& subComp : subComps)
    {
        if (nullptr != subComp)
        {
            ROS2Interface->Subscribers.Add(*subComp->TopicName, subComp->Subscriber);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("%s is added"), *subComp->TopicName);
        }
    }
    TInlineComponentArray<UROS2ServiceClientComponent*> srvClientComps(GetOwner());
    for (auto& srvClientComp : srvClientComps)
    {
        if (nullptr != srvClientComp)
        {
            ROS2Interface->ServiceClients.Add(*srvClientComp->ServiceName, srvClientComp->ServiceClient);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("%s is added"), *srvClientComp->ServiceName);
        }
    }
    TInlineComponentArray<UROS2ServiceServerComponent*> srvServerComps(GetOwner());
    for (auto& srvServerComp : srvServerComps)
    {
        if (nullptr != srvServerComp)
        {
            ROS2Interface->ServiceServers.Add(*srvServerComp->ServiceName, srvServerComp->ServiceServer);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("%s is added"), *srvServerComp->ServiceName);
        }
    }
    TInlineComponentArray<UROS2ActionClientComponent*> actClientComps(GetOwner());
    for (auto& actClientComp : actClientComps)
    {
        if (nullptr != actClientComp)
        {
            ROS2Interface->ActionClients.Add(*actClientComp->ActionName, actClientComp->ActionClient);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("%s is added"), *actClientComp->ActionName);
        }
    }
    TInlineComponentArray<UROS2ActionServerComponent*> actServerComps(GetOwner());
    for (auto& actServerComp : actServerComps)
    {
        if (nullptr != actServerComp)
        {
            ROS2Interface->ActionServers.Add(*actServerComp->ActionName, actServerComp->ActionServer);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("%s is added"), *actServerComp->ActionName);
        }
    }
}
