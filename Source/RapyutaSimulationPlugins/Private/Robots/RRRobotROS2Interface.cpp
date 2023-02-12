// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotROS2Interface.h"

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
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotBaseVehicle.h"

void URRRobotROS2Interface::Initialize(ARRBaseRobot* InRobot)
{
    UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] [URRRobotROS2Interface::Initialize] InitializeROS2Node"), *GetName());
    if (nullptr == InRobot)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] [URRRobotROS2Interface::Initialize] No pawn is given."), *GetName());
        return;
    }

    Robot = InRobot;
    Robot->ROS2Interface = this;

    // Instantiate a ROS2 node for InRobot
    InitRobotROS2Node(InRobot);

    // Initialize Robot's sensors (lidar, etc.)
    // NOTE: This inits both static sensors added by BP robot & possiblly also dynamic ones added in the overriding child InitSensors()
    verify(InRobot->InitSensors(RobotROS2Node));

    // Refresh TF, Odom publishers
    verify(InitPublishers());

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

    // Additional initialization implemented in BP
    BPInitialize();
}

void URRRobotROS2Interface::DeInitialize()
{
    if (nullptr == RobotROS2Node)
    {
        return;
    }

    Robot->ROS2Interface = nullptr;
    Robot = nullptr;

    StopPublishers();
}

void URRRobotROS2Interface::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URRRobotROS2Interface, Robot);
    DOREPLIFETIME(URRRobotROS2Interface, RobotROS2Node);
    DOREPLIFETIME(URRRobotROS2Interface, ROSSpawnParameters);
    DOREPLIFETIME(URRRobotROS2Interface, OdomPublisher);
    DOREPLIFETIME(URRRobotROS2Interface, bPublishOdom);
    DOREPLIFETIME(URRRobotROS2Interface, bPublishOdomTf);
    DOREPLIFETIME(URRRobotROS2Interface, CmdVelTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, JointsCmdTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, bWarnAboutMissingLink);
}

void URRRobotROS2Interface::InitRobotROS2Node(ARRBaseRobot* InRobot)
{
    const FString nodeName = URRGeneralUtils::GetNewROS2NodeName(InRobot->GetName());
    if (RobotROS2Node == nullptr)
    {
        FActorSpawnParameters spawnParams;
        spawnParams.Name = FName(*nodeName);
        RobotROS2Node = NewObject<UROS2NodeComponent>(this);
    }
    RobotROS2Node->Name = nodeName;

    // Set robot's [ROS2Node] namespace from spawn parameters if existing
    if (ROSSpawnParameters)
    {
        RobotROS2Node->Namespace = ROSSpawnParameters->GetNamespace();
    }
    else
    {
        RobotROS2Node->Namespace = InRobot->RobotUniqueName;
    }
    RobotROS2Node->Init();
}

bool URRRobotROS2Interface::InitPublishers()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // OdomPublisher (with TF)
    auto* robotVehicle = Cast<ARRRobotBaseVehicle>(Robot);
    if (bPublishOdom && robotVehicle != nullptr)
    {
        if (nullptr == OdomPublisher)
        {
            OdomPublisher = NewObject<URRROS2OdomPublisher>(this);
            OdomPublisher->SetDefaultDelegates();
            OdomPublisher->bPublishOdomTf = bPublishOdomTf;
            OdomPublisher->PublicationFrequencyHz = OdomPublicationFrequencyHz;
        }
        OdomPublisher->InitializeWithROS2(RobotROS2Node);

        // If publishing odom, it must be an [ARRRobotBaseVehicle]
        // todo separate ROS2Interface for mobile robot.
        OdomPublisher->RobotVehicle = robotVehicle;
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
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s][URRRobotROS2Interface][InitPublishers] %s is nullptr. Please create before initialization."),
                   *pub.Key,
                   *GetName());
        }
    }

    return true;
}

void URRRobotROS2Interface::StopPublishers()
{
    if (bPublishOdom && OdomPublisher)
    {
        OdomPublisher->RobotVehicle = nullptr;
    }

    // Additional publishers by child class or robot
    for (auto& pub : Publishers)
    {
        if (pub.Value != nullptr)
        {
            pub.Value->StopPublishTimer();
        }
    }
}

bool URRRobotROS2Interface::InitSubscriptions()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    // Subscription with callback to enqueue vehicle spawn info.
    ROS2_CREATE_SUBSCRIBER(
        RobotROS2Node, this, CmdVelTopicName, UROS2TwistMsg::StaticClass(), &URRRobotROS2Interface::MovementCallback);

    // Subscription with callback to enqueue vehicle spawn info.
    ROS2_CREATE_SUBSCRIBER(
        RobotROS2Node, this, JointsCmdTopicName, UROS2JointStateMsg::StaticClass(), &URRRobotROS2Interface::JointStateCallback);

    // Additional subscribers by child class or robot
    for (auto& sub : Subscribers)
    {
        if (sub.Value != nullptr)
        {
            RobotROS2Node->AddSubscription(sub.Value);
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s][URRRobotROS2Interface][InitSubscriptions] %s is nullptr. Please create before initialization."),
                   *sub.Key,
                   *GetName());
        }
    }
    return true;
}

bool URRRobotROS2Interface::InitServiceClients()
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
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s][URRRobotROS2Interface][InitServiceClients] %s is nullptr. Please create before initialization."),
                   *client.Key,
                   *GetName());
        }
    }
    return true;
}

bool URRRobotROS2Interface::InitServiceServers()
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
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s][URRRobotROS2Interface][InitServiceServers] %s is nullptr. Please create before initialization."),
                   *server.Key,
                   *GetName());
        }
    }
    return true;
}

bool URRRobotROS2Interface::InitActionClients()
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
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s][URRRobotROS2Interface][InitActionClients] %s is nullptr. Please create before initialization."),
                   *client.Key,
                   *GetName());
        }
    }
    return true;
}

bool URRRobotROS2Interface::InitActionServers()
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
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s][URRRobotROS2Interface][InitActionServers] %s is nullptr. Please create before initialization."),
                   *server.Key,
                   *GetName());
        }
    }
    return true;
}

void URRRobotROS2Interface::MovementCallback(const UROS2GenericMsg* Msg)
{
    const UROS2TwistMsg* twistMsg = Cast<UROS2TwistMsg>(Msg);
    if (IsValid(twistMsg))
    {
        // TODO refactoring will be needed to put units and system of reference conversions in a consistent location
        // probably should not stay in msg though
        FROSTwist twist;
        twistMsg->GetMsg(twist);
        const FVector linear(URRConversionUtils::VectorROSToUE(twist.Linear));
        const FVector angular(URRConversionUtils::RotationROSToUE(twist.Angular));

        // (Note) In this callback, which could be invoked from a ROS working thread,
        // thus any direct referencing to its member in this GameThread lambda needs to be verified.
        AsyncTask(ENamedThreads::GameThread,
                  [this, linear, angular]
                  {
                      ARRRobotBaseVehicle* robotVehicle = CastChecked<ARRRobotBaseVehicle>(Robot);
                      check(IsValid(robotVehicle));
                      robotVehicle->SetLinearVel(linear);
                      robotVehicle->SetAngularVel(angular);
                  });
    }
}

void URRRobotROS2Interface::JointStateCallback(const UROS2GenericMsg* Msg)
{
    const UROS2JointStateMsg* jointStateMsg = Cast<UROS2JointStateMsg>(Msg);
    if (IsValid(jointStateMsg))
    {
        // TODO refactoring will be needed to put units and system of reference conversions in a consistent location
        // probably should not stay in msg though
        FROSJointState jointState;
        jointStateMsg->GetMsg(jointState);

        // Check Joint type. should be different function?
        ERRJointControlType jointControlType;
        if (jointState.Name.Num() == jointState.Position.Num())
        {
            jointControlType = ERRJointControlType::POSITION;
        }
        else if (jointState.Name.Num() == jointState.Velocity.Num())
        {
            jointControlType = ERRJointControlType::VELOCITY;
        }
        else if (jointState.Name.Num() == jointState.Effort.Num())
        {
            jointControlType = ERRJointControlType::EFFORT;
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s] [URRRobotROS2Interface] [JointStateCallback] Effort control is not supported."),
                   *GetName());
            return;
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s] [URRRobotROS2Interface] [JointStateCallback] position, velocity or effort array must be same "
                        "size of name array"),
                   *GetName());
            return;
        }

        // Calculate input, ROS to UE conversion.
        TMap<FString, TArray<float>> joints;
        for (auto i = 0; i < jointState.Name.Num(); ++i)
        {
            if (!Robot->Joints.Contains(jointState.Name[i]))
            {
                if (bWarnAboutMissingLink)
                {
                    UE_LOG(LogRapyutaCore,
                           Warning,
                           TEXT("[%s] [URRRobotROS2Interface] [JointStateCallback] vehicle do not have joint named %s."),
                           *GetName(),
                           *jointState.Name[i]);
                }
                continue;
            }

            TArray<float> input;
            if (ERRJointControlType::POSITION == jointControlType)
            {
                input.Add(jointState.Position[i]);
            }
            else if (ERRJointControlType::VELOCITY == jointControlType)
            {
                input.Add(jointState.Velocity[i]);
            }
            else
            {
                UE_LOG(LogRapyutaCore,
                       Warning,
                       TEXT("[%s] [URRRobotROS2Interface] [JointStateCallback] position, velocity or effort array must be same "
                            "size of name array"),
                       *GetName());
                continue;
            }

            // ROS To UE conversion
            if (Robot->Joints[jointState.Name[i]]->LinearDOF == 1)
            {
                input[0] *= 100;    // todo add conversion to conversion util
            }
            else if (Robot->Joints[jointState.Name[i]]->RotationalDOF == 1)
            {
                input[0] *= 180 / M_PI;    // todo add conversion to conversion util
            }
            else
            {
                UE_LOG(LogRapyutaCore,
                       Warning,
                       TEXT("[%s] [URRRobotROS2Interface] [JointStateCallback] Supports only single DOF joint. %s has %d "
                            "linear DOF and %d rotational DOF"),
                       *jointState.Name[i],
                       Robot->Joints[jointState.Name[i]]->LinearDOF,
                       Robot->Joints[jointState.Name[i]]->RotationalDOF);
            }

            joints.Emplace(jointState.Name[i], input);
        }

        // (Note) In this callback, which could be invoked from a ROS working thread,
        // thus any direct referencing to its member in this GameThread lambda needs to be verified.
        AsyncTask(ENamedThreads::GameThread,
                  [this, joints, jointControlType]
                  {
                      check(IsValid(Robot));
                      Robot->SetJointState(joints, jointControlType);
                  });
    }
}

URRRobotROS2InterfaceComponent::URRRobotROS2InterfaceComponent()
{
    ROS2Interface = CastChecked<URRRobotROS2Interface>(
        URRUObjectUtils::CreateSelfSubobject(this, ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));

    if (nullptr == Robot)
    {
        UE_LOG_WITH_INFO(LogTemp,
                         Warning,
                         TEXT("[URRRobotROS2InterfaceComponent][URRRobotROS2InterfaceComponent] Robot is nullptr. Trying to get "
                              "owner as robot."));
        Robot = Cast<ARRBaseRobot>(GetOwner());
    }

    if (nullptr != Robot)
    {
        ROS2Interface->ROSSpawnParameters = Robot->ROSSpawnParameters;
    }
    else
    {
        UE_LOG_WITH_INFO(LogTemp,
                         Warning,
                         TEXT("[URRRobotROS2InterfaceComponent][URRRobotROS2InterfaceComponent] Robot is nullptr and Owner is not "
                              "Robot. Can't set Spawnparameter"));
    }

    ROS2Interface->SetupROSParamsAll();
}

void URRRobotROS2InterfaceComponent::AddAllSubComponentToROSInterface()
{
    // add all ros components under this class to ROSInterface to make it initialize with ROSInterface initialization.
    TInlineComponentArray<UROS2PublisherComponent*> pubComps(GetOwner());
    for (auto& pubComp : pubComps)
    {
        if (nullptr != pubComp)
        {
            ROS2Interface->Publishers.Add(*pubComp->TopicName, pubComp->Publisher);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[URRRobotROS2InterfaceComponent] %s is added"), *pubComp->TopicName);
        }
    }
    TInlineComponentArray<UROS2SubscriberComponent*> subComps(GetOwner());
    for (auto& subComp : subComps)
    {
        if (nullptr != subComp)
        {
            ROS2Interface->Subscribers.Add(*subComp->TopicName, subComp->Subscriber);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[URRRobotROS2InterfaceComponent] %s is added"), *subComp->TopicName);
        }
    }
    TInlineComponentArray<UROS2ServiceClientComponent*> srvClientComps(GetOwner());
    for (auto& srvClientComp : srvClientComps)
    {
        if (nullptr != srvClientComp)
        {
            ROS2Interface->ServiceClients.Add(*srvClientComp->ServiceName, srvClientComp->ServiceClient);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[URRRobotROS2InterfaceComponent] %s is added"), *srvClientComp->ServiceName);
        }
    }
    TInlineComponentArray<UROS2ServiceServerComponent*> srvServerComps(GetOwner());
    for (auto& srvServerComp : srvServerComps)
    {
        if (nullptr != srvServerComp)
        {
            ROS2Interface->ServiceServers.Add(*srvServerComp->ServiceName, srvServerComp->ServiceServer);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[URRRobotROS2InterfaceComponent] %s is added"), *srvServerComp->ServiceName);
        }
    }
    TInlineComponentArray<UROS2ActionClientComponent*> actClientComps(GetOwner());
    for (auto& actClientComp : actClientComps)
    {
        if (nullptr != actClientComp)
        {
            ROS2Interface->ActionClients.Add(*actClientComp->ActionName, actClientComp->ActionClient);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[URRRobotROS2InterfaceComponent] %s is added"), *actClientComp->ActionName);
        }
    }
    TInlineComponentArray<UROS2ActionServerComponent*> actServerComps(GetOwner());
    for (auto& actServerComp : actServerComps)
    {
        if (nullptr != actServerComp)
        {
            ROS2Interface->ActionServers.Add(*actServerComp->ActionName, actServerComp->ActionServer);
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[URRRobotROS2InterfaceComponent] %s is added"), *actServerComp->ActionName);
        }
    }
}
