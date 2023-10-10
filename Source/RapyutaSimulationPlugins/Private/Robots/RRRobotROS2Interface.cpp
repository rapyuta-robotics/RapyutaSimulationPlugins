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

void URRRobotROS2Interface::Initialize(ARRBaseRobot* InRobot)
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Verbose, TEXT("InitializeROS2Interface"));
#endif
    if (nullptr == InRobot)
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("No pawn is given."));
        return;
    }

    Robot = InRobot;
    Robot->ROS2Interface = this;

    // Instantiate a ROS 2 node for InRobot
    InitRobotROS2Node(InRobot);

    // OdomPublisher (with TF)
    if (bPublishOdom && Robot->bMobileRobot)
    {
        if (nullptr == OdomComponent)
        {
            OdomComponent =
                URRUObjectUtils::CreateChildComponent<URRBaseOdomComponent>(Robot, *FString::Printf(TEXT("%sOdom"), *GetName()));
            OdomComponent->bPublishOdomTf = bPublishOdomTf;
            OdomComponent->PublicationFrequencyHz = OdomPublicationFrequencyHz;
            OdomComponent->RootOffset = Robot->RootOffset;
        }
    }

    // Initialize Robot's sensors (lidar, etc.)
    // NOTE: This inits both static sensors added by BP robot & possiblly also dynamic ones added in the overriding child InitSensors()
    verify(InRobot->InitSensors(RobotROS2Node));

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

    // Additional initialization implemented in BP
    BPInitialize();
}

void URRRobotROS2Interface::DeInitialize()
{
    if (nullptr == RobotROS2Node)
    {
        return;
    }

    if (nullptr != Robot)
    {
        Robot->ROS2Interface = nullptr;
        Robot = nullptr;
    }

    StopPublishers();
}

void URRRobotROS2Interface::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URRRobotROS2Interface, Robot);
    DOREPLIFETIME(URRRobotROS2Interface, RobotROS2Node);
    DOREPLIFETIME(URRRobotROS2Interface, ROSSpawnParameters);
    DOREPLIFETIME(URRRobotROS2Interface, OdomComponent);
    DOREPLIFETIME(URRRobotROS2Interface, bPublishOdom);
    DOREPLIFETIME(URRRobotROS2Interface, bPublishOdomTf);
    DOREPLIFETIME(URRRobotROS2Interface, OdomPublicationFrequencyHz);
    DOREPLIFETIME(URRRobotROS2Interface, CmdVelTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, JointCmdTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, JointStateTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, bWarnAboutMissingLink);
}

void URRRobotROS2Interface::InitRobotROS2Node(ARRBaseRobot* InRobot)
{
    const FString nodeName = URRGeneralUtils::GetNewROS2NodeName(InRobot->GetName());
    if (RobotROS2Node == nullptr)
    {
        FActorSpawnParameters spawnParams;
        spawnParams.Name = FName(*nodeName);
        RobotROS2Node = NewObject<UROS2NodeComponent>(InRobot);
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

    // JointState publisher
    if (Robot && Robot->bJointControl)
    {
        ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(RobotROS2Node,
                                            this,
                                            JointStateTopicName,
                                            UROS2Publisher::StaticClass(),
                                            UROS2JointStateMsg::StaticClass(),
                                            JointStatePublicationFrequencyHz,
                                            &URRRobotROS2Interface::UpdateJointState,
                                            UROS2QoS::Default,
                                            JointStatePublisher);
    }

    if (Robot && bPublishJointTf)
    {
        JointsTFPublisher = CastChecked<URRROS2TFsPublisher>(RobotROS2Node->CreateLoopPublisherWithClass(
            TEXT("tf"), URRROS2TFsPublisher::StaticClass(), JointTfPublicationFrequencyHz));

        for (const auto& joint : Robot->Joints)
        {
            if (joint.Value == nullptr)
            {
                continue;
            }

            const FString* parentLinkName = Robot->Links.FindKey(joint.Value->ParentLink);
            const FString* childLinkName = Robot->Links.FindKey(joint.Value->ChildLink);
            if (!parentLinkName->IsEmpty() && !childLinkName->IsEmpty())
            {
                URRROS2JointTFComponent::AddJoint(joint.Value, *parentLinkName, *childLinkName, JointsTFPublisher);
            }
        }
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

void URRRobotROS2Interface::StopPublishers()
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

bool URRRobotROS2Interface::InitSubscriptions()
{
    if (false == IsValid(RobotROS2Node))
    {
        return false;
    }

    if (Robot && Robot->bMobileRobot)
    {
        // Subscription with callback to enqueue vehicle spawn info.
        RR_ROBOT_ROS2_SUBSCRIBE_TO_TOPIC(CmdVelTopicName, UROS2TwistMsg::StaticClass(), &URRRobotROS2Interface::MovementCallback);
    }

    if (Robot && Robot->bJointControl)
    {
        // Subscription with callback to enqueue vehicle spawn info.
        RR_ROBOT_ROS2_SUBSCRIBE_TO_TOPIC(
            JointCmdTopicName, UROS2JointStateMsg::StaticClass(), &URRRobotROS2Interface::JointCmdCallback);
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
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *client.Key, *GetName());
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
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *server.Key, *GetName());
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
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *client.Key, *GetName());
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
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("[%s] %s is nullptr. Please create before initialization."), *server.Key, *GetName());
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
        const FVector linear = URRConversionUtils::VectorROSToUE(twist.Linear);
        const FVector angular = URRConversionUtils::RotationROSToUEVector(twist.Angular, true);

        // (Note) In this callback, which could be invoked from a ROS working thread,
        // thus any direct referencing to its member in this GameThread lambda needs to be verified.
        AsyncTask(ENamedThreads::GameThread,
                  [this, linear, angular]
                  {
                      if (IsValid(Robot))
                      {
                          Robot->SetLinearVel(linear);
                          Robot->SetAngularVel(angular);
                      }
                  });
    }
}

void URRRobotROS2Interface::JointCmdCallback(const UROS2GenericMsg* Msg)
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
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("Effort control is not supported."));
            return;
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(
                LogRapyutaCore, Warning, TEXT("[position, velocity or effort array must be same size of name array"));
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
                    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("robot do not have joint named %s."), *jointState.Name[i]);
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
                UE_LOG_WITH_INFO(
                    LogRapyutaCore, Warning, TEXT("position, velocity or effort array must be same size of name array"));
                continue;
            }

            // ROS To UE conversion
            if (Robot->Joints[jointState.Name[i]]->LinearDOF == 1)
            {
                input[0] = URRConversionUtils::DistanceROSToUE(input[0]);
            }
            else if (Robot->Joints[jointState.Name[i]]->RotationalDOF == 1)
            {
                input[0] = FMath::RadiansToDegrees(input[0]);
            }
            else
            {
                UE_LOG_WITH_INFO(LogRapyutaCore,
                                 Warning,
                                 TEXT("[%s] Supports only single DOF joint. %s has %d "
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
                      if (!IsValid(Robot))
                      {
                          UE_LOG_WITH_INFO_NAMED(
                              LogRapyutaCore, Warning, TEXT("Robot is nullptr. RobotROS2Interface::Robot must not be nullptr."));
                          return;
                      }
                      Robot->SetJointState(joints, jointControlType);
                  });
    }
}

void URRRobotROS2Interface::UpdateJointState(UROS2GenericMsg* InMessage)
{
    if (nullptr == Robot)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Robot is not set."));
        return;
    }

    FROSJointState msg;
    msg.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(Robot->GetWorld()));

    for (const auto& joint : Robot->Joints)
    {
        if (nullptr == joint.Value)
        {
            continue;
        }

        msg.Name.Emplace(joint.Key);

        // UE to ROS conversion
        if (joint.Value->LinearDOF == 1)
        {
            msg.Position.Emplace(joint.Value->Position[0]);
            msg.Velocity.Emplace(joint.Value->LinearVelocity[0]);
        }
        else if (joint.Value->RotationalDOF == 1)
        {
            msg.Position.Emplace(FMath::DegreesToRadians(joint.Value->Orientation.Euler()[0]));
            msg.Velocity.Emplace(FMath::DegreesToRadians(joint.Value->AngularVelocity[0]));
        }

        msg.Effort.Emplace(0);    //effort is not supported yet.
    }
    CastChecked<UROS2JointStateMsg>(InMessage)->SetMsg(msg);
}

URRRobotROS2InterfaceComponent::URRRobotROS2InterfaceComponent()
{
    ROS2Interface = CastChecked<URRRobotROS2Interface>(
        URRUObjectUtils::CreateSelfSubobject(this, ROS2InterfaceClass, FString::Printf(TEXT("%sROS2Interface"), *GetName())));

    if (nullptr == Robot)
    {
        UE_LOG_WITH_INFO_NAMED(LogTemp,
                               Warning,
                               TEXT("Robot is nullptr. Trying to get "
                                    "owner as robot."));
        Robot = Cast<ARRBaseRobot>(GetOwner());
    }

    if (nullptr != Robot)
    {
        ROS2Interface->ROSSpawnParameters = Robot->ROSSpawnParameters;
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogTemp,
                               Warning,
                               TEXT("Robot is nullptr and Owner is not "
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
