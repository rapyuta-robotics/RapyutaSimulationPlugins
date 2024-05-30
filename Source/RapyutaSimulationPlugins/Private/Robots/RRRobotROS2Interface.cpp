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

void URRRobotROS2Interface::Initialize(AActor* Owner)
{
    Robot = Cast<ARRBaseRobot>(Owner);
    if (nullptr == Robot)
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("Owner should be child class of RRBaseRobot."));
        return;
    }

    Robot->ROS2Interface = this;
    ROSSpawnParameters = Robot->ROSSpawnParameters;

    Super::Initialize(Owner);
}

void URRRobotROS2Interface::InitInterfaces()
{
    if (!Robot)
    {
        return;
    }

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
    verify(Robot->InitSensors(RobotROS2Node));
    Super::InitInterfaces();
}

void URRRobotROS2Interface::DeInitialize()
{
    Super::DeInitialize();

    if (nullptr != Robot)
    {
        Robot->ROS2Interface = nullptr;
        Robot = nullptr;
    }
}

void URRRobotROS2Interface::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URRRobotROS2Interface, Robot);
    DOREPLIFETIME(URRRobotROS2Interface, OdomComponent);
    DOREPLIFETIME(URRRobotROS2Interface, bPublishOdom);
    DOREPLIFETIME(URRRobotROS2Interface, bPublishOdomTf);
    DOREPLIFETIME(URRRobotROS2Interface, OdomPublicationFrequencyHz);
    DOREPLIFETIME(URRRobotROS2Interface, CmdVelTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, JointCmdTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, JointStateTopicName);
    DOREPLIFETIME(URRRobotROS2Interface, bWarnAboutMissingLink);
}

void URRRobotROS2Interface::InitROS2NodeParam(AActor* Owner)
{
    Super::InitROS2NodeParam(Owner);
    if (!ROSSpawnParameters && Robot)
    {
        RobotROS2Node->Namespace = Robot->RobotUniqueName;
        if (bUseActorNameAsNamespace)
        {
#if WITH_EDITOR
            RobotROS2Node->Namespace = UKismetSystemLibrary::GetDisplayName(Owner);
#else
            RobotROS2Node->Namespace = Owner->GetName();
#endif
        }
    }
}

bool URRRobotROS2Interface::InitPublishers()
{
    if (!Super::InitPublishers())
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

    return true;
}

bool URRRobotROS2Interface::InitSubscriptions()
{
    if (!Super::InitSubscriptions())
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
            msg.Position.Emplace(URRConversionUtils::DistanceUEToROS(joint.Value->Position[0]));
            msg.Velocity.Emplace(URRConversionUtils::DistanceUEToROS(joint.Value->LinearVelocity[0]));
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
