// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotROS2Interface.h"

// rclUE
#include "Msgs/ROS2JointStateMsg.h"
#include "Msgs/ROS2TwistMsg.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotBaseVehicle.h"

void URRRobotROS2Interface::Initialize(ARRBaseRobot* InRobot)
{
    Robot = InRobot;
    Robot->ROS2Interface = this;

    // Instantiate a ROS2 node for InRobot
    InitRobotROS2Node(InRobot);

    // Initialize Robot's sensors (lidar, etc.)
    verify(InRobot->InitSensors(RobotROS2Node));

    // Refresh TF, Odom publishers
    verify(InitPublishers());
    InitSubscriptions();
}

void URRRobotROS2Interface::InitRobotROS2Node(ARRBaseRobot* InRobot)
{
    if (nullptr == RobotROS2Node)
    {
        RobotROS2Node = GetWorld()->SpawnActor<AROS2Node>();
    }
    RobotROS2Node->AttachToActor(InRobot, FAttachmentTransformRules::KeepRelativeTransform);
    // GUID is to make sure the node name is unique, even for multiple Sims?
    RobotROS2Node->Name = URRGeneralUtils::GetNewROS2NodeName(Robot->GetName());

    // Set robot's [ROS2Node] namespace from spawn parameters if existing
    UROS2Spawnable* rosSpawnParameters = InRobot->FindComponentByClass<UROS2Spawnable>();
    if (rosSpawnParameters)
    {
        RobotROS2Node->Namespace = rosSpawnParameters->GetNamespace();
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
    if (bPublishOdom)
    {
        if (nullptr == OdomPublisher)
        {
            OdomPublisher = NewObject<URRROS2OdomPublisher>(this);
            OdomPublisher->SetupUpdateCallback();
            OdomPublisher->bPublishOdomTf = bPublishOdomTf;
        }
        OdomPublisher->InitializeWithROS2(RobotROS2Node);
        OdomPublisher->RobotVehicle = Cast<ARRRobotBaseVehicle>(Robot);
    }
    return true;
}

void URRRobotROS2Interface::CreatePublisher(const FString& InTopicName,
                                            const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                            const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                                            int32 InPubFrequency,
                                            UROS2Publisher*& OutPublisher)
{
    if (nullptr == OutPublisher)
    {
        OutPublisher = UROS2Publisher::CreatePublisher(this, InTopicName, InPublisherClass, InMsgClass, InPubFrequency);
    }
    OutPublisher->InitializeWithROS2(RobotROS2Node);
}

void URRRobotROS2Interface::StopPublishers()
{
    if (bPublishOdom && OdomPublisher)
    {
        OdomPublisher->RevokeUpdateCallback();
        OdomPublisher->RobotVehicle = nullptr;
    }
}

void URRRobotROS2Interface::InitSubscriptions()
{
    // Subscription with callback to enqueue vehicle spawn info.
    RR_ROS2_SUBSCRIBE_TO_TOPIC(
        RobotROS2Node, this, CmdVelTopicName, UROS2TwistMsg::StaticClass(), &URRRobotROS2Interface::MovementCallback);

    // Subscription with callback to enqueue vehicle spawn info.
    RR_ROS2_SUBSCRIBE_TO_TOPIC(
        RobotROS2Node, this, JointsCmdTopicName, UROS2JointStateMsg::StaticClass(), &URRRobotROS2Interface::JointStateCallback);
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
        const FVector linear(URRConversionUtils::VectorROSToUE(twist.linear));
        const FVector angular(URRConversionUtils::RotationROSToUE(twist.angular));

        // (Note) In this callback, which could be invoked from a ROS working thread,
        // thus any direct referencing to its member in this GameThread lambda needs to be verified.
        AsyncTask(ENamedThreads::GameThread,
                  [this, linear, angular]
                  {
                      ARRRobotBaseVehicle* robotVehicle = Cast<ARRRobotBaseVehicle>(Robot);
                      if (robotVehicle)
                      {
                          check(IsValid(robotVehicle));
                          robotVehicle->SetLinearVel(linear);
                          robotVehicle->SetAngularVel(angular);
                      }
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
        if (jointState.name.Num() == jointState.position.Num())
        {
            jointControlType = ERRJointControlType::POSITION;
        }
        else if (jointState.name.Num() == jointState.velocity.Num())
        {
            jointControlType = ERRJointControlType::VELOCITY;
        }
        else if (jointState.name.Num() == jointState.effort.Num())
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
        for (auto i = 0; i < jointState.name.Num(); ++i)
        {
            if (!Robot->Joints.Contains(jointState.name[i]))
            {
                if (bWarnAboutMissingLink)
                {
                    UE_LOG(LogRapyutaCore,
                           Warning,
                           TEXT("[%s] [URRRobotROS2Interface] [JointStateCallback] vehicle do not have joint named %s."),
                           *GetName(),
                           *jointState.name[i]);
                }
                continue;
            }

            TArray<float> input;
            if (ERRJointControlType::POSITION == jointControlType)
            {
                input.Add(jointState.position[i]);
            }
            else if (ERRJointControlType::VELOCITY == jointControlType)
            {
                input.Add(jointState.velocity[i]);
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
            if (Robot->Joints[jointState.name[i]]->LinearDOF == 1)
            {
                input[0] *= 100;    // todo add conversion to conversion util
            }
            else if (Robot->Joints[jointState.name[i]]->RotationalDOF == 1)
            {
                input[0] *= 180 / M_PI;    // todo add conversion to conversion util
            }
            else
            {
                UE_LOG(LogRapyutaCore,
                       Warning,
                       TEXT("[%s] [URRRobotROS2Interface] [JointStateCallback] Supports only single DOF joint. %s has %d "
                            "linear DOF and %d rotational DOF"),
                       *jointState.name[i],
                       Robot->Joints[jointState.name[i]]->LinearDOF,
                       Robot->Joints[jointState.name[i]]->RotationalDOF);
            }

            joints.Emplace(jointState.name[i], input);
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
