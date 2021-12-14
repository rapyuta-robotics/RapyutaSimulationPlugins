// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "RRROS2GameMode.h"

// rclUE
#include "Msgs/ROS2StringMsg.h"
#include "ROS2Node.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Tools/SimulationState.h"

void ARRROS2GameMode::BeginPlay()
{
    Super::BeginPlay();

    UWorld* currentWorld = GetWorld();
    ROS2Node = currentWorld->SpawnActor<AROS2Node>();
    ROS2Node->Namespace.Reset();
    ROS2Node->Init();

    // Create Clock publisher
    ClockPublisher = NewObject<UROS2Publisher>(this);
    ClockPublisher->RegisterComponent();
    ClockPublisher->TopicName = TEXT("clock");
    ClockPublisher->PublicationFrequencyHz = 1;
    ClockPublisher->MsgClass = UROS2StringMsg::StaticClass();

    // [ClockPublisher] must be registered to [ROS2Node] before being initialized
    ROS2Node->AddPublisher(ClockPublisher);
    ClockPublisher->Init(UROS2QoS::ClockPub);

    SimulationState = currentWorld->SpawnActor<ASimulationState>();
    SimulationState->ROSServiceNode = ROS2Node;
}
