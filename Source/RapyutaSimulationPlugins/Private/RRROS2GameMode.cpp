// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "RRROS2GameMode.h"

// rclUE
#include "Msgs/ROS2ClockMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2ClockPublisher.h"
#include "Tools/SimulationState.h"

ARRROS2GameMode::ARRROS2GameMode(){
    ClockPublisher = CreateDefaultSubobject<URRROS2ClockPublisher>(TEXT("ClockPublisher"));
    SimulationState = CreateDefaultSubobject<ASimulationState>(TEXT("SimulationState"));
}

void ARRROS2GameMode::Init()
{
    UWorld* currentWorld = GetWorld();
    ROS2Node = currentWorld->SpawnActor<AROS2Node>();
    ROS2Node->Namespace.Reset();
    ROS2Node->Name = UENodeName;
    ROS2Node->Init();

    // Create Clock publisher
    // ClockPublisher = NewObject<URRROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
    ClockPublisher->InitializeWithROS2(ROS2Node);

    // Simulation state
    // SimulationState = currentWorld->SpawnActor<ASimulationState>();
    // SimulationState->ROSServiceNode = ROS2Node;
    SimulationState->InitializeWithROS2(ROS2Node);
}

void ARRROS2GameMode::BeginPlay()
{
    Super::BeginPlay();
    if (bAutoStart)
    {
        Init()
    }

}
