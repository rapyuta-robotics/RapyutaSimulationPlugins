// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "RRROS2GameMode.h"

// rclUE
#include "Msgs/ROS2ClockMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/ROS2ClockPublisher.h"
#include "Tools/SimulationState.h"

void ARRROS2GameMode::BeginPlay()
{
    Super::BeginPlay();

    UWorld* currentWorld = GetWorld();
    ROS2Node = currentWorld->SpawnActor<AROS2Node>();
    ROS2Node->Namespace.Reset();
    ROS2Node->Init();

    // Create Clock publisher
    ClockPublisher = NewObject<UROS2ClockPublisher>(this);
    // ClockPublisher's RegisterComponent() is done by [AROS2Node::AddPublisher()]
    ClockPublisher->InitializeWithROS2(ROS2Node);

    // Simulation state
    SimulationState = currentWorld->SpawnActor<ASimulationState>();
    SimulationState->ROSServiceNode = ROS2Node;
}
