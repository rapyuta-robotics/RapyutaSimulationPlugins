// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#include "Core/RRCrowdAIController.h"

// RapyutaSimulationPlugins
#include "Core/RRCrowdFollowingComponent.h"

ARRCrowdAIController::ARRCrowdAIController(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<URRCrowdFollowingComponent>(TEXT("PathFollowingComponent")))
{
}
