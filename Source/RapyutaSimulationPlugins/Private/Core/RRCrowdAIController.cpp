// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#include "Core/RRCrowdAIController.h"

// UE
#include "Navigation/CrowdFollowingComponent.h"

ARRCrowdAIController::ARRCrowdAIController(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UCrowdFollowingComponent>(TEXT("PathFollowingComponent")))
{
}
