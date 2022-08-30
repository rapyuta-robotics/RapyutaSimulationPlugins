// Copyright 2020-2022 Rapyuta Robotics Co., Ltd

#include "Core/RRCrowdFollowingComponent.h"

URRCrowdFollowingComponent::URRCrowdFollowingComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SimulationState = ECrowdSimulationState::ObstacleOnly;
}
