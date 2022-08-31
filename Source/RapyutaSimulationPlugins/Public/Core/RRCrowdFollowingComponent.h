/**
 * @file RRCrowdFollowingComponent.h
 * @brief Base component for crowd path following movement
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Navigation/CrowdFollowingComponent.h"

#include "RRCrowdFollowingComponent.generated.h"

/**
 * @brief Base component for crowd path following movement
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRCrowdFollowingComponent : public UCrowdFollowingComponent
{
    GENERATED_BODY()

public:
    URRCrowdFollowingComponent(const FObjectInitializer& ObjectInitializer);

    bool IsIdle() const
    {
        return (EPathFollowingStatus::Idle == Status);
    }

    bool IsReadyForNewMovementOrder() const
    {
        return IsIdle();
    }
};
