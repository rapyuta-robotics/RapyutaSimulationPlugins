/**
 * @file RRCrowdAIController.h
 * @brief Base Crowd AI Controller for Character-based classes, utilizing UCrowdFollowingComponent
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "AIController.h"

#include "RRCrowdAIController.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRCrowdAIController : public AAIController
{
    GENERATED_BODY()
public:
    ARRCrowdAIController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());
};
