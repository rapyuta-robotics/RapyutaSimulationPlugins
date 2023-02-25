/**
 * @file RRCrowdAIController.h
 * @brief Base Crowd AI Controller for Pawn-based classes, utilizing URRCrowdFollowingComponent
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "AIController.h"

#include "RRCrowdAIController.generated.h"

/**
 * @brief Base Crowd AI Controller for Pawn-based classes, utilizing #URRCrowdFollowingComponent
 *
 * @sa [AAIController](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/)
 * @sa https://answers.unrealengine.com/questions/871116/view.html
 * @sa https://answers.unrealengine.com/questions/239159/how-many-ai-controllers-should-i-have.html
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRCrowdAIController : public AAIController
{
    GENERATED_BODY()
public:
    ARRCrowdAIController(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());
};
