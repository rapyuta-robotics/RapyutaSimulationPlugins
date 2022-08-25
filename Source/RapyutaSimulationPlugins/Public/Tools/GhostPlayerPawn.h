/**
 * @file GhostPlayerPawn.h
 * @brief BaseComponents which is used when spawning Actor from ROS2 service in #ASimulationState.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/DefaultPawn.h"

#include "GhostPlayerPawn.generated.h"

class USphereComponent;

/**
 * @brief Default player pawn without collision and mesh
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API AGhostPlayerPawn : public ADefaultPawn
{
    GENERATED_BODY()

public:
    AGhostPlayerPawn();
};
