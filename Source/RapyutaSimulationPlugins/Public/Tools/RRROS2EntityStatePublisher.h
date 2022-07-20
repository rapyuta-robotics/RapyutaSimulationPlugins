/**
 * @file RRROS2EntityStatePublisher.h
 * @brief Entity(Actor) state publisher class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */



#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2EntityStatePublisher.generated.h"

/**
 * @brief Entity(Actor) state publisher class
 * @todo add ROSService to change reference frame
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2EntityStatePublisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2EntityStatePublisher();
};
