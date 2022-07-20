/**
 * @file RRROS2ImagePublisher.h
 * @brief Image publisher class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2ImagePublisher.generated.h"

class URRROS2CameraComponent;

/**
 * @brief Image publisher class
 * 
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2ImagePublisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2ImagePublisher();
};
