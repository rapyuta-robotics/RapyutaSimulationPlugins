// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// RapyutaSimulationPlugins
#include "Robots/RRRobotVehicleROSController.h"

#include "TurtlebotROSController.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotROSController : public ARRRobotVehicleROSController
{
    GENERATED_BODY()

public:
    ATurtlebotROSController(const FObjectInitializer& ObjectInitializer);
};
