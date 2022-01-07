// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"

#include "Turtlebot.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ATurtlebot : public ARobotVehicle
{
    GENERATED_BODY()

protected:
    virtual void ConfigureVehicleMoveComponent() override;
};
