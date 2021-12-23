// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// std
#include <random>

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "ROS2Node.h"
#include "ROS2Publisher.h"

#include "RRLidar.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRLidar : public AActor
{
    GENERATED_BODY()

public:
    ARRLidar();

    UPROPERTY()
    URRBaseLidarComponent* LidarComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URRBaseLidarComponent> LidarComponentClass = URRBaseLidarComponent::StaticClass();
};
