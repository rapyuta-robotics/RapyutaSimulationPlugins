// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

// rclUE
#include "ROS2Node.h"
#include "ROS2Publisher.h"

#include "RRLidar.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRLidar : public AStaticMeshActor
{
    GENERATED_BODY()

public:
    ARRLidar();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRBaseLidarComponent* LidarComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TSubclassOf<URRBaseLidarComponent> LidarComponentClass = URRBaseLidarComponent::StaticClass();
};
