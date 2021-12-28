// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRLidar.h"

ARRLidar::ARRLidar()
{
    LidarComponent =
        Cast<URRBaseLidarComponent>(CreateDefaultSubobject(TEXT("Lidar"), LidarComponentClass, LidarComponentClass, true, false));
    UStaticMeshComponent* staticMeshComponent = GetStaticMeshComponent();
    USceneComponent* rootComponent =
        staticMeshComponent ? static_cast<USceneComponent*>(staticMeshComponent) : static_cast<USceneComponent*>(LidarComponent);
    SetRootComponent(rootComponent);
    if (LidarComponent != rootComponent)
    {
        LidarComponent->SetupAttachment(rootComponent);
    }
}
