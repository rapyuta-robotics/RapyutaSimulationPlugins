// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRLidar.h"

void ARRLidar::OnConstruction(const FTransform& InTransform)
{
    Super::OnConstruction(InTransform);
    if (LidarComponentClass)
    {
        LidarComponent =
            NewObject<URRBaseLidarComponent>(this, LidarComponentClass, *FString::Printf(TEXT("%s_LidarComp"), *GetName()));

        // Set Root Component
        UStaticMeshComponent* staticMeshComponent = GetStaticMeshComponent();
        USceneComponent* newRootComponent = staticMeshComponent ? static_cast<USceneComponent*>(staticMeshComponent)
                                                                : static_cast<USceneComponent*>(LidarComponent);
        SetRootComponent(newRootComponent);
        if (LidarComponent != newRootComponent)
        {
            LidarComponent->SetupAttachment(newRootComponent);
        }
        LidarComponent->RegisterComponent();
    }
    else
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[LidarComponentClass] has not been configured!"));
    }
}
