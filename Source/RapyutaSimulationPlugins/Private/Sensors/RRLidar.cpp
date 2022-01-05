// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRLidar.h"

void ARRLidar::OnConstruction(const FTransform& InTransform)
{
    Super::OnConstruction(InTransform);
    if (LidarComponentClass)
    {
        // (NOTE) Being created in [OnConstruction], PIE will cause this to be reset anyway, thus requires recreation
        LidarComponent =
            NewObject<URRBaseLidarComponent>(this, LidarComponentClass, *FString::Printf(TEXT("%s_LidarComp"), *GetName()));
        LidarComponent->RegisterComponent();

        // Set Root Component
        UStaticMeshComponent* staticMeshComponent = GetStaticMeshComponent();
        USceneComponent* newRootComponent = staticMeshComponent ? static_cast<USceneComponent*>(staticMeshComponent)
                                                                : static_cast<USceneComponent*>(LidarComponent);
        SetRootComponent(newRootComponent);
        if (LidarComponent != newRootComponent)
        {
            // (NOTE) SetupAttachment() should mostly be called as in ctor, when the comp is created with CreateDefaultSubobject()
            LidarComponent->AttachToComponent(newRootComponent, FAttachmentTransformRules::KeepRelativeTransform);
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore, Fatal, TEXT("[%s] [LidarComponentClass] has not been configured!"), *GetName());
    }
}
