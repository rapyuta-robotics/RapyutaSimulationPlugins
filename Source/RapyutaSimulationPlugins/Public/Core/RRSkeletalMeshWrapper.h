// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"

// RapyutaSimRobotImporter
#include "Core/RRMeshData.h"

class RAPYUTASIMULATIONPLUGINS_API RRSkeletalMeshWrapper
{
public:
    void Init(USkeletalMeshComponent* InSkeletalMeshComponent);
    
    void PrintInfo() const;

    UPROPERTY()
    TArray<FRRMeshData> MeshDataSet;

    FVector GetSize() const
    {
        return 2 * SkeletalMeshComponent->SkeletalMesh->GetBounds().BoxExtent;
    }

    FVector GetExtent() const
    {
        return SkeletalMeshComponent->SkeletalMesh->GetBounds().BoxExtent;
    }

    void GetLocalBounds(FVector& OutMinBounds, FVector& OutMaxBounds)
    {
        FBoxSphereBounds bounds = SkeletalMeshComponent->SkeletalMesh->GetBounds();
        OutMinBounds = bounds.Origin - bounds.BoxExtent;
        OutMaxBounds = bounds.Origin + bounds.BoxExtent;
    }

    USkeletalMeshComponent* SkeletalMeshComponent;
};