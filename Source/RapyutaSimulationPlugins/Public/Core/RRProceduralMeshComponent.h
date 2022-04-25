// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "ProceduralMeshComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshData.h"
#include "Core/RRUObjectUtils.h"

#include "RRProceduralMeshComponent.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRProceduralMeshComponent : public UProceduralMeshComponent
{
    GENERATED_BODY()

public:
    // This Initializer-based ctor is used due to [UProceduralMeshComponent] still having it
    URRProceduralMeshComponent(const FObjectInitializer& ObjectInitializer);

    UPROPERTY()
    FString MeshUniqueName;
    FString GetBodySetupModelName() const
    {
        return URRUObjectUtils::ComposeDynamicResourceName(TEXT("BS"), MeshUniqueName);
    }

    UPROPERTY()
    ERRShapeType ShapeType = ERRShapeType::INVALID;

    void Initialize(bool bIsStaticBody, bool bInIsPhysicsEnabled)
    {
    }
    bool InitializeMesh(const FString& InMeshFileName);
    FOnMeshCreationDone OnMeshCreationDone;

    UPROPERTY()
    FTransform OriginRelativeTransform = FTransform::Identity;

    UPROPERTY()
    bool bIsStationary = false;

    void SetMeshSize(const FVector& InSize);
    FVector GetSize() const
    {
        // TBD
        return FVector::ZeroVector;
    }

    FVector GetExtent() const
    {
        return 0.5f * GetSize();
    }

    void LockSelf()
    {
    }

    void HideSelf(bool IsHidden)
    {
    }

    // Collision/Overlapping --
    void SetCollisionModeAvailable(bool bIsOn, bool bIsHitEventEnabled = false);
    void EnableOverlapping();

    bool IsMeshDataValid() const;
    bool GetMeshData(FRRMeshData& OutMeshData, bool bFromBuffer);
    void GetLocalBounds(FVector& OutMinBounds, FVector& OutMaxBounds)
    {
    }

protected:
    virtual bool ShouldCreatePhysicsState() const override
    {
        return true;
    }

private:
    // This is used as a buffer storing loaded mesh data in worker thread,
    // thus needs to belong to each individual mesh comp of its own
    // As such, this just exists transiently and should not be queried. Use FRRMeshData::MeshDataStore instead
    UPROPERTY()
    FRRMeshData MeshDataBuffer;

    UPROPERTY()
    FTimerHandle CollisionCookingTimerHandle;
    UPROPERTY()
    FTimerHandle BodySetupTimerHandle;

    bool CreateMeshBody();
    void CreateMeshSection(const TArray<FRRMeshNodeData>& InMeshSectionData);
    void FinalizeMeshBodyCreation(UBodySetup* InBodySetup, const FString& InBodySetupModelName);
};
