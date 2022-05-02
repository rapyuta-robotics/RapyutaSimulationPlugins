/**
 * @file RRProceduralMeshComponent.h
 * @brief Procedural mesh components. this class is used to spawn robot and object from ROS2 service.
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "ProceduralMeshComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshData.h"
#include "Core/RRUObjectUtils.h"

#include "RRProceduralMeshComponent.generated.h"

/**
 * @brief Procedural mesh components. this class is used to spawn robot and object from ROS2 service.
 * 
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRProceduralMeshComponent : public UProceduralMeshComponent
{
    GENERATED_BODY()

public:

    /**
     * @brief Construct a new URRProceduralMeshComponent object
     * This Initializer-based ctor is used due to [UProceduralMeshComponent] still having it.
     * The collision cooking is critical for sweeping movement to work after spawning Proc mesh actor.
     * Due to [FinishPhysicsAsyncCook] being not virtual and private, it is unable to catch [FOnAsyncPhysicsCookFinished] event
     * Thus we could only rely on [UBodySetup::bCreatedPhysicsMeshes]
     * @param ObjectInitializer 
     */
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

    /**
     * @brief Initialize mesh. initialization is different based on mesh type.
     * Uses #URRThreadUtils::DoAsyncTaskInThread and #URRThreadUtils::DoTaskInGameThread to load Mesh
     * @param InMeshFileName 
     * @return true 
     * @return false 
     */
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

    /**
     * @brief Create a Mesh Body object
     * This function is hooked up from an async task running in GameThread
     * @return true 
     * @return false
     *
     * @todo explain impl detail.
     */
    bool CreateMeshBody();

    void CreateMeshSection(const TArray<FRRMeshNodeData>& InMeshSectionData);
    
    void FinalizeMeshBodyCreation(UBodySetup* InBodySetup, const FString& InBodySetupModelName);
};
