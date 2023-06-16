/**
 * @file RRStaticMeshComponent.h
 * @brief static mesh component
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/StaticMeshComponent.h"
#include "CoreMinimal.h"
#include "MeshDescriptionBuilder.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshData.h"
#include "Core/RRUObjectUtils.h"

#include "RRStaticMeshComponent.generated.h"

// [URRStaticMeshComponent] - COMPONENT OF [ARRMeshActor] --
/**
 * @brief Component of #ARRMeshActor. UStaticMeshComponent with utils.
 * @todo documentation.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRStaticMeshComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
    friend class URRObjectUtils;

public:
    URRStaticMeshComponent();
    static constexpr int8 CUSTOM_DEPTH_STENCIL_VOID = 0;

    /**
     * @brief Set Mobility and Physics Enabled.
     * @note Upon creation, DO NOT enable physics here yet, which will makes mesh comp detached from its parent.
     * Physics should only be enabled after children mesh comps have also been created, so they will be welded.
     * Refer to [FBodyInstance::SetInstanceSimulatePhysics()]
     *
     * @param bInIsStationary
     * @param bInIsPhysicsEnabled
     */
    void Initialize(bool bInIsStationary, bool bInIsPhysicsEnabled);

    /**
     * @brief Initialize mesh. initialization is different based on mesh type, #ERRShapeType, which can be get from #URRGameSingleton::GetShapeTypeFromMeshName
     * Uses #URRThreadUtils::DoAsyncTaskInThread and #URRThreadUtils::DoTaskInGameThread to load Mesh
     * If ShapeType is not #ERRShapeType::MESH i.e. primitive-shape, initialization is done by #SetMesh
     * @param InMeshFileName
     * @return true
     * @return false
     */
    bool InitializeMesh(const FString& InMeshFileName);

    FOnMeshCreationDone OnMeshCreationDone;

    UPROPERTY()
    int8 SceneInstanceId = 0;

    UPROPERTY()
    URRActorCommon* ActorCommon = nullptr;

    UPROPERTY()
    FString MeshUniqueName;
    /**
     * @brief Create a Mesh Body object
     * @note This function could be invoked from an async task running in GameThread
     * @param InMeshData
     * @return UStaticMesh*
     */
    UStaticMesh* CreateMeshBody(const FRRMeshData& InMeshData);

    /**
     * @brief Set Static mesh from UstaticMesh
     * @sa [SetStaticMesh](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Components/UStaticMeshComponent/SetStaticMesh/)
     *
     * @param InStaticMesh
     */
    void SetMesh(UStaticMesh* InStaticMesh);

    void SetMeshSize(const FVector& InSize)
    {
        SetWorldScale3D(InSize / GetStaticMesh()->GetBoundingBox().GetSize());
    }

    UPROPERTY()
    ERRShapeType ShapeType = ERRShapeType::NONE;

    UPROPERTY()
    FTransform OriginRelativeTransform;

    UPROPERTY()
    bool bIsStationary = false;

    //! Whether static mesh is built with simple collision, also configurable in ARRMeshActor::Initialize()
    UPROPERTY()
    bool bUseDefaultSimpleCollision = true;

    //! Whether a complex collision mesh is built alongside the visual static mesh
    //! Meant for ECollisionTraceFlag::CTF_UseComplexAsSimple, which only works with ECollisionChannel::ECC_WorldStatic
    UPROPERTY()
    bool bUseComplexCollision = false;

    /**
     * @brief Create a static mesh
     * @param InMeshData
     * @param bInAsVisualMesh Whether it is created as a visual or collision mesh
     * @return UStaticMesh
     */
    UStaticMesh* CreateMesh(const FRRMeshData& InMeshData, bool bInAsVisualMesh);

    /**
     * @brief Generate custom simple collision, only if not #bUseDefaultSimpleCollision
     */
    void GenerateCustomSimpleCollision(const FRRMeshData& InMeshData, UBodySetup* OutBodySetup);

    /**
     * @brief Get the Size of bounding box of the mesh.
     *
     * @return FVector
     */
    UFUNCTION()
    FVector GetSize() const;

    /**
     * @brief Get the extent of bounding box of the mesh.
     *
     * @return FVector
     */
    UFUNCTION()
    FVector GetExtent() const;

    UFUNCTION()
    FVector GetScaledExtent() const;

    /**
     * @brief  This function is used proprietarily for Generic Link/Joint (Non-Articulation) structure.
     *
     */
    UFUNCTION()
    void LockSelf();

    UFUNCTION()
    void HideSelf(bool bInHidden);

    // Collision/Overlapping --
    /**
     * @brief Set the Collision Mode with some preset parameters
     * @todo is preset parameter cover all use case?
     * @param bInCollisionEnabled
     * @param bInHitEventEnabled
     */
    UFUNCTION()
    void SetCollisionModeAvailable(bool bInCollisionEnabled, bool bInHitEventEnabled = false);

    UFUNCTION()
    void EnableOverlapping(bool bOverlapEventEnabled);

protected:
    /**
    * @brief
    * It is considered that a mesh component should only be physically activated if its Owning Actor is Physically Enabled,
    * thus not auto activating physics here! Rather, it is left to the Owning actor to do it!
    */
    virtual void BeginPlay() override;

private:
    UPROPERTY()
    FTimerHandle StaticMeshTimerHandle;

    void CreateMeshSection(const TArray<FRRMeshNodeData>& InMeshSectionData, FMeshDescriptionBuilder& OutMeshDescBuilder);
};
