/**
 * @file RRMeshActor.h
 * @brief Mesh Actor
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRProceduralMeshComponent.h"
#include "Core/RRStaticMeshComponent.h"
#include "Core/RRUObjectUtils.h"
#include "RapyutaSimulationPlugins.h"

#include "RRMeshActor.generated.h"

DECLARE_DELEGATE_OneParam(FOnMeshActorDeactivated, ARRMeshActor*);

/**
 * @brief Mesh actor.#ARRBaseActor with list of #UMeshComponent
 * @sa #RRProceduralMeshComponent
 * @sa #RRStaticMeshComponent
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRMeshActor : public ARRBaseActor
{
    GENERATED_BODY()
public:
    /**
     * @brief Construct a new ARRMeshActor object. Create and setup a scene component as root.
     */
    ARRMeshActor();

public:
    /**
     * @brief ACTOR INTIALIZING GENERAL INFO (Unique name, mesh list, material list, etc.)
     * 
     * @tparam TActorSpawnInfo 
     * @param InActorInfo 
     * @return true 
     * @return false 
     */
    template<typename TActorSpawnInfo>
    bool InitializeWithSpawnInfo(const TActorSpawnInfo& InActorInfo)
    {
        ActorInfo = MakeShared<TActorSpawnInfo>(InActorInfo);

        // ACTOR INTIALIZING GENERAL INFO (Unique name, mesh list, material list, etc.)
        return Initialize();
    }

    /**
     * @brief Initialize MeshActor by 1)Create child mesh components, 2)#SetCustomDepthEnabled, 3)Set mobility. 
     * 
     * @return true 
     * @return false 
     */
    virtual bool Initialize() override;

    /**
     * @brief Checm #MeshCompList and #BaseMeshComp
     * 
     * @param bIsLogged 
     * @return true 
     * @return false 
     */
    virtual bool HasInitialized(bool bIsLogged = false) const override;

    /**
     * @brief Reset #MeshCompList
     * 
     */
    virtual void Reset() override;
    void DrawTransform();

public:
    //! Body mesh component list
    UPROPERTY(VisibleAnywhere)
    TArray<UMeshComponent*> MeshCompList;

    //! Created mesh components num up to the moment
    UPROPERTY()
    int32 CreatedMeshesNum = 0;

    //! Planned num of mesh components to be created
    UPROPERTY()
    int32 ToBeCreatedMeshesNum = 0;

    //! Base mesh comp, normally also as the root comp
    UPROPERTY(VisibleAnywhere)
    UMeshComponent* BaseMeshComp = nullptr;

    /**
     * @brief Get #BaseMeshComp's material
     * @param InMaterialIndex
     */
    UMaterialInterface* GetBaseMeshMaterial(int32 InMaterialIndex = 0) const
    {
        return BaseMeshComp ? BaseMeshComp->GetMaterial(InMaterialIndex) : nullptr;
    }

    /**
     * @brief Declare mesh actor full creation with all meshes created
     * 
     * @param bInCreationResult 
     */
    virtual void DeclareFullCreation(bool bInCreationResult);

public:
    //! Cell index if arranged in a grid
    UPROPERTY()
    FIntVector CellIdx = FIntVector::ZeroValue;

    /**
     * @brief Get body mesh component
     * @param Index
     */
    UMeshComponent* GetMeshComponent(int32 Index = 0) const;

    /**
     * @brief Create mesh component list
     * @tparam TMeshComp
     * @param InParentComp
     * @param InMeshUniqueNameList
     * @param InMeshRelTransf
     * @param InMaterialNameList
     */
    template<typename TMeshComp>
    TArray<TMeshComp*> CreateMeshComponentList(USceneComponent* InParentComp,
                                               const TArray<FString>& InMeshUniqueNameList,
                                               const TArray<FTransform>& InMeshRelTransf = TArray<FTransform>(),
                                               const TArray<FString>& InMaterialNameList = TArray<FString>())
    {
        // (Note) This method could be invoked multiple times
        TArray<TMeshComp*> addedMeshCompList;
        if (InMeshRelTransf.Num() > 0)
        {
            verify(InMeshRelTransf.Num() == InMeshUniqueNameList.Num());
        }
        if (InMaterialNameList.Num() > 0)
        {
            verify(InMaterialNameList.Num() == InMeshUniqueNameList.Num());
        }

        TMeshComp* meshComp = nullptr;
        for (auto i = 0; i < InMeshUniqueNameList.Num(); ++i)
        {
            const FString& meshUniqueName = InMeshUniqueNameList[i];

            // [ProcMeshComp] Verify path as absolute & existing
            if ((false == FPaths::IsRelative(meshUniqueName)) && (false == FPaths::FileExists(meshUniqueName)))
            {
                UE_LOG_WITH_INFO(LogTemp, Error, TEXT("Mesh invalid [%s] is non-existent"), *meshUniqueName);
                continue;
            }

            // [OBJECT MESH COMP] --
            //
            meshComp = URRUObjectUtils::CreateMeshComponent<TMeshComp>(
                this,
                meshUniqueName,
                FString::Printf(TEXT("%s_MeshComp_%u"), *ActorInfo->UniqueName, MeshCompList.Num()),
                InMeshRelTransf.IsValidIndex(i) ? InMeshRelTransf[i] : FTransform::Identity,
                ActorInfo->bIsStationary,
                ActorInfo->bIsPhysicsEnabled,
                ActorInfo->bIsCollisionEnabled,
                ActorInfo->bIsOverlapEventEnabled,
                InParentComp);

            if (meshComp)
            {
                // (Note) This must be the full path to the mesh file on disk
                if (meshComp->InitializeMesh(meshUniqueName))
                {
                    addedMeshCompList.AddUnique(meshComp);
                }
                else
                {
                    UE_LOG_WITH_INFO_NAMED(LogTemp, Error, TEXT("Failed initializing mesh comp[%s]"), *meshUniqueName);
                }
            }
            else
            {
                UE_LOG_WITH_INFO(LogTemp,
                                 Error,
                                 TEXT("[%s:%d] - Failed creating child Mesh Component [%s]!"),
                                 *ActorInfo->UniqueName,
                                 this,
                                 *meshUniqueName);
            }

            // [MeshCompList] <- [addedMeshCompList]
            MeshCompList.Append(addedMeshCompList);
        }

        // Change RootComponent -> BaseMeshComp
        if ((nullptr == BaseMeshComp) && (MeshCompList.Num() > 0))
        {
            BaseMeshComp = MeshCompList[0];

            if (BaseMeshComp->GetRelativeTransform().Equals(FTransform::Identity))
            {
                // Set as Root Component
                // Set the main mesh comp as the root
                // (Not clear why using the default scene component as the root just disrupts actor-children relative movement,
                // and thus also compromise the actor transform itself)!
                USceneComponent* oldRoot = RootComponent;
                SetRootComponent(BaseMeshComp);
                if (oldRoot)
                {
                    oldRoot->DestroyComponent();
                }
            }
        }

        return addedMeshCompList;
    }

    /**
     * @brief Callback as a body mesh is created
     * @param bInCreationResult
     * @param InMeshBodyComponent
     */
    virtual void OnBodyComponentMeshCreationDone(bool bInCreationResult, UObject* InMeshBodyComponent);

    /**
     * @brief Enable/Disable Custom Depth rendering pass
     * @param bIsCustomDepthEnabled 
     */
    void SetCustomDepthEnabled(bool bIsCustomDepthEnabled);
    
    /**
     * @brief Set Custom Depth Stencil value uniformly for all child mesh comps
      * 
      * @param InCustomDepthStencilValue 
      */
    void SetCustomDepthStencilValue(int32 InCustomDepthStencilValue);

    /**
     * @brief Whether Custom depth rendering is enabled
      * 
      * @return true 
      * @return false 
      */
    bool IsCustomDepthEnabled() const;

    /**
     * @brief Get mesh comps' custom depth stencil values
      * @return TArray<int32> 
      */
    TArray<int32> GetCustomDepthStencilValueList() const;

    //! Delegate on mesh actor being deactivated
    FOnMeshActorDeactivated OnDeactivated;

    /**
     * @brief Activate/Deactivate mesh actor
      * 
      * @param bInIsActivated 
      */
    FORCEINLINE virtual void SetActivated(bool bInIsActivated)
    {
#if RAPYUTA_SIM_VISUAL_DEBUG
        // Visible/Invisibile
        SetActorHiddenInGame(!bInIsActivated);
#endif

        // Then teleport itself to a camera-blind location if being deactivated,
        // so when it get activated back, it would not happen to appear at an unintended pose
        if (false == bInIsActivated)
        {
            CellIdx = FIntVector::NoneValue;
            SetActorLocation(FVector(0.f, 0.f, -5000.f));
            OnDeactivated.ExecuteIfBound(this);
        }
        else if (CellIdx == FIntVector::NoneValue)
        {
            CellIdx = FIntVector::ZeroValue;
        }

        // RenderCustomDepth (must be after [OnDeactivated], thus its current CustomDepthStencilValue could be stored)
        SetCustomDepthEnabled(bInIsActivated);
    }

    /**
     * @brief Whether mesh actor is activated
     */
    bool IsActivated() const
    {
        return (CellIdx != FIntVector::NoneValue);
    }

protected:
    //! Last body mesh creation result
    UPROPERTY(VisibleAnywhere)
    uint8 bLastMeshCreationResult : 1;

    //! Whether all body meshes are fully created
    UPROPERTY(VisibleAnywhere)
    uint8 bFullyCreated : 1;
};
