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
/**
 * @brief Mesh actor.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRMeshActor : public ARRBaseActor
{
    GENERATED_BODY()
public:
    /**
     * @brief Construct a new ARRMeshActor object
     *
     */
    ARRMeshActor();

public:
    template<typename TActorSpawnInfo>
    bool InitializeWithSpawnInfo(const TActorSpawnInfo& InActorInfo)
    {
        ActorInfo = MakeShared<TActorSpawnInfo>(InActorInfo);

        // ACTOR INTIALIZING GENERAL INFO (Unique name, mesh list, material list, etc.)
        return Initialize();
    }

    virtual bool Initialize() override;
    virtual bool HasInitialized(bool bIsLogged = false) const override;
    virtual void Reset() override;

public:
    UPROPERTY(VisibleAnywhere)
    TArray<UMeshComponent*> MeshCompList;
    UPROPERTY()
    int32 CreatedMeshesNum = 0;
    UPROPERTY()
    int32 ToBeCreatedMeshesNum = 0;

    UPROPERTY(VisibleAnywhere)
    UMeshComponent* BaseMeshComp = nullptr;
    UMaterialInterface* GetBaseMeshMaterial(int32 InMaterialIndex = 0) const
    {
        return BaseMeshComp ? BaseMeshComp->GetMaterial(InMaterialIndex) : nullptr;
    }

    UPROPERTY()
    TArray<ARRMeshActor*> PartnerList;
    UPROPERTY()
    FIntVector CellIdx = FIntVector::ZeroValue;

    virtual void DeclareFullCreation(bool bInCreationResult);

public:
    void DrawTransform();

    UMeshComponent* GetMeshComponent(int32 Index = 0) const;
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
                UE_LOG(LogTemp, Error, TEXT("Mesh invalid [%s] is non-existent"), *meshUniqueName);
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
                    UE_LOG(LogTemp, Error, TEXT("%s: Failed initializing mesh comp[%s]"), *GetName(), *meshUniqueName);
                }
            }
            else
            {
                UE_LOG(LogTemp,
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
        // NOTE: If we are in mid of loading up other mesh comps, changing root mid-way could disrupt the component hiearchy
        if ((nullptr == BaseMeshComp) && (MeshCompList.Num() > 0) && (MeshCompList.Num() == ToBeCreatedMeshesNum))
        {
            BaseMeshComp = MeshCompList[0];

            // Set as Root Component
            // Set the main mesh comp as the root
            // (Not clear why using the default scene component as the root just disrupts actor-children relative movement,
            // and thus also compromise the actor transform itself)!
            if (RootComponent)
            {
                RootComponent->DestroyComponent();
            }
            SetRootComponent(BaseMeshComp);
        }

        return addedMeshCompList;
    }

    virtual void OnBodyComponentMeshCreationDone(bool bInCreationResult, UObject* InMeshBodyComponent);
    void SetCustomDepthEnabled(bool bIsCustomDepthEnabled);
    bool IsCustomDepthEnabled() const;

    FORCEINLINE void SetActivated(bool bInIsActivated)
    {
#if RAPYUTA_SIM_VISUAL_DEBUG
        // Visible/Invisibile
        SetActorHiddenInGame(!bInIsActivated);

        // RenderCustomDepth
        SetCustomDepthEnabled(bInIsActivated);
#endif

        // Then teleport itself to a camera-blind location if being deactivated,
        // so when it get activated back, it would not happen to appear at an unintended pose
        if (false == bInIsActivated)
        {
            AddActorWorldOffset(FVector(0.f, 0.f, -500.f));
            CellIdx = FIntVector::ZeroValue;
        }
    }

protected:
    UPROPERTY(VisibleAnywhere)
    uint8 bLastMeshCreationResult : 1;

    UPROPERTY(VisibleAnywhere)
    uint8 bFullyCreated : 1;
};
