// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRProceduralMeshComponent.h"
#include "Core/RRStaticMeshComponent.h"
#include "Core/RRUObjectUtils.h"
#include "RapyutaSimulationPlugins.h"

#include "RRMeshActor.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRMeshActor : public ARRBaseActor
{
    GENERATED_BODY()
public:
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

    virtual void DeclareFullCreation(bool bInCreationResult);

public:
    void DrawTransform();

    UMeshComponent* GetMeshComponent(int32 Index = 0) const;
    template<typename TMeshComp>
    TArray<TMeshComp*> CreateMeshComponentList(USceneComponent* InParentComp,
                                               const TArray<FString>& InMeshUniqueNameList,
                                               const TArray<FTransform>& InMeshRelTransf = TArray<FTransform>(),
                                               const TArray<FString>& InMeshMaterialNameList = TArray<FString>())
    {
        // (Note) This method could be invoked multiple times
        TArray<TMeshComp*> addedMeshCompList;
        if (InMeshRelTransf.Num() > 0)
        {
            verify(InMeshRelTransf.Num() == InMeshUniqueNameList.Num());
        }
        if (InMeshMaterialNameList.Num() > 0)
        {
            verify(InMeshMaterialNameList.Num() == InMeshUniqueNameList.Num());
        }

        TMeshComp* meshComp = nullptr;
        for (auto i = 0; i < InMeshUniqueNameList.Num(); ++i)
        {
            const FString& meshUniqueName = InMeshUniqueNameList[i];

            // [ProcMeshComp] Verify path as absolute & existing
            if constexpr (TIsSame<TMeshComp, URRProceduralMeshComponent>::Value)
            {
                if ((false == FPaths::IsRelative(meshUniqueName)) && (false == FPaths::FileExists(meshUniqueName)))
                {
                    UE_LOG(LogTemp, Error, TEXT("Mesh invalid [%s] is non-existent"), *meshUniqueName);
                    continue;
                }
            }

            static int64 count = 0;
            // [OBJECT MESH COMP] --
            //
            meshComp = URRUObjectUtils::CreateMeshComponent<TMeshComp>(
                this,
                meshUniqueName,
                FString::Printf(TEXT("%s_MeshComp_%ld"), *ActorInfo->UniqueName, count++),
                InMeshRelTransf.IsValidIndex(i) ? InMeshRelTransf[i] : FTransform::Identity,
                ActorInfo->IsStationary,
                ActorInfo->IsPhysicsEnabled,
                ActorInfo->IsCollisionEnabled,
                InParentComp);

            if (meshComp)
            {
                // [ProcMeshComp] Load mesh from disk path
                if constexpr (TIsSame<TMeshComp, URRProceduralMeshComponent>::Value)
                {
                    // (Note) This must be the full path to the mesh file on disk
                    if (meshComp->InitializeMesh(meshUniqueName))
                    {
                        if (InMeshMaterialNameList.IsValidIndex(i))
                        {
                            URRUObjectUtils::CreateMeshCompMaterialInstance(meshComp, i, InMeshMaterialNameList[i]);
                        }
                        addedMeshCompList.AddUnique(meshComp);
                    }
                    else
                    {
                        UE_LOG(LogTemp, Error, TEXT("%s: Failed initializing Proc mesh comp[%s]"), *GetName(), *meshUniqueName);
                    }
                }
                else
                {
                    if (InMeshMaterialNameList.IsValidIndex(i))
                    {
                        URRUObjectUtils::CreateMeshCompMaterialInstance(meshComp, i, InMeshMaterialNameList[i]);
                    }
                    addedMeshCompList.AddUnique(meshComp);
                    OnBodyComponentMeshCreationDone(true, meshComp);
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
        // Base Mesh Component Configs
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
        // Visible/Invisibile
        SetActorHiddenInGame(!bInIsActivated);

        // RenderCustomDepth
        SetCustomDepthEnabled(bInIsActivated);

        // Then teleport itself to a camera-blind location if being deactivated,
        // so when it get activated back, it would not happen to appear at an unintended pose
        if (false == bInIsActivated)
        {
            AddActorWorldOffset(FVector(0.f, 0.f, -500.f));
        }
    }

protected:
    UPROPERTY(VisibleAnywhere)
    uint8 bLastMeshCreationResult : 1;

    UPROPERTY(VisibleAnywhere)
    uint8 bFullyCreated : 1;
};
