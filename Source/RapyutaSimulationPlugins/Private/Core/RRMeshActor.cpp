// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRMeshActor.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameState.h"
#include "Core/RRMathUtils.h"
#include "Core/RRStaticMeshComponent.h"
#include "Core/RRUObjectUtils.h"

using URRMeshComponent =
    typename TChooseClass<RAPYUTA_DATA_SYNTH_USE_ENTITY_STATIC_MESH, URRStaticMeshComponent, URRProceduralMeshComponent>::Result;

ARRMeshActor::ARRMeshActor()
{
    // CREATE & SETUP A SCENE COMPONENT AS ROOT
    // -> THIS IS REQUIRED TO ALLOW ACTOR BEING SPAWNED WITH A USER TRANSFORM, WHICH IS APPLIED TO THE SCENE-COMPONENT ROOT
    URRUObjectUtils::SetupDefaultRootComponent(this);

    bLastMeshCreationResult = false;
    bFullyCreated = false;
}

bool ARRMeshActor::Initialize()
{
    if (false == Super::Initialize())
    {
        return false;
    }

    // 1- Create child mesh components
    if (ActorInfo.IsValid())
    {
        ToBeCreatedMeshesNum = ActorInfo->MeshUniqueNameList.Num();
        CreateMeshComponentList<URRMeshComponent>(
            GetRootComponent(), ActorInfo->MeshUniqueNameList, ActorInfo->MeshRelTransformList, ActorInfo->MaterialNameList);
    }

    // 2- By default, a mesh actor has its CustomDepthRender enabled, which is cheap, for segmask capturing.
    // This requires MeshCompList having been created
    SetCustomDepthEnabled(true);

    // 3- This will take effect on all child mesh components
    GetRootComponent()->SetMobility((ActorInfo.IsValid() && ActorInfo->bIsStationary) ? EComponentMobility::Stationary
                                                                                      : EComponentMobility::Movable);
    return true;
}

bool ARRMeshActor::HasInitialized(bool bIsLogged) const
{
    if (!Super::HasInitialized(bIsLogged))
    {
        return false;
    }

    // [Mesh Comp List]
    auto meshCompNum = MeshCompList.Num();
    if (0 == meshCompNum)
    {
        if (bIsLogged)
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("[%s] [MeshCompList] has not been created!"), *GetName());
        }

        if (meshCompNum)
        {
            UE_LOG(LogRapyutaCore,
                   Fatal,
                   TEXT("[%s] Actor has mesh info [%d] but [MeshCompList] has not been created!"),
                   *GetName(),
                   meshCompNum);
            return false;
        }
    }
    else if (nullptr == BaseMeshComp)
    {
        UE_LOG(LogRapyutaCore,
               Fatal,
               TEXT("[%s] [MeshCompList] [%d] was created but [BaseMeshComp] is NULL!"),
               *GetName(),
               meshCompNum);
        return false;
    }

    return true;
}

void ARRMeshActor::Reset()
{
    Super::Reset();
    MeshCompList.Reset();
    CreatedMeshesNum = 0;
}

UMeshComponent* ARRMeshActor::GetMeshComponent(int32 Index) const
{
    return MeshCompList.IsValidIndex(Index) ? MeshCompList[Index] : nullptr;
}

void ARRMeshActor::SetCustomDepthEnabled(bool bIsCustomDepthEnabled)
{
    for (auto& meshComp : MeshCompList)
    {
        // [RenderCustomDepth]
        meshComp->SetRenderCustomDepth(bIsCustomDepthEnabled);
    }
}
bool ARRMeshActor::IsCustomDepthEnabled() const
{
    for (const auto& meshComp : MeshCompList)
    {
        if ((false == meshComp->bRenderCustomDepth) ||
            (meshComp->CustomDepthStencilValue <= URRStaticMeshComponent::CUSTOM_DEPTH_STENCIL_VOID))
        {
            return false;
        }
    }
    return true;
}

void ARRMeshActor::OnBodyComponentMeshCreationDone(bool bInCreationResult, UObject* InMeshBodyComponent)
{
    // Accumulatively result marking
    bLastMeshCreationResult = (0 == CreatedMeshesNum) ? bInCreationResult : (bLastMeshCreationResult && bInCreationResult);
    if (ToBeCreatedMeshesNum == (++CreatedMeshesNum))
    {
        DeclareFullCreation(bLastMeshCreationResult);
    }
}

void ARRMeshActor::DeclareFullCreation(bool bInCreationResult)
{
    bFullyCreated = bInCreationResult;
    if (bInCreationResult)
    {
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] MESH ACTOR CREATED!"), *GetName());
#endif
    }
    else
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("[%s] MESH ACTOR CREATION FAILED!"), *GetName());
    }

    // (NOTE) Since [ProcMeshComp] also created default material instance once mesh section is created,
    // [ActorInfo]'s Override materials could only be set here once the full creation is done.
    if (ActorInfo.IsValid())
    {
        for (auto i = 0; i < ActorInfo->MeshUniqueNameList.Num(); ++i)
        {
            if (ActorInfo->MaterialNameList.IsValidIndex(i))
            {
                URRUObjectUtils::CreateMeshCompMaterialInstance(MeshCompList[i], 0, ActorInfo->MaterialNameList[i]);
            }
        }
    }

    // SIGNAL [MESH ACTOR]
    ActorCommon->OnMeshActorFullyCreated.ExecuteIfBound(bInCreationResult, this);

#if RAPYUTA_SIM_VISUAL_DEBUG
    URRUObjectUtils::DrawActorBoundingBox(this);
#endif
}
