// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRMeshActor.h"

// Native
#include <type_traits>

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameState.h"
#include "Core/RRMathUtils.h"
#include "Core/RRSceneDirector.h"
#include "Core/RRStaticMeshComponent.h"
#include "Core/RRUObjectUtils.h"

using URRMeshComponent = typename std::conditional<RAPYUTA_RUNTIME_MESH_ENTITY_USE_STATIC_MESH,
    URRStaticMeshComponent,
    URRProceduralMeshComponent>::type;

ARRMeshActor::ARRMeshActor()
{
    // CREATE & SETUP A SCENE COMPONENT AS ROOT
    // -> THIS IS REQUIRED TO ALLOW ACTOR BEING SPAWNED WITH A USER TRANSFORM, WHICH IS APPLIED TO THE SCENE-COMPONENT ROOT
    // Refer to [AActor::PostSpawnInitialize()]
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
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Display, TEXT("[MeshCompList] has not been created!"));
        }

        if (meshCompNum)
        {
            UE_LOG_WITH_INFO_NAMED(
                LogRapyutaCore, Fatal, TEXT("Actor has mesh info [%d] but [MeshCompList] has not been created!"), meshCompNum);
            return false;
        }
    }
    else if (nullptr == BaseMeshComp)
    {
        UE_LOG_WITH_INFO_NAMED(
            LogRapyutaCore, Fatal, TEXT("[MeshCompList] [%d] was created but [BaseMeshComp] is NULL!"), meshCompNum);
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

        // [CustomDepthStencilValue]
        // Since deactivated actors do not appear in scene so their custom depth stencil values should be reused
        if (bIsCustomDepthEnabled && IsDataSynthEntity())
        {
#if RAPYUTA_USE_SCENE_DIRECTOR
            auto* sceneDirector = RRGameState->GetSceneInstance<URRSceneInstance>(SceneInstanceId)->SceneDirector;
            meshComp->SetCustomDepthStencilValue((sceneDirector->SceneEntityMaskValueList.Num() > 0)
                                                     ? sceneDirector->SceneEntityMaskValueList.Pop()
                                                     : ActorCommon->GenerateUniqueDepthStencilValue());
#endif
        }
        else
        {
            meshComp->SetCustomDepthStencilValue(URRActorCommon::DEFAULT_CUSTOM_DEPTH_STENCIL_VALUE_VOID);
        }
    }
}

void ARRMeshActor::SetCustomDepthStencilValue(int32 InCustomDepthStencilValue)
{
    bool bCustomDepthEnabled = (InCustomDepthStencilValue >= 0);
    for (auto& meshComp : MeshCompList)
    {
        // [RenderCustomDepth]
        meshComp->SetRenderCustomDepth(bCustomDepthEnabled);

        // [CustomDepthStencilValue]
        if (bCustomDepthEnabled && IsDataSynthEntity())
        {
            auto* sceneDirector = RRGameState->GetSceneInstance<URRSceneInstance>(SceneInstanceId)->SceneDirector;
            meshComp->SetCustomDepthStencilValue(InCustomDepthStencilValue);
        }
        else
        {
            meshComp->SetCustomDepthStencilValue(URRActorCommon::DEFAULT_CUSTOM_DEPTH_STENCIL_VALUE_VOID);
        }
    }
}

bool ARRMeshActor::IsCustomDepthEnabled() const
{
    for (const auto& meshComp : MeshCompList)
    {
        if ((false == meshComp->bRenderCustomDepth) ||
            (meshComp->CustomDepthStencilValue <= URRActorCommon::DEFAULT_CUSTOM_DEPTH_STENCIL_VALUE_VOID))
        {
            return false;
        }
    }
    return true;
}

TArray<int32> ARRMeshActor::GetCustomDepthStencilValueList() const
{
    TArray<int32> customDepthStencilValueList;
    for (const auto& meshComp : MeshCompList)
    {
        customDepthStencilValueList.AddUnique(BaseMeshComp->CustomDepthStencilValue);
    }

    return customDepthStencilValueList;
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
#if RAPYUTA_SIM_VERBOSE
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("[%s] MESH ACTOR CREATED!"));
#endif
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("MESH ACTOR CREATION FAILED!"));
    }

    // (NOTE) Since each of [MeshCompList] also created default material instance once mesh section is created,
    // [ActorInfo]'s Override materials could only be set here once the full creation is done.
    if (ActorInfo.IsValid())
    {
        for (auto& meshComp : MeshCompList)
        {
            if (ActorInfo->MaterialNameList.Num() == 0)
            {
                continue;
            }
            for (auto j = 0; j < meshComp->GetMaterials().Num(); ++j)
            {
                const auto& matName =
                    ActorInfo->MaterialNameList.IsValidIndex(j) ? ActorInfo->MaterialNameList[j] : ActorInfo->MaterialNameList[0];
                URRUObjectUtils::CreateMeshCompMaterialInstance(meshComp, j, matName);
            }
        }
    }

    // SIGNAL [MESH ACTOR]
    ActorCommon->OnMeshActorFullyCreated.ExecuteIfBound(bInCreationResult, this);

#if RAPYUTA_SIM_VISUAL_DEBUG
    URRUObjectUtils::DrawActorBoundingBox(this);
#endif
}
