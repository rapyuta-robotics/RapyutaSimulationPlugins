// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRMeshActor.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRMathUtils.h"
#include "Core/RRStaticMeshComponent.h"
#include "Core/RRUObjectUtils.h"

ARRMeshActor::ARRMeshActor()
{
    // CREATE & SETUP A SCENE COMPONENT AS ROOT
    // -> THIS IS REQUIRED TO ALLOW ACTOR BEING SPAWNED WITH A USER TRANSFORM, WHICH IS APPLIED TO THE SCENE-COMPONENT ROOT
    URRUObjectUtils::SetupDefaultRootComponent(this);
}

bool ARRMeshActor::Initialize()
{
    if (false == Super::Initialize())
    {
        return false;
    }

    // 1- Create child mesh comnponents
    if (ActorInfo.IsValid())
    {
        CreateMeshComponentList(GetRootComponent(), ActorInfo->MeshUniqueNameList);
    }

    // 2- By default, a block object has its CustomDepthRender enabled, which is cheap, for segmask capturing.
    // This requires MeshCompList having been created
    SetCustomDepthEnabled(true);

    // 3- This will take effect on all child mesh components
    GetRootComponent()->SetMobility((ActorInfo.IsValid() && ActorInfo->IsStationary) ? EComponentMobility::Stationary
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
}

TArray<URRStaticMeshComponent*> ARRMeshActor::CreateMeshComponentList(USceneComponent* InParentComp,
                                                                      const TArray<FString>& InMeshUniqueNameList,
                                                                      const TArray<FTransform>& InMeshRelTransf)
{
    // (Note) This method could be invoked multiple times
    TArray<URRStaticMeshComponent*> addedMeshCompList;
    if (InMeshRelTransf.Num() > 0)
    {
        verify(InMeshUniqueNameList.Num() == InMeshRelTransf.Num());
    }

    URRStaticMeshComponent* meshComp = nullptr;
    for (auto i = 0; i < InMeshUniqueNameList.Num(); ++i)
    {
        const FString& meshUniqueName = InMeshUniqueNameList[i];
        static int64 count = 0;
        // [OBJECT MESH COMP] --
        //
        meshComp = URRUObjectUtils::CreateMeshComponent<URRStaticMeshComponent>(
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
            addedMeshCompList.AddUnique(meshComp);
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("[%s:%d] - Failed creating child Mesh Component [%s]!"),
                   *ActorInfo->UniqueName,
                   this,
                   *meshUniqueName);
        }
    }
    MeshCompList.Append(addedMeshCompList);

#if 0    // To be confirmed
    // Base Mesh Component Configs
    if (MeshCompList.Num() > 0)
    {
        BaseMeshComp = MeshCompList[0];

        // Set as Root Component
        // Set the main mesh comp as the root
        // (Not clear why using the default scene component as the root just disrupts actor-children relative movement,
        // and thus also compromise the actor transform itself)!
        if (MeshCompList.Num() == 1)
        {
            BaseMeshComp->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
            SetRootComponent(BaseMeshComp);
        }
    }
#endif
    return addedMeshCompList;
}

URRStaticMeshComponent* ARRMeshActor::GetMeshComponent(int32 Index) const
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
