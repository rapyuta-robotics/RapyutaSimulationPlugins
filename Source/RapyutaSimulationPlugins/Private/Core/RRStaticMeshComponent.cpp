// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRStaticMeshComponent.h"

// UE
#include "Materials/MaterialInstanceDynamic.h"

// RapyutaSimulationPlugins
#include "Core/RRGameSingleton.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRMeshActor.h"
#include "Core/RRThreadUtils.h"
#include "Core/RRTypeUtils.h"

URRStaticMeshComponent::URRStaticMeshComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    bTickInEditor = false;
    bUseDefaultCollision = true;

    OnMeshCreationDone.BindUObject(Cast<ARRMeshActor>(GetOwner()), &ARRMeshActor::OnBodyComponentMeshCreationDone);
}

void URRStaticMeshComponent::BeginPlay()
{
    Super::BeginPlay();

    ActorCommon = URRActorCommon::GetActorCommon(SceneInstanceId);
    // [Initialize()] should be invoked right after the mesh comp is created!
    // Please refer to [URRUObjectUtils::CreateMeshComponent()]

    // It is considered that a mesh component should only be physically activated if its Owning Actor is Physically Enabled,
    // thus not auto activating physics here! Rather, it is left to the Owning actor to do it!
}

void URRStaticMeshComponent::Initialize(bool bInIsStationary, bool bInIsPhysicsEnabled)
{
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[%s] STATIC MESH COMP INITIALIZED - Stationary: %d Physics Enabled: %d!"),
           *GetName(),
           bInIsStationary,
           bInIsPhysicsEnabled);
#endif

    bIsStationary = bInIsStationary;

    // (NOTE) Upon creation, DO NOT enable physics here yet, which will makes mesh comp detached from its parent.
    // Physics should only be enabled after children mesh comps have also been created, so they will be welded.
    // Refer to [FBodyInstance::SetInstanceSimulatePhysics()]
    SetSimulatePhysics(bInIsPhysicsEnabled);

    // CustomDepthStencilValue
    SetCustomDepthStencilValue(URRActorCommon::GenerateUniqueDepthStencilValue());
}

void URRStaticMeshComponent::SetMesh(UStaticMesh* InStaticMesh)
{
    verify(IsValid(InStaticMesh));

    const ECollisionEnabled::Type collisionType = BodyInstance.GetCollisionEnabled();
    if ((ECollisionEnabled::PhysicsOnly == collisionType) || (ECollisionEnabled::QueryAndPhysics == collisionType))
    {
        UBodySetup* bodySetup = InStaticMesh->GetBodySetup();
        verify(bodySetup);
        verify(EBodyCollisionResponse::BodyCollision_Enabled == bodySetup->CollisionReponse);
        verify(bodySetup->bCreatedPhysicsMeshes);
        verify(false == bodySetup->bFailedToCreatePhysicsMeshes);
        verify(bodySetup->bHasCookedCollisionData);
    }

    SetStaticMesh(InStaticMesh);
    OnMeshCreationDone.ExecuteIfBound(true, this);
}

FVector URRStaticMeshComponent::GetSize() const
{
    return GetStaticMesh()->GetBoundingBox().GetSize();
}

FVector URRStaticMeshComponent::GetExtent() const
{
    return GetStaticMesh()->GetBoundingBox().GetExtent();
}

FVector URRStaticMeshComponent::GetScaledExtent() const
{
    return GetComponentScale() * GetExtent();
}

void URRStaticMeshComponent::SetCollisionModeAvailable(bool IsOn, bool IsHitEventEnabled)
{
    if (IsOn)
    {
#if 1
        bUseDefaultCollision = true;
#else
        SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
        SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);
        SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        SetNotifyRigidBodyCollision(IsHitEventEnabled);
#endif
    }
    else
    {
        SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);
        SetCollisionEnabled(ECollisionEnabled::NoCollision);
        SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
    }
}

void URRStaticMeshComponent::EnableOverlapping()
{
    SetSimulatePhysics(false);
    SetCollisionProfileName(TEXT("Overlap"));
    SetCollisionEnabled(ECollisionEnabled::QueryOnly);    // SUPER IMPORTANT!
    SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
    SetGenerateOverlapEvents(true);
}

// This function is used proprietarily for Generic Link/Joint (Non-Articulation) structure
void URRStaticMeshComponent::LockSelf()
{
#if 1
    SetConstraintMode(EDOFMode::SixDOF);
#else
    // PHYSX only
    FBodyInstance* bodyInstance = GetBodyInstance();
    bodyInstance->SetDOFLock(EDOFMode::SixDOF);
    bodyInstance->bLockXTranslation = true;
    bodyInstance->bLockYTranslation = true;
    bodyInstance->bLockZTranslation = true;
    bodyInstance->bLockXRotation = true;
    bodyInstance->bLockYRotation = true;
    bodyInstance->bLockZRotation = true;
#endif
}

void URRStaticMeshComponent::HideSelf(bool IsHidden)
{
    SetVisibility(!IsHidden);
    SetHiddenInGame(IsHidden);
}
