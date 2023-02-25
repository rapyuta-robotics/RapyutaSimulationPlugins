// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRStaticMeshComponent.h"

// UE
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"

// RapyutaSimulationPlugins
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshUtils.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
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

    // CustomDepthStencil will be actively set by stakeholders or ARRMeshActor if needs be
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

    // Signal [[OnMeshCreationDone]] async
    // Specifically, the signal is used to trigger ARRMeshActor::DeclareFullCreation()], which requires its MeshCompList
    // to be fullfilled in advance!
    AsyncTask(ENamedThreads::GameThread, [this]() { OnMeshCreationDone.ExecuteIfBound(true, this); });
}

bool URRStaticMeshComponent::InitializeMesh(const FString& InMeshFileName)
{
    URRGameSingleton* gameSingleton = URRGameSingleton::Get();
    UStaticMesh* staticMesh = gameSingleton->GetStaticMesh(InMeshFileName, false);
    const bool bStaticMeshAlreadyExists = (nullptr != staticMesh);
    MeshUniqueName = bStaticMeshAlreadyExists
                         ? InMeshFileName
                         : URRUObjectUtils::ComposeDynamicResourceName(TEXT("SM"), *FPaths::GetBaseFilename(InMeshFileName));
    ShapeType = URRGameSingleton::GetShapeTypeFromMeshName(InMeshFileName);

#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore, Warning, TEXT("URRStaticMeshComponent::InitializeMesh: %s - %s"), *GetName(), *InMeshFileName);
#endif

    switch (ShapeType)
    {
        case ERRShapeType::MESH:
            if (bStaticMeshAlreadyExists)
            {
                SetMesh(staticMesh);
            }
            else if (gameSingleton->HasSimResource(ERRResourceDataType::UE_STATIC_MESH, MeshUniqueName))
            {
                // Wait for StaticMesh[MeshUniqueName] has been fully loaded
                URRCoreUtils::RegisterRepeatedExecution(
                    GetWorld(),
                    StaticMeshTimerHandle,
                    [this, gameSingleton]()
                    {
                        UStaticMesh* existentStaticMesh = gameSingleton->GetStaticMesh(MeshUniqueName, false);
                        if (existentStaticMesh)
                        {
                            // Stop the repeat timer first, note that this will invalidate all the captured variants except this
                            URRCoreUtils::StopRegisteredExecution(GetWorld(), StaticMeshTimerHandle);
                            SetMesh(existentStaticMesh);
                        }
                    },
                    0.01f);
            }
            else
            {
                // (NOTE) Temporary create an empty place-holder with [MeshUniqueName],
                // so other StaticMeshComps, wanting to reuse the same [MeshUniqueName], could check & wait for its creation
                gameSingleton->AddDynamicResource<UStaticMesh>(ERRResourceDataType::UE_STATIC_MESH, nullptr, MeshUniqueName);

                TSharedPtr<FRRMeshData> meshData = FRRMeshData::GetMeshData(MeshUniqueName);
                if (meshData.IsValid())
                {
                    verify(CreateMeshBody(*meshData));
                }
                else
                {
                    // Start async mesh loading
                    Async(
#if WITH_EDITOR
                        EAsyncExecution::LargeThreadPool,
#else
                        EAsyncExecution::ThreadPool,
#endif
                        [this, InMeshFileName]()
                        {
                            FRRMeshData runtimeMeshData;
                            TSharedPtr<Assimp::Importer> meshImporter = MakeShared<Assimp::Importer>();
                            runtimeMeshData = URRMeshUtils::LoadMeshFromFile(InMeshFileName, *meshImporter);
                            runtimeMeshData.MeshImporter = meshImporter;
                            runtimeMeshData.MeshUniqueName = MeshUniqueName;
                            if (runtimeMeshData.IsValid())
                            {
                                AsyncTask(ENamedThreads::GameThread,
                                          [this, loadedMeshData = MoveTemp(runtimeMeshData)]() mutable
                                          {
                                              verify(loadedMeshData.IsValid());
                                              // Create mesh body, signalling [OnMeshCreationDone()]
                                              verify(CreateMeshBody(loadedMeshData));
                                              // Save [loadedMeshData] to [FRRMeshData::MeshDataStore]
                                              FRRMeshData::AddMeshData(MeshUniqueName,
                                                                       MakeShared<FRRMeshData>(MoveTemp(loadedMeshData)));
                                          });
                            }
                        });
                }
            }
            break;

        case ERRShapeType::PLANE:
        case ERRShapeType::CYLINDER:
        case ERRShapeType::BOX:
        case ERRShapeType::SPHERE:
        case ERRShapeType::CAPSULE:
            // (NOTE) Due to primitive mesh ranging in various size, its data that is also insignificant is not cached by
            // [FRRMeshData::AddMeshData]
            SetMesh(URRGameSingleton::Get()->GetStaticMesh(MeshUniqueName));
            break;
    }
    return true;
}

UStaticMesh* URRStaticMeshComponent::CreateMeshBody(const FRRMeshData& InMeshData)
{
    // (NOTE) This function could be invoked from an async task running in GameThread
    if (false == InMeshData.IsValid())
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[%s] CreateMeshBody() STATIC MESH DATA BUFFER [%s] IS INVALID"),
               *GetName(),
               *MeshUniqueName);
        return nullptr;
    }

    // Static mesh
    // Mesh description will hold all the geometry, uv, normals going into the static mesh
    FMeshDescription meshDesc;
    FStaticMeshAttributes attributes(meshDesc);
    attributes.Register();

    FMeshDescriptionBuilder meshDescBuilder;
    meshDescBuilder.SetMeshDescription(&meshDesc);
    meshDescBuilder.EnablePolyGroups();
    meshDescBuilder.SetNumUVLayers(1);

    for (const auto& node : InMeshData.Nodes)
    {
        CreateMeshSection(node.Meshes, meshDescBuilder);
    }

    // Build static mesh
    UStaticMesh::FBuildMeshDescriptionsParams meshDescParams;
    // NOTE: bFastBuild off is only supported for WITH_EDITOR
    meshDescParams.bFastBuild = true;
    meshDescParams.bBuildSimpleCollision = true;
    UStaticMesh* staticMesh = NewObject<UStaticMesh>(this, FName(*MeshUniqueName));
    staticMesh->SetLightMapCoordinateIndex(0);
#if WITH_EDITOR
    // Ref: FStaticMeshFactoryImpl::SetupMeshBuildSettings()
    // LOD
    ITargetPlatform* currentPlatform = GetTargetPlatformManagerRef().GetRunningTargetPlatform();
    check(currentPlatform);
    const FStaticMeshLODGroup& lodGroup = currentPlatform->GetStaticMeshLODSettings().GetLODGroup(NAME_None);
    int32 lodsNum = lodGroup.GetDefaultNumLODs();
    while (staticMesh->GetNumSourceModels() < lodsNum)
    {
        staticMesh->AddSourceModel();
    }
    for (auto lodIndex = 0; lodIndex < lodsNum; ++lodIndex)
    {
        auto& sourceModel = staticMesh->GetSourceModel(lodIndex);
        sourceModel.ReductionSettings = lodGroup.GetDefaultSettings(lodIndex);
        sourceModel.BuildSettings.bGenerateLightmapUVs = true;
        sourceModel.BuildSettings.SrcLightmapIndex = 0;
        sourceModel.BuildSettings.DstLightmapIndex = 0;
        sourceModel.BuildSettings.bRecomputeNormals = false;
        sourceModel.BuildSettings.bRecomputeTangents = false;

        // LOD Section
        auto& sectionInfoMap = staticMesh->GetSectionInfoMap();
        for (auto meshSectionIndex = 0; meshSectionIndex < InMeshData.Nodes.Num(); ++meshSectionIndex)
        {
            FMeshSectionInfo info = sectionInfoMap.Get(lodIndex, meshSectionIndex);
            info.MaterialIndex = 0;
            sectionInfoMap.Remove(lodIndex, meshSectionIndex);
            sectionInfoMap.Set(lodIndex, meshSectionIndex, info);
        }
    }
    staticMesh->SetLightMapResolution(lodGroup.GetDefaultLightMapResolution());
#endif

    // Mesh's static materials
    for (const auto& materialInstance : InMeshData.MaterialInstances)
    {
        staticMesh->AddMaterial(materialInstance->GetBaseMaterial());
    }
    staticMesh->BuildFromMeshDescriptions({&meshDesc}, meshDescParams);
    // staticMesh->PostLoad();

    // Add to the global resource store
    URRGameSingleton::Get()->AddDynamicResource<UStaticMesh>(ERRResourceDataType::UE_STATIC_MESH, staticMesh, MeshUniqueName);

    // This also signals [OnMeshCreationDone] async
    SetMesh(staticMesh);
    return staticMesh;
}

void URRStaticMeshComponent::CreateMeshSection(const TArray<FRRMeshNodeData>& InMeshSectionData,
                                               FMeshDescriptionBuilder& OutMeshDescBuilder)
{
#if RAPYUTA_SIM_DEBUG
    uint32 meshSectionIndex = 0;
#endif
    for (auto& mesh : InMeshSectionData)
    {
        if (mesh.TriangleIndices.Num() == 0)
        {
            continue;
        }

#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s]CREATE STATIC MESH SECTION[%u]: Vertices(%u) - VertexColors(%u) - TriangleIndices(%u) - Normals(%u) - "
                    "UVs(%u) - "
                    "ProcTangents(%u) - "
                    "Material(%u)"),
               *GetName(),
               meshSectionIndex,
               mesh.Vertices.Num(),
               mesh.VertexColors.Num(),
               mesh.TriangleIndices.Num(),
               mesh.Normals.Num(),
               mesh.UVs.Num(),
               mesh.ProcTangents.Num(),
               mesh.MaterialIndex);
#endif

        // Create vertex instances (3 per face)
        TArray<FVertexID> vertexIDs;
        for (auto i = 0; i < mesh.Vertices.Num(); ++i)
        {
            vertexIDs.Emplace(OutMeshDescBuilder.AppendVertex(mesh.Vertices[i]));
        }

        // Vertex instances
        TArray<FVertexInstanceID> vertexInsts;
        for (auto i = 0; i < mesh.TriangleIndices.Num(); ++i)
        {
            // Face(towards -X) vertex instance
            const auto vIdx = mesh.TriangleIndices[i];
            const FVertexInstanceID instanceID = OutMeshDescBuilder.AppendInstance(vertexIDs[vIdx]);
            OutMeshDescBuilder.SetInstanceNormal(instanceID, mesh.Normals[vIdx]);
            OutMeshDescBuilder.SetInstanceUV(instanceID, mesh.UVs[vIdx], 0);
            OutMeshDescBuilder.SetInstanceColor(instanceID, FVector4f(FLinearColor(mesh.VertexColors[vIdx])));
            vertexInsts.Emplace(instanceID);
        }

        // Polygon group & Triangles
        FPolygonGroupID polygonGroupID = OutMeshDescBuilder.AppendPolygonGroup();
        for (auto i = 0; i < vertexInsts.Num() / 3; ++i)
        {
            const auto j = 3 * i;
            OutMeshDescBuilder.AppendTriangle(vertexInsts[j], vertexInsts[j + 1], vertexInsts[j + 2], polygonGroupID);
        }

#if RAPYUTA_SIM_DEBUG
        meshSectionIndex++;
#endif
    }
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

void URRStaticMeshComponent::SetCollisionModeAvailable(bool bInCollisionEnabled, bool bInHitEventEnabled)
{
    if (bInCollisionEnabled)
    {
#if 1
        bUseDefaultCollision = true;
#else
        SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
        SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);
        SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        SetNotifyRigidBodyCollision(bInHitEventEnabled);
#endif
    }
    else
    {
        SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);
        SetCollisionEnabled(ECollisionEnabled::NoCollision);
        SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
    }
}

void URRStaticMeshComponent::EnableOverlapping(bool bOverlapEventEnabled)
{
    SetSimulatePhysics(false);
    SetCollisionProfileName(TEXT("Overlap"));
    SetCollisionEnabled(ECollisionEnabled::QueryOnly);    // SUPER IMPORTANT!
    SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
    SetGenerateOverlapEvents(bOverlapEventEnabled);
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

void URRStaticMeshComponent::HideSelf(bool bInHidden)
{
    SetVisibility(!bInHidden);
    SetHiddenInGame(bInHidden);
}
