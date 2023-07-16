// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRStaticMeshComponent.h"

// UE
#if WITH_EDITOR
#include "ConvexDecompTool.h"
#endif
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshActor.h"
#include "Core/RRMeshUtils.h"
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
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore,
                           Warning,
                           TEXT("STATIC MESH COMP INITIALIZED - Stationary: %d Physics Enabled: %d!"),
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
        if (ensure(bodySetup))
        {
            // NOTE: Only stationary object is supported to have Complex collision
            if (!bIsStationary)
            {
                ensure(bodySetup->CollisionTraceFlag != ECollisionTraceFlag::CTF_UseComplexAsSimple);
            }
            ensure(EBodyCollisionResponse::BodyCollision_Enabled == bodySetup->CollisionReponse);
            if (bUseDefaultSimpleCollision)
            {
                ensure(bodySetup->bCreatedPhysicsMeshes);
                ensure(false == bodySetup->bFailedToCreatePhysicsMeshes);
                ensure(bodySetup->bHasCookedCollisionData);
            }
        }
    }

    // NOTE: [MarkRenderStateDirty() & [RecreatePhysicsState()] already called here-in
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
    MeshUniqueName = bStaticMeshAlreadyExists ? InMeshFileName
                                              : URRUObjectUtils::ComposeDynamicResourceName(
                                                    URRGameSingleton::GetAssetNamePrefix(ERRResourceDataType::UE_STATIC_MESH),
                                                    *FPaths::GetBaseFilename(InMeshFileName));
    ShapeType = URRGameSingleton::GetShapeTypeFromMeshName(InMeshFileName);

#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("- %s"), *InMeshFileName);
#endif

    switch (ShapeType)
    {
        case ERRShapeType::MESH:
            if (bStaticMeshAlreadyExists)
            {
                UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Warning, TEXT("REUSE IN-MEMORY STATIC MESH[%s]"), *MeshUniqueName);
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

                            UE_LOG_WITH_INFO_SHORT(
                                LogRapyutaCore, Warning, TEXT("REUSE IN-MEMORY STATIC MESH[%s]"), *MeshUniqueName);
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
                    ensure(CreateMeshBody(*meshData));
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
                                              // Create mesh body, signalling [OnMeshCreationDone()]
                                              if (ensure(loadedMeshData.IsValid()) && ensure(CreateMeshBody(loadedMeshData)))
                                              {
                                                  // Save [loadedMeshData] to [FRRMeshData::MeshDataStore]
                                                  FRRMeshData::AddMeshData(MeshUniqueName,
                                                                           MakeShared<FRRMeshData>(MoveTemp(loadedMeshData)));
                                              }
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

UStaticMesh* URRStaticMeshComponent::CreateMesh(const FRRMeshData& InMeshData, bool bInAsVisualMesh)
{
    // (NOTE) This function could be invoked from an async task running in GameThread
    if (false == InMeshData.IsValid())
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("STATIC MESH DATA BUFFER [%s] IS INVALID"), *MeshUniqueName);
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
    meshDescParams.bUseHashAsGuid = true;
    // NOTE: bFastBuild must be true in packaged Sim, in Editor it is supported for both true & false
    meshDescParams.bFastBuild = !WITH_EDITOR;
    meshDescParams.bBuildSimpleCollision = false;
#if RAPYUTA_SIM_DEBUG
    // If this function runs in thread, do not mark the package dirty since MarkPackageDirty is not thread safe
    // This also requires MarkPackageDirty() to be called later in game thread
    meshDescParams.bMarkPackageDirty = false;
#endif
    // NOTE: Commit might be slow but needed since we want the mesh to be saved out to disk for reuse
    meshDescParams.bCommitMeshDescription = true;

    // Only allow CPU access in Game & for complex collision, which might be expensive to be sent to GPU
    meshDescParams.bAllowCpuAccess = !WITH_EDITOR && bUseComplexCollision;

    UStaticMesh* staticMesh = NewObject<UStaticMesh>(
        this, bInAsVisualMesh ? FName(*MeshUniqueName) : FName(*FString::Printf(TEXT("UCX_%s"), *MeshUniqueName)));
    if (bInAsVisualMesh)
    {
        staticMesh->SetLightMapCoordinateIndex(0);
#if WITH_EDITOR
        // Ref: FStaticMeshFactoryImpl::SetupMeshBuildSettings()
        // LOD
        ITargetPlatform* currentPlatform = GetTargetPlatformManagerRef().GetRunningTargetPlatform();
        check(currentPlatform);
        const FStaticMeshLODGroup& lodGroup = currentPlatform->GetStaticMeshLODSettings().GetLODGroup(NAME_None);
        int32 lodsNum = lodGroup.GetDefaultNumLODs();
        if (lodsNum == 0)
        {
            lodsNum = 1;
        }
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
    }

    // Build mesh
    staticMesh->BuildFromMeshDescriptions({&meshDesc}, meshDescParams);
    return staticMesh;
}

UStaticMesh* URRStaticMeshComponent::CreateMeshBody(const FRRMeshData& InMeshData)
{
    UStaticMesh* visualMesh = CreateMesh(InMeshData, true);
    if (nullptr == visualMesh)
    {
        return nullptr;
    }

    // Generate complex in case of no simple collision
    if (false == bUseDefaultSimpleCollision)
    {
        auto* bodySetup = visualMesh->GetBodySetup();
        if (bUseComplexCollision)
        {
#if WITH_EDITOR
            visualMesh->ComplexCollisionMesh = CreateMesh(InMeshData, false);
#endif
            bodySetup->CollisionTraceFlag = ECollisionTraceFlag::CTF_UseComplexAsSimple;
        }
        else
        {
            // Ref: UProceduralMeshComponent::UpdateCollision()
            GenerateCustomSimpleCollision(InMeshData, bodySetup);
            bodySetup->BodySetupGuid = FGuid::NewGuid();
            bodySetup->CollisionTraceFlag = ECollisionTraceFlag::CTF_UseSimpleAsComplex;
            bodySetup->bHasCookedCollisionData = true;
            bodySetup->bMeshCollideAll = true;
            bodySetup->InvalidatePhysicsData();
            bodySetup->CreatePhysicsMeshes();
#if RAPYUTA_SIM_DEBUG
            // This rebuilds only renderable data thus may not be required, also possibly causing crash -> kept purely for debug reference
            visualMesh->Build(true);
            visualMesh->PostLoad();
#endif
        }
    }
    // NOTE: visualMesh's RenderData, upon [FBuildMeshDescriptionsParams::bFastBuild == false] will be defer-created in [FinishPostLoadInternal()->InitResources()],
    // thus [visualMeshRenderData->IsInitialized()] may not instantly inited here like in fast build.

    // Add to the global resource store
    URRGameSingleton::Get()->AddDynamicResource<UStaticMesh>(ERRResourceDataType::UE_STATIC_MESH, visualMesh, MeshUniqueName);

    // Auto-save [visualMesh] to uasset on disk, to be used directly in future Sim runs
#if RAPYUTA_SIM_VERBOSE
    UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] Source models num %d"), *visualMesh->GetName(), visualMesh->GetNumSourceModels());
#endif
#if WITH_EDITOR
    if (visualMesh->IsSourceModelValid(0))
    {
        if (visualMesh->GetSourceModel(0).IsMeshDescriptionValid())
        {
            URRAssetUtils::SaveObjectToAssetInModule(
                visualMesh, ERRResourceDataType::UE_STATIC_MESH, MeshUniqueName, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME);
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("[%s] static mesh: Source model[0] has invalid mesh description"),
                   *visualMesh->GetName());
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("[%s] static mesh has invalid Source model"), *visualMesh->GetName());
    }
#endif

    // This also signals [OnMeshCreationDone] async
    SetMesh(visualMesh);
    return visualMesh;
}

void URRStaticMeshComponent::CreateMeshSection(const TArray<FRRMeshNodeData>& InMeshSectionData,
                                               FMeshDescriptionBuilder& OutMeshDescBuilder)
{
#if RAPYUTA_SIM_VERBOSE
    uint32 meshSectionIndex = 0;
#endif
    for (auto& mesh : InMeshSectionData)
    {
        if (mesh.TriangleIndices.Num() == 0)
        {
            continue;
        }

#if RAPYUTA_SIM_VERBOSE
        UE_LOG_WITH_INFO_NAMED(
            LogRapyutaCore,
            Warning,
            TEXT("CREATE STATIC MESH SECTION[%u]: Vertices(%u) - VertexColors(%u) - TriangleIndices(%u) - Normals(%u) - "
                 "UVs(%u) - "
                 "ProcTangents(%u) - "
                 "Material(%u)"),
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

void URRStaticMeshComponent::GenerateCustomSimpleCollision(const FRRMeshData& InMeshData, UBodySetup* OutBodySetup)
{
    TArray<FVector3f> verts;
    TArray<uint32> indices;
    for (auto& meshNode : InMeshData.Nodes)
    {
        for (auto& mesh : meshNode.Meshes)
        {
            for (const auto& vertex : mesh.Vertices)
            {
                verts.Add(FVector3f(vertex));
            }
            for (const auto& triangleIdx : mesh.TriangleIndices)
            {
                indices.Add(triangleIdx);
            }
        }
    }
#if WITH_EDITOR
    DecomposeMeshToHulls(OutBodySetup, verts, indices, 64, 100000);
#endif
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
