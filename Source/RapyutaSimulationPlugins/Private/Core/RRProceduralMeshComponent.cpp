// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRProceduralMeshComponent.h"

// Assimp
#include "assimp/Importer.hpp"

// UE
#include "Async/Async.h"
#include "DrawDebugHelpers.h"
#include "KismetProceduralMeshLibrary.h"
#include "RenderUtils.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshActor.h"
#include "Core/RRMeshData.h"
#include "Core/RRMeshUtils.h"
#include "Core/RRUObjectUtils.h"

URRProceduralMeshComponent::URRProceduralMeshComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    // The collision cooking is critical for sweeping movement to work after spawning Proc mesh actor.
    // Due to [FinishPhysicsAsyncCook] being not virtual and private, it is unable to catch [FOnAsyncPhysicsCookFinished] event
    // Thus we could only rely on [UBodySetup::bCreatedPhysicsMeshes]
    bUseAsyncCooking = true;
    bUseComplexAsSimpleCollision = false;
    bCanEverAffectNavigation = true;

    OnMeshCreationDone.BindUObject(Cast<ARRMeshActor>(GetOwner()), &ARRMeshActor::OnBodyComponentMeshCreationDone);
}

void URRProceduralMeshComponent::Initialize(bool bIsStaticBody, bool bInIsPhysicsEnabled)
{
    // CustomDepthStencilValue
    ARRMeshActor* ownerActor = CastChecked<ARRMeshActor>(GetOwner());
    if (ownerActor->GameMode->IsDataSynthSimType() && ownerActor->IsDataSynthEntity())
    {
        verify(IsValid(ownerActor->ActorCommon));
        SetCustomDepthStencilValue(ownerActor->ActorCommon->GenerateUniqueDepthStencilValue());
    }
}

bool URRProceduralMeshComponent::InitializeMesh(const FString& InMeshFileName)
{
    MeshUniqueName = FPaths::GetBaseFilename(InMeshFileName);
    ShapeType = URRGameSingleton::GetShapeTypeFromMeshName(InMeshFileName);

    switch (ShapeType)
    {
        case ERRShapeType::MESH:
        {
            const bool bIsMeshAlreadyLoaded = FRRMeshData::IsMeshDataAvailable(MeshUniqueName);
#if RAPYUTA_SIM_DEBUG
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("URRProceduralMeshComponent::InitializeMesh: %s - %s - Already loaded %d"),
                   *GetName(),
                   *InMeshFileName,
                   bIsMeshAlreadyLoaded);
#endif
            if (bIsMeshAlreadyLoaded)
            {
                CreateMeshBody();
            }
            else
            {
                URRThreadUtils::DoAsyncTaskInThread<void>(
                    [this, InMeshFileName]()
                    {
                        if (!MeshDataBuffer.MeshImporter)
                        {
                            MeshDataBuffer.MeshImporter = MakeShared<Assimp::Importer>();
                        }
                        MeshDataBuffer = URRMeshUtils::LoadMeshFromFile(InMeshFileName, *MeshDataBuffer.MeshImporter);
                    },
                    [this]()
                    {
                        URRThreadUtils::DoTaskInGameThread(
                            [this]()
                            {
                                // Save [MeshDataBuffer] to [FRRMeshData::MeshDataStore]
                                verify(MeshDataBuffer.IsValid());
                                FRRMeshData::AddMeshData(MeshUniqueName, MakeShared<FRRMeshData>(MoveTemp(MeshDataBuffer)));

                                // Then create mesh body, signalling [OnMeshCreationDone()],
                                // which might reference [FRRMeshData::MeshDataStore]
                                CreateMeshBody();
                            });
                    });
            }
        }
        break;

        case ERRShapeType::PLANE:
        case ERRShapeType::CYLINDER:
        case ERRShapeType::BOX:
        case ERRShapeType::SPHERE:
        case ERRShapeType::CAPSULE:
            // Let the primitive-shape mesh be created on the fly in SetMeshSize()
            // NOTE: Due to primitive mesh ranging in various size, its data that is also insignificant is not cached by
            // [FRRMeshData::AddMeshData]
            // SIGNAL [Mesh Created]
            OnMeshCreationDone.ExecuteIfBound(true, this);
            break;
    }
    return true;
}

bool URRProceduralMeshComponent::CreateMeshBody()
{
    // (NOTE) This function is hooked up from an async task running in GameThread
    const TSharedPtr<FRRMeshData> loadedMeshData = FRRMeshData::GetMeshData(MeshUniqueName);
    const FRRMeshData& bodyMeshData = loadedMeshData.IsValid() ? *loadedMeshData : MeshDataBuffer;

    if (false == bodyMeshData.IsValid())
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[%s] CreateMeshBody() PROC MESH DATA BUFFER [%s] IS INVALID"),
               *GetName(),
               *MeshUniqueName);
        return false;
    }

    // VISUAL MESH DATA --
    for (const auto& node : bodyMeshData.Nodes)
    {
        // (NOTE) This also invoke UpdateCollision() but without collision info yet
        CreateMeshSection(node.Meshes);
    }
    for (auto matIndex = 0; matIndex < bodyMeshData.MaterialInstances.Num(); ++matIndex)
    {
        SetMaterial(matIndex, bodyMeshData.MaterialInstances[matIndex]);
    }
    // These should have been marked by PMC itself
    // MarkRenderStateDirty();
    // MarkRenderDynamicDataDirty();

    // COLLISION MESH DATA --
    URRGameSingleton* gameSingleton = URRGameSingleton::Get();
    const FString bodySetupModelName = GetBodySetupModelName();

    if (gameSingleton->HasSimResource(ERRResourceDataType::UE_BODY_SETUP, bodySetupModelName))
    {
        // Wait for BodySetup[bodySetupModelName] has been fully cooked
        URRCoreUtils::RegisterRepeatedExecution(
            GetWorld(),
            BodySetupTimerHandle,
            [this, gameSingleton, bodySetupModelName]()
            {
                UBodySetup* existentBodySetup = gameSingleton->GetBodySetup(bodySetupModelName);
                if (existentBodySetup)
                {
                    verify(existentBodySetup->bCreatedPhysicsMeshes);
                    // Stop the repeat timer first, note that this will invalidate all the captured variants except this
                    URRCoreUtils::StopRegisteredExecution(GetWorld(), BodySetupTimerHandle);

                    // REUSE [existentBodySetup]
                    ProcMeshBodySetup = existentBodySetup;
                    RecreatePhysicsState();

                    OnMeshCreationDone.ExecuteIfBound(true, this);
                }
            },
            0.01f);
        return true;
    }
    else
    {
        // COOK COLLISON
        // (NOTE) Temporary create an empty place-holder with [bodySetupModelName],
        // so other ProcMeshComps, wanting to reuse the same [MeshUniqueName], could check & wait for its cooking
        gameSingleton->AddDynamicResource<UBodySetup>(ERRResourceDataType::UE_BODY_SETUP, nullptr, bodySetupModelName);

        // REGISTER collision info, Creating new [ProcMeshBodySetup]
        // Ref: Super::SetCollisionConvexMeshes(MeshData.ConvexCollision);
        TArray<TArray<FVector>> convexMeshes;
        for (const auto& node : bodyMeshData.Nodes)
        {
            for (const auto& mesh : node.Meshes)
            {
                convexMeshes.Emplace(mesh.Vertices);
            }
#if RAPYUTA_SIM_DEBUG
            UE_LOG(LogRapyutaCore, Display, TEXT("Proc mesh-Convex Collision added: %d"), node.Meshes.Num());
#endif
        }

        // The current body setup will be updated to one created on-the-fly then,
        // thus its dynamically allocated collision data needs to be flushed first before registering new one
        GetBodySetup()->ClearPhysicsMeshes();
        SetCollisionConvexMeshes(convexMeshes);

        // REGISTER [ProcMeshBodySetup] depending on ASYNC/SYNC Collision cooking
        if (bUseAsyncCooking)
        {
            // Wait for [ProcMeshBodySetup]'s physics meshes to be cooked
            URRCoreUtils::RegisterRepeatedExecution(
                GetWorld(),
                CollisionCookingTimerHandle,
                [this, bodySetupModelName]()
                {
                    // (NOTE) Upon collision cooking finish, ProcMeshBodySetup will have been updated to the latest created body
                    // setup in the async queue Refer to [FinishPhysicsAsyncCook()]
                    UBodySetup* latestBodySetup = GetBodySetup();
                    if (latestBodySetup->bCreatedPhysicsMeshes)
                    {
                        // (NOTE) Stop the repeat timer first, which also destroy the captured [bodySetupModelName],
                        // thus needs to save it
                        const FString modelName = bodySetupModelName;
                        URRCoreUtils::StopRegisteredExecution(GetWorld(), CollisionCookingTimerHandle);
                        FinalizeMeshBodyCreation(latestBodySetup, modelName);
                    }
                },
                0.01f);
        }
        else
        {
            // To signal [OnMeshCreationDone] async, thus MeshCompList could get fulfilled first
            AsyncTask(ENamedThreads::GameThread,
                      [this, bodySetupModelName]() { FinalizeMeshBodyCreation(GetBodySetup(), bodySetupModelName); });
        }
        return true;
    }
}

void URRProceduralMeshComponent::FinalizeMeshBodyCreation(UBodySetup* InBodySetup, const FString& InBodySetupModelName)
{
    // Add [ProcMeshBodySetup] -> BodySetups pool
    const bool bSuccessful = (false == InBodySetup->bFailedToCreatePhysicsMeshes);
    if (bSuccessful)
    {
        InBodySetup->bSharedCookedData = true;
        URRGameSingleton::Get()->AddDynamicResource<UBodySetup>(
            ERRResourceDataType::UE_BODY_SETUP, InBodySetup, InBodySetupModelName);
    }
    OnMeshCreationDone.ExecuteIfBound(bSuccessful, this);
}

void URRProceduralMeshComponent::CreateMeshSection(const TArray<FRRMeshNodeData>& InMeshSectionData)
{
    uint32 meshSectionIndex = 0;
    for (auto& mesh : InMeshSectionData)
    {
        if (mesh.TriangleIndices.Num() == 0)
        {
            continue;
        }

#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s]CREATE PROCEDURAL MESH SECTION[%u]: Vertices(%u) - VertexColors(%u) - TriangleIndices(%u) - Normals(%u) - "
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

        // Create Mesh Section
        TArray<FVector2D> uvs;

        for(const auto& uv : mesh.UVs)
        {
            uvs.Add(FVector2D(uv));
        }
        
        Super::CreateMeshSection(meshSectionIndex,
                                 mesh.Vertices,
                                 mesh.TriangleIndices,
                                 mesh.Normals,
                                 uvs,
                                 mesh.VertexColors,
                                 mesh.ProcTangents,
                                 bUseComplexAsSimpleCollision);
        SetMeshSectionVisible(meshSectionIndex, true);
        meshSectionIndex++;
    }
}

bool URRProceduralMeshComponent::IsMeshDataValid() const
{
    const TSharedPtr<FRRMeshData> meshData = FRRMeshData::GetMeshData(MeshUniqueName);
    return meshData.IsValid() && meshData.Get()->IsValid();
}

bool URRProceduralMeshComponent::GetMeshData(FRRMeshData& OutMeshData, bool bFromBuffer)
{
    // [MeshData] could be empty if the component loads its mesh from an existing URuntimeMesh,
    // which have been created from some previous robot creation, in which case it is not reliable.
    if (bFromBuffer)
    {
        const TSharedPtr<FRRMeshData> meshData = FRRMeshData::GetMeshData(MeshUniqueName);
        if (meshData.IsValid())
        {
            OutMeshData = *meshData;
            verify(OutMeshData.IsValid());

            // [FRRMeshData::MeshDataStore] store raw data loaded from 3D cad file, thus is agnostic of Mesh comp-specific transform
            OutMeshData.TransformBy(GetComponentTransform());
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s] GetMeshData() PROC MESH DATA BUFFER [%s] IS INVALID"),
                   *GetName(),
                   *MeshUniqueName);
        }
    }
    else
    {
        // (NOTE) This will not contain BoneInfluences, which is not available by ProcMesh
        OutMeshData.Reset();
        const int32 sectionsNum = GetNumSections();
        if (sectionsNum > 0)
        {
            OutMeshData.bIsValid = true;
            FRRMeshNode meshNode;
            for (auto i = 0; i < sectionsNum; ++i)
            {
                FRRMeshNodeData section;
                TArray<FVector2D> uvs;
                
                UKismetProceduralMeshLibrary::GetSectionFromProceduralMesh(
                    this, i, section.Vertices, section.TriangleIndices, section.Normals, uvs, section.ProcTangents);

                for(const auto& uv : uvs)
                {
                    section.UVs.Add(FVector2f(uv));
                }
                
                for (auto& vertex : section.Vertices)
                {
                    vertex = GetComponentTransform().TransformPosition(vertex);
                }
                meshNode.Meshes.Add(MoveTemp(section));
            }
            OutMeshData.Nodes.Emplace(MoveTemp(meshNode));
        }
    }
    return (OutMeshData.IsValid());
}

void URRProceduralMeshComponent::SetMeshSize(const FVector& InSize)
{
    switch (ShapeType)
    {
        case ERRShapeType::BOX:
        case ERRShapeType::PLANE:
        {
            FRRMeshNodeData newNodeData;
            TArray<FVector2D> uvs;
            
            UKismetProceduralMeshLibrary::GenerateBoxMesh(InSize / 2,
                                                          newNodeData.Vertices,
                                                          newNodeData.TriangleIndices,
                                                          newNodeData.Normals,
                                                          uvs,
                                                          newNodeData.ProcTangents);

            for(const auto& uv : uvs)
            {
                newNodeData.UVs.Add(FVector2f(uv));
            }

            // Create new mesh section
            ClearAllMeshSections();
            CreateMeshSection({newNodeData});
            SetMeshSectionVisible(0, true);

            // Also new collision convex mesh
            Super::SetCollisionConvexMeshes({newNodeData.Vertices});
        }
        break;

        case ERRShapeType::CYLINDER:
        case ERRShapeType::CAPSULE:
        case ERRShapeType::SPHERE:
            // Ref:
            // UnrealEngine/Engine/Plugins/Runtime/ProceduralMeshComponent/Source/ProceduralMeshComponent/Private/KismetProceduralMeshLibrary.cpp:504
            static uint32 sCount = 0;
            UStaticMeshComponent* tempStaticMeshComp = URRUObjectUtils::CreateSelfSubobject<UStaticMeshComponent>(
                nullptr, FString::Printf(TEXT("%sSM%d"), *GetName(), ++sCount));
            UStaticMesh* staticMesh = URRGameSingleton::Get()->GetStaticMesh(MeshUniqueName);
            tempStaticMeshComp->SetStaticMesh(staticMesh);
            tempStaticMeshComp->SetWorldScale3D(InSize / staticMesh->GetBoundingBox().GetSize());

            // Get geom data from static mesh
            UKismetProceduralMeshLibrary::CopyProceduralMeshFromStaticMeshComponent(tempStaticMeshComp, 0, this, true);
            break;
    }
}

void URRProceduralMeshComponent::SetCollisionModeAvailable(bool bIsOn, bool bIsHitEventEnabled)
{
    if (bIsOn)
    {
        SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
        SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);
        SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        SetNotifyRigidBodyCollision(bIsHitEventEnabled);
    }
    else
    {
        SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);
        SetCollisionEnabled(ECollisionEnabled::NoCollision);
        SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
    }
}

void URRProceduralMeshComponent::EnableOverlapping()
{
    SetSimulatePhysics(false);
    SetCollisionProfileName(TEXT("Overlap"));
    SetCollisionEnabled(ECollisionEnabled::QueryOnly);    // SUPER IMPORTANT!
    SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
    SetGenerateOverlapEvents(true);
}
