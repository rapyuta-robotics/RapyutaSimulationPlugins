// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRActorCommon.h"

// UE

// RapyutaSimulationPlugins
#include "Core/RRCamera.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRGameState.h"
#include "Core/RRMeshActor.h"
#include "Core/RRPlayerController.h"
#include "Core/RRSceneDirector.h"
#include "Core/RRThreadUtils.h"
#include "Core/RRUObjectUtils.h"

void URRSceneInstance::ConfigureStaticClasses()
{
    ActorCommonClass = URRActorCommon::StaticClass();
    SceneDirectorClass = ARRSceneDirector::StaticClass();
}

bool URRSceneInstance::IsValid(bool bIsLogged) const
{
    if (!PlayerController)
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("PlayerController is NULL!"));
        }
        return false;
    }

    if (!ActorCommon)
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("ActorCommon object is NULL!"));
        }
        return false;
    }

    if (!ActorCommon->HasInitialized(bIsLogged))
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("ActorCommon: Not yet initialized!"))
        }
        return false;
    }

    // [SceneDirector] waits for [GameState], which instantiate [SimSceneInstance] + [PlayerController] to initialize,
    // thus of course not include the check for its [HasSimSceneInitialized()] here!
    // In essence, [SceneDirector]'s initialization is considered as Sim's post-initialization.
    if (!SceneDirector)
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("SceneDirector is NULL!"));
        }
        return false;
    }
    return true;
}

// ===================================================================================================================================
// [FRRHomoMeshEntityGroup] --
//
FString FRRHomoMeshEntityGroup::GetGroupModelName() const
{
    const int32 num = Num();
    return (num == 1)  ? Entities[0]->EntityModelName
           : (num > 1) ? FString::Printf(TEXT("Merged%d_%s"), num, *Entities[0]->EntityModelName)
                       : FString();
}

FString FRRHomoMeshEntityGroup::GetGroupName() const
{
    const int32 num = Num();
    return (num == 1)  ? Entities[0]->GetName()
           : (num > 1) ? FString::Printf(TEXT("Merged%d_%s"), num, *Entities[0]->GetName())
                       : FString();
}
// ===================================================================================================================================
// [FRRActorSpawnInfo] --
//
FRRActorSpawnInfo::FRRActorSpawnInfo()
{
    bIsTickEnabled = false;
    bIsStationary = false;
    bIsPhysicsEnabled = true;
    bIsCollisionEnabled = true;
    bIsSelfCollision = false;
    bIsOverlapEventEnabled = false;
    bIsDataSynthEntity = false;
}

FRRActorSpawnInfo::FRRActorSpawnInfo(const FString& InEntityModelName,
                                     const FString& InUniqueName,
                                     const FTransform& InActorTransform,
                                     const TArray<FTransform>& InMeshRelTransformList,
                                     const TArray<FString>& InMeshUniqueNameList,
                                     const TArray<FString>& InMaterialNameList,
                                     bool bInIsStationary,
                                     bool bInIsPhysicsEnabled,
                                     bool bInIsCollisionEnabled,
                                     bool bInIsOverlapEventEnabled)
    : EntityModelName(InEntityModelName),
      UniqueName(InUniqueName),
      ActorTransform(InActorTransform),
      MeshRelTransformList(InMeshRelTransformList),
      MeshUniqueNameList(InMeshUniqueNameList),
      MaterialNameList(InMaterialNameList),
      bIsStationary(bInIsStationary),
      bIsPhysicsEnabled(bInIsPhysicsEnabled),
      bIsCollisionEnabled(bInIsCollisionEnabled),
      bIsOverlapEventEnabled(bInIsOverlapEventEnabled)
{
    bIsTickEnabled = false;
    bIsSelfCollision = false;
    bIsDataSynthEntity = false;
}

void FRRActorSpawnInfo::operator()(const FString& InEntityModelName,
                                   const FString& InUniqueName,
                                   const FTransform& InActorTransform,
                                   const TArray<FTransform>& InMeshRelTransformList,
                                   const TArray<FString>& InMeshUniqueNameList,
                                   const TArray<FString>& InMaterialNameList,
                                   bool bInIsStationary,
                                   bool bInIsPhysicsEnabled,
                                   bool bInIsCollisionEnabled,
                                   bool bInIsOverlapEventEnabled)
{
    EntityModelName = InEntityModelName;
    UniqueName = InUniqueName;
    ActorTransform = InActorTransform;
    MeshRelTransformList = InMeshRelTransformList;
    MeshUniqueNameList = InMeshUniqueNameList;
    MeshUniqueNameList.Remove(EMPTY_STR);

    // Even if this actor has mesh content, it could use the [StaticMesh]'s built-in materials.
    // Thus, [MaterialNameList], which is configured to be passed in at [ARRMeshActor] spawning,
    // does NOT NECESSARILY always contain some material!
    MaterialNameList = InMaterialNameList;
    MaterialNameList.Remove(EMPTY_STR);

    bIsStationary = bInIsStationary;
    bIsPhysicsEnabled = bInIsPhysicsEnabled;
    bIsCollisionEnabled = bInIsCollisionEnabled;
    bIsOverlapEventEnabled = bInIsOverlapEventEnabled;
    bIsTickEnabled = false;
    bIsSelfCollision = false;
    bIsDataSynthEntity = false;
}

// ===================================================================================================================================
// [URRActorCommon] --
//
std::once_flag URRActorCommon::OnceFlag;
uint64 URRActorCommon::SLatestSceneId = 0;
TArray<int32> URRActorCommon::StaticCustomDepthStencilList;

//! This is used as a map due to the unequivalence of element order with [ARRGameState::SceneInstanceList]
TMap<int8, URRActorCommon*> URRActorCommon::SActorCommonList;

URRActorCommon* URRActorCommon::GetActorCommon(int8 InSceneInstanceId, UClass* InActorCommonClass, UObject* InOuter)
{
    if (!SActorCommonList.Contains(InSceneInstanceId))
    {
        // First time fetching should come with valid [InOuter] for the creation
        if (InOuter)
        {
            verify(InActorCommonClass);
            verify(InOuter && InOuter->IsValidLowLevel());
            URRActorCommon* actorCommon = Cast<URRActorCommon>(URRUObjectUtils::CreateSelfSubobject(
                InOuter, InActorCommonClass, FString::Printf(TEXT("%d_ActorCommon"), InSceneInstanceId)));
            verify(actorCommon);
            actorCommon->SceneInstanceId = InSceneInstanceId;
            SActorCommonList.Add(InSceneInstanceId, actorCommon);

            // This [OnPostWorldCleanup()] is meant to called once only!
            // Not sure about whether there is an UE internal check on already bound function, but better not relying on it.
            if (URRActorCommon::DEFAULT_SCENE_INSTANCE_ID == InSceneInstanceId)
            {
                FWorldDelegates::OnPostWorldCleanup.AddStatic(&URRActorCommon::OnPostWorldCleanup);
            }
        }
        else
        {
            return nullptr;
        }
    }

    return SActorCommonList[InSceneInstanceId];
}

void URRActorCommon::OnPostWorldCleanup(UWorld* InWorld, bool /*bInSessionEnded*/, bool /*bInCleanupResources*/)
{
    if ((URRActorCommon::SActorCommonList.Num() > 0) && URRActorCommon::SActorCommonList[0]->GetWorld() == InWorld)
    {
        URRActorCommon::SActorCommonList.Reset();
    }
}

URRActorCommon::URRActorCommon()
{
}

void URRActorCommon::PrintSimConfig() const
{
    UE_LOG(LogRapyutaCore, Display, TEXT("ACTOR COMMON CONFIG -----------------------------"));
}

void URRActorCommon::OnStartSim()
{
    std::call_once(OnceFlag, [this]() { PrintSimConfig(); });

    UWorld* currentWorld = GetWorld();
    checkf(currentWorld, TEXT("[URRActorCommon::OnStartSim] Failed fetching Game World"));

    GameMode = Cast<ARRGameMode>(currentWorld->GetAuthGameMode());
    check(GameMode);

    GameState = Cast<ARRGameState>(currentWorld->GetGameState());
    check(GameState);

    SetupEnvironment();
}

void URRActorCommon::SetupEnvironment()
{
    UWorld* currentWorld = GetWorld();
    checkf(currentWorld, TEXT("[URRActorCommon::SetupEnvironment] Failed fetching Game World"));

    // By default, these just reference GameState's floor & wall but could be overridable in child class setup
    SceneFloor = GameState->MainFloor;
    SceneWall = GameState->MainWall;

    // Spawn Scene main camera
    SceneCamera =
        Cast<ARRCamera>(URRUObjectUtils::SpawnSimActor(GetWorld(),
                                                       SceneInstanceId,
                                                       ARRCamera::StaticClass(),
                                                       TEXT("RapyutaCamera"),
                                                       FString::Printf(TEXT("%d_SceneCamera"), SceneInstanceId),
                                                       FTransform(SceneInstanceLocation),
                                                       ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn));
}

bool URRActorCommon::HasInitialized(bool bIsLogged) const
{
    if (false == GetWorld()->IsNetMode(ENetMode::NM_Standalone))
    {
        return true;
    }

    if (!SceneCamera)
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("SceneCamera is NULL!"))
        }
        return false;
    }
    return true;
}
