// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRActorCommon.h"

// UE

// RapyutaSimulationPlugins
#include "Core/RRCamera.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRGameState.h"
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
            UE_LOG(LogRapyutaCore, Display, TEXT("[URRSceneInstance] PlayerController is NULL!"));
        }
        return false;
    }

    if (!ActorCommon)
    {
        if (bIsLogged)
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("[URRSceneInstance] ActorCommon object is NULL!"));
        }
        return false;
    }

    if (!ActorCommon->HasInitialized(bIsLogged))
    {
        if (bIsLogged)
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("[URRSceneInstance] ActorCommon: Not yet initialized!"))
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
            UE_LOG(LogRapyutaCore, Display, TEXT("[URRSceneInstance] [SceneDirector] is NULL!"));
        }
        return false;
    }
    return true;
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
                                     bool bInIsCollisionEnabled)
    : EntityModelName(InEntityModelName),
      UniqueName(InUniqueName),
      ActorTransform(InActorTransform),
      MeshRelTransformList(InMeshRelTransformList),
      MeshUniqueNameList(InMeshUniqueNameList),
      MaterialNameList(InMaterialNameList),
      bIsStationary(bInIsStationary),
      bIsPhysicsEnabled(bInIsPhysicsEnabled),
      bIsCollisionEnabled(bInIsCollisionEnabled)
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
                                   bool bInIsCollisionEnabled)
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
    bIsTickEnabled = false;
    bIsSelfCollision = false;
    bIsDataSynthEntity = false;
}

// ===================================================================================================================================
// [URRActorCommon] --
//
std::once_flag URRActorCommon::OnceFlag;
uint64 URRActorCommon::SLatestSceneId = 0;
// This is used as a map due to the unequivalence of element order with [ARRGameState::SceneInstanceList]
TMap<int8, URRActorCommon*> URRActorCommon::SActorCommonList;
URRActorCommon* URRActorCommon::GetActorCommon(int8 InSceneInstanceId, UClass* InActorCommonClass, UObject* InOuter)
{
    if (!SActorCommonList.Contains(InSceneInstanceId))
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
    // Here, we initialize artifacts that utilized play resources
    UWorld* currentWorld = GetWorld();
    checkf(currentWorld, TEXT("[URRActorCommon::SetupEnvironment] Failed fetching Game World"));

    // Fetch Main background environment
    if (!MainEnvironment)
    {
        MainEnvironment = URRUObjectUtils::FindEnvironmentActor(currentWorld);
        // Not all maps has MainEnvironment setup
    }

    if (!MainFloor)
    {
        MainFloor = URRUObjectUtils::FindFloorActor(currentWorld);
        // Not all maps has MainFloor setup
    }

    if (!MainWall)
    {
        MainWall = URRUObjectUtils::FindWallActor(currentWorld);
        // Not all maps has MainWall setup
    }

    // Spawn MainCamera
    MainCamera =
        Cast<ARRCamera>(URRUObjectUtils::SpawnSimActor(GetWorld(),
                                                       SceneInstanceId,
                                                       ARRCamera::StaticClass(),
                                                       TEXT("RapyutaCamera"),
                                                       FString::Printf(TEXT("%d_MainCamera"), SceneInstanceId),
                                                       FTransform(SceneInstanceLocation),
                                                       ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn));
    verify(MainCamera);
}

bool URRActorCommon::HasInitialized(bool bIsLogged) const
{
    if (!MainCamera)
    {
        if (bIsLogged)
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("[URRActorCommon]:: MainCamera is NULL!"))
        }
        return false;
    }
    return true;
}

void URRActorCommon::MoveEnvironmentToSceneInstance(int8 InSceneInstanceId)
{
    if (IsValid(MainEnvironment))
    {
        const FVector currentEnvLocation = MainEnvironment->GetActorLocation();
        const FVector sceneInstanceLocation = URRCoreUtils::GetSceneInstanceLocation(InSceneInstanceId);
        MainEnvironment->SetActorLocation(FVector(sceneInstanceLocation.X, sceneInstanceLocation.Y, currentEnvLocation.Z));
    }
}
