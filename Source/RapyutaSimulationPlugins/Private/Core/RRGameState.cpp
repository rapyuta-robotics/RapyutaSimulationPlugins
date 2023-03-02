// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRGameState.h"

// UE
#include "Camera/CameraActor.h"
#include "Engine/World.h"
#include "GenericPlatform/GenericPlatformMath.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/App.h"
#include "Misc/DateTime.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameInstance.h"
#include "Core/RRMathUtils.h"
#include "Core/RRMeshActor.h"
#include "Core/RRPlayerController.h"
#include "Core/RRROS2GameMode.h"
#include "Core/RRSceneDirector.h"
#include "Core/RRUObjectUtils.h"

ARRGameState::ARRGameState()
{
    URRUObjectUtils::SetupActorTick(this, true);
    SceneInstanceClass = URRSceneInstance::StaticClass();
}

void ARRGameState::PrintSimConfig() const
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("GAME STATE CONFIG -----------------------------"));
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("SCENE_INSTANCES_NUM: %d"), SCENE_INSTANCES_NUM);
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("SCENE_INSTANCES_DISTANCE_INTERVAL: %f(cm)"), SCENE_INSTANCES_DISTANCE_INTERVAL);
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Display,
                     TEXT("SIM_OUTPUTS_BASE_FOLDER_NAME: %s -> %s"),
                     *SIM_OUTPUTS_BASE_FOLDER_NAME,
                     *GetSimOutputsBaseFolderPath());

    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("OPERATION_BATCHES_NUM: %d"), OPERATION_BATCHES_NUM);
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("ENTITY_BOUNDING_BOX_VERTEX_NORMALS:"));
    for (const auto& vertexNormal : ENTITY_BOUNDING_BOX_VERTEX_NORMALS)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Display,
                         TEXT("%s"),
                         *FString::Printf(TEXT("%f, %f, %f"), vertexNormal.X, vertexNormal.Y, vertexNormal.Z));
    }
}

void ARRGameState::StartSim()
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("[Start Sim with Num of SceneInstances: %d]"), SCENE_INSTANCES_NUM);

    URRGameSingleton::Get()->PrintSimConfig();
    PrintSimConfig();

    GameMode = URRCoreUtils::GetGameMode<ARRGameMode>(this);
    check(GameMode);

    GameInstance = URRCoreUtils::GetGameInstance<URRGameInstance>(this);
    check(GameInstance);

    // Each Sim scene instance has a Player Controller on its own, the max number of which is defined by
    // [UGameViewportClient::MaxSplitscreenPlayers]
    const int32 maxSplitscreenPlayers = URRCoreUtils::GetMaxSplitscreenPlayers(this);
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("MAX SPLIT SCREEN PLAYERS: %d"), maxSplitscreenPlayers);
    verify(SCENE_INSTANCES_NUM <= maxSplitscreenPlayers);

    // 0- Stream level & Fetch static-env actors
    SetupEnvironment();

    for (int8 i = 0; i < SCENE_INSTANCES_NUM; ++i)
    {
        // 1 - Create a new one added into [SceneInstanceList], each of which houses ~Common objects, and configures
        // ~CommonClass, ~SceneDirectorClass
        CreateSceneInstance(i);

        // 2 - Create <SceneType>Common & <Plugin>Common objects
        CreateServiceObjects(i);

        // 3 - Trigger OnStartSim() for creating plugins' own common artifacts
        StartSubSim(i);

        // 4 - Do preliminary configuration/spawning for the Sim operation
        InitializeSim(i);

        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("SIM SCENE INSTANCE [%d] STARTED SUCCESSFULLY! "), i);
    }
}

void ARRGameState::SetupEnvironment()
{
    FetchEnvStaticActors();
}

void ARRGameState::FetchEnvStaticActors()
{
    // Fetch env actors
    UWorld* currentWorld = GetWorld();
    checkf(currentWorld, TEXT("Failed fetching Game World"));

    // Fetch Main background environment
    MainEnvironment = URRUObjectUtils::FindEnvironmentActor(currentWorld);
    // Not all maps has MainEnvironment setup

    MainFloor = URRUObjectUtils::FindFloorActor(currentWorld);
    // Not all maps has MainFloor setup

    MainWall = URRUObjectUtils::FindWallActor(currentWorld);
    // Not all maps has MainWall setup

    MainLights = URRUObjectUtils::FindActorListByType<ALight>(currentWorld);
    // Not all maps has MainLights setup
}

void ARRGameState::CreateSceneInstance(int8 InSceneInstanceId)
{
    verify(InSceneInstanceId >= 0);
    URRSceneInstance* newSceneInstance = Cast<URRSceneInstance>(URRUObjectUtils::CreateSelfSubobject(
        this, SceneInstanceClass, FString::Printf(TEXT("%d_%s"), InSceneInstanceId, *URRGameInstance::SMapName)));
    verify(newSceneInstance);
    newSceneInstance->ConfigureStaticClasses();
    SceneInstanceList.Add(newSceneInstance);

    if (IsNetMode(NM_Standalone))
    {
        // Create SceneDirector's own PlayerController, which actually creates its instance based on PlayerControllerClass
        // configured in [GameMode]'s ctor! ! [PlayerController] MUST BE CREATED EARLIER THAN ALL OTHER SIM SCENE'S ACTORS AND
        // OBJECTS
        newSceneInstance->PlayerController =
            (0 == InSceneInstanceId)
                ?    // This must be checked explicitly versus [0], NOT [URRActorCommon::DEFAULT_SCENE_INSTANCE_ID]
                URRCoreUtils::GetPlayerController<ARRPlayerController>(0, this)
                : URRCoreUtils::CreatePlayerController<ARRPlayerController>(InSceneInstanceId, this);
        newSceneInstance->PlayerController->SceneInstanceId = InSceneInstanceId;
    }
}

void ARRGameState::CreateServiceObjects(int8 InSceneInstanceId)
{
    // Instantiate Actor Common, of which the ctor should not depend on any play resource.
    // Common objects in general must be created here first, then will be referenced anywhere else.
    auto& sceneInstance = SceneInstanceList[InSceneInstanceId];
    verify(sceneInstance);
    sceneInstance->ActorCommon = URRActorCommon::GetActorCommon(InSceneInstanceId, sceneInstance->ActorCommonClass, this);

    int32 sideNum = FMath::CeilToInt(FMath::Sqrt(static_cast<float>(SCENE_INSTANCES_NUM)));
    sceneInstance->ActorCommon->SceneInstanceLocation.X = (InSceneInstanceId / sideNum) * SCENE_INSTANCES_DISTANCE_INTERVAL;
    sceneInstance->ActorCommon->SceneInstanceLocation.Y = (InSceneInstanceId % sideNum) * SCENE_INSTANCES_DISTANCE_INTERVAL;
    sceneInstance->ActorCommon->SceneInstanceLocation.Z = 0.f;
}

bool ARRGameState::HasSceneInstanceListBeenCreated(bool bIsLogged) const
{
    if (!SceneInstanceList.Num())
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("SceneInstanceList is EMPTY!"));
        }
        return false;
    }

    return true;
}

void ARRGameState::StartSubSim(int8 InSceneInstanceId)
{
    SceneInstanceList[InSceneInstanceId]->ActorCommon->OnStartSim();
}

void ARRGameState::InitializeSim(int8 InSceneInstanceId)
{
    // Sim Scene Instance List
    verify(SceneInstanceList.IsValidIndex(InSceneInstanceId));
    auto& sceneInstance = SceneInstanceList[InSceneInstanceId];

    // Spawn [SceneDirector], which runs the main operation of the mode
    // This, due to making use of scene instance's actors, is spawned last.
    // ALSO, we should not write Scene-specific operation/functionality-related -starting codes inside scene instance's actors
    // (camera, actors, spawners) and leave it up to [SceneDirector] to decide the start sequence order!
    sceneInstance->SceneDirector = Cast<ARRSceneDirector>(
        URRUObjectUtils::SpawnSimActor(GetWorld(), InSceneInstanceId, sceneInstance->SceneDirectorClass, TEXT("SceneDirector")));
    verify(sceneInstance->SceneDirector);
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Display,
                     TEXT("Scene Director actor named [%s] - SceneInstanceId(%d): SPAWNED!"),
                     *sceneInstance->SceneDirector->GetName(),
                     sceneInstance->SceneDirector->SceneInstanceId);
}

bool ARRGameState::HasInitialized(bool bIsLogged) const
{
    // [SceneInstanceList]
    if (!HasSceneInstanceListBeenCreated(bIsLogged))
    {
        if (bIsLogged)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("[SceneInstanceList]: Not yet created!"));
        }
        return false;
    }

    // AND its specific contents (~Common, ~SceneDirector) are verified here-in!
    for (int8 i = 0; i < SceneInstanceList.Num(); ++i)
    {
        if (!SceneInstanceList[i]->IsValid(bIsLogged))
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("[sceneInstance index[%d]]: is invalid!"), i);
            }
            return false;
        }
    }

    return true;
}

bool ARRGameState::HaveAllSceneInstancesCompleted() const
{
    for (int8 i = 0; i < SceneInstanceList.Num(); ++i)
    {
        if (false == SceneInstanceList[i]->SceneDirector->HasOperationCompleted(false))
        {
            return false;
        }
    }

    return true;
}

void ARRGameState::MoveEnvironmentToSceneInstance(int8 InSceneInstanceId)
{
    if (InSceneInstanceId != LastSceneInstanceId)
    {
        if (IsValid(MainEnvironment))
        {
            MainEnvironment->AddActorWorldOffset(URRCoreUtils::GetSceneInstanceLocation(InSceneInstanceId) -
                                                 URRCoreUtils::GetSceneInstanceLocation(LastSceneInstanceId));
        }
        LastSceneInstanceId = InSceneInstanceId;
    }
}

void ARRGameState::FinalizeSim()
{
    if (IsNetMode(NM_Standalone))
    {
        for (auto& sceneInstance : SceneInstanceList)
        {
            verify(sceneInstance && sceneInstance->IsValid(true));
        }
    }
}

void ARRGameState::BeginPlay()
{
    Super::BeginPlay();

    // Trigger OnBeginPlay() for plugins' own common artifacts, which have been created earlier in this ::[StartSim()]
    BeginSubPlay();
}

void ARRGameState::BeginSubPlay()
{
    for (auto& sceneInstance : SceneInstanceList)
    {
        sceneInstance->ActorCommon->OnBeginPlay();
    }
}

void ARRGameState::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // This function works like an unwinding stack. where the children class's contents
    // are finalized ahead of parent's
    // => Reaching here means that all ~Common's OnEndPlay() have already finished!
    // 1 -
    for (auto& sceneInstance : SceneInstanceList)
    {
        sceneInstance->ActorCommon->OnEndPlay(EndPlayReason);
    }

    // 2 - Clear and reset Sim [static] contents (like ~FunctionLibrary's static ~Common object handles, due to not managed by UE
    // UObject system)
    FinalizeSim();

    // 3 - These resources were initialized in [ARRGameMode::StartPlay()], which does not have a corresponding EndPlay()
    URRGameSingleton::Get()->FinalizeResources();

    Super::EndPlay(EndPlayReason);
}

void ARRGameState::SetAllEntitiesActivated(bool bIsActivated)
{
    for (auto& entity : AllDynamicMeshEntities)
    {
        entity->SetActivated(bIsActivated);
    }
}

ARRMeshActor* ARRGameState::FindEntityByModel(const FString& InEntityModelName, bool bToActivate, bool bToTakeAway)
{
    ARRMeshActor* resultEntity = nullptr;
    for (auto& entity : AllDynamicMeshEntities)
    {
        if (InEntityModelName == entity->EntityModelName)
        {
            if (bToActivate)
            {
                // Bring the found one on stage
                entity->SetActivated(true);
            }
            resultEntity = entity;
            break;
        }
    }

    if (bToTakeAway && resultEntity)
    {
        // Better save first then remove
        OrphanEntities.AddUnique(resultEntity);
        // (NOTE) Use RemoveSwap() during the for loop would cause crash:
        // "Attempting to use a container element that already comes from the container being modified"
        AllDynamicMeshEntities.RemoveSwap(resultEntity);
    }
    return resultEntity;
}

void ARRGameState::AddEntity(ARRMeshActor* InEntity)
{
    AllDynamicMeshEntities.AddUnique(InEntity);
}
