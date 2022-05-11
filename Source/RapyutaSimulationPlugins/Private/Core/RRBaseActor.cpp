// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRBaseActor.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameState.h"
#include "Core/RRPlayerController.h"
#include "Core/RRUObjectUtils.h"

TMap<UClass*, TUniquePtr<std::once_flag>> ARRBaseActor::OnceFlagList;
std::once_flag ARRBaseActor::OnceFlag;
int8 ARRBaseActor::SSceneInstanceId = URRActorCommon::DEFAULT_SCENE_INSTANCE_ID;
ARRBaseActor::ARRBaseActor()
{
    SceneInstanceId = ARRBaseActor::SSceneInstanceId;

    // UE HAS A VERY SPECIAL WAY OF HAVING [AActor] INSTANTIATED, IN WHICH
    // DEFAULT OBJECT IS ALWAYS CREATED AS THE EDITOR IS LOADED UP.
    // HENCE, IT'S RECOMMENDED NOT TO PUT PARTICULAR ACTOR'S CONTENT INSTANTIATION OR INITIALIZATION INSIDE CTOR,
    // ESPECIALLY IF THOSE CONTENTS (eg CHILD MESH COMPONENTS) RELY ON SIM'S GLOBAL RESOURCES, WHICH ARE INITIALIZED LATER DURING
    // SIM START-UP!
    //
    // Please put them in [Initialize()] instead!
}

ARRBaseActor::ARRBaseActor(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SceneInstanceId = ARRBaseActor::SSceneInstanceId;
}

// (NOTE) This method, if being called, could only go with a RRGameMode-inheriting game mode setup!
// Currently, ARRROS2GameMode & ARRGameMode are separate ones.
// & Maps of ARRROS2GameMode do NOT YET have actors invoking this method.
// It is up to the Child class, [Initialize()] could be run inside [BeginPlay()] or some place else in advance!
bool ARRBaseActor::Initialize()
{
    UClass* thisClass = GetClass();
    if (false == OnceFlagList.Contains(thisClass))
    {
        ARRBaseActor::OnceFlagList.Add(thisClass, MakeUnique<std::once_flag>());
    }
    std::call_once(*OnceFlagList[thisClass], [this]() { PrintSimConfig(); });

    // Tick setup
    SetTickEnabled(ActorInfo ? ActorInfo->IsTickEnabled : false);

    // SETUP + CONFIGURE ESSENTIAL GAME & SIM COMMON OBJECTS
    GameMode = URRCoreUtils::GetGameMode<ARRGameMode>(this);
    verify(GameMode);

    GameState = URRCoreUtils::GetGameState<ARRGameState>(this);
    verify(GameState);

    PlayerController = URRCoreUtils::GetPlayerController<ARRPlayerController>(SceneInstanceId, this);
    verify(PlayerController);

    ActorCommon = URRActorCommon::GetActorCommon(SceneInstanceId);
    return true;
}

bool ARRBaseActor::HasInitialized(bool bIsLogged) const
{
    if (!ActorInfo.IsValid())
    {
        if (bIsLogged)
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("[%s] [ActorInfo] has not been instantiated!!"), *GetName());
        }
        return false;
    }

    return true;
}

void ARRBaseActor::SetTickEnabled(bool bInIsTickEnabled)
{
    URRUObjectUtils::SetupActorTick(this, bInIsTickEnabled);
    if (bInIsTickEnabled)
    {
        // This does not apply for all general actors (if called could cause crash), and so normally for RR-based actors only.
        // https://forums.unrealengine.com/t/actor-component-will-not-tick/96404/6
        PrimaryActorTick.RegisterTickFunction(GetLevel());
    }
}

void ARRBaseActor::Reset()
{
    ActorInfo->ClearMeshInfo();
}
