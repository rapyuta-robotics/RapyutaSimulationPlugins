// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRBaseActor.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRGameState.h"
#include "Core/RRPlayerController.h"
#include "Core/RRUObjectUtils.h"

TMap<UClass*, TUniquePtr<std::once_flag>> ARRBaseActor::OnceFlagList;
std::once_flag ARRBaseActor::OnceFlag;
int8 ARRBaseActor::SSceneInstanceId = URRActorCommon::DEFAULT_SCENE_INSTANCE_ID;
ARRBaseActor::ARRBaseActor()
{
    SetupDefaultBase();
}

ARRBaseActor::ARRBaseActor(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefaultBase();
}

void ARRBaseActor::SetupDefaultBase()
{
    // UE has a very special way of having [AActor] instantiated, in which
    // default object is always created as the editor is loaded up.
    // Hence, it's recommended not to put particular Actor's content instantiation or initialization inside ctor,
    // especially if those contents (eg child mesh components) rely on sim's global resources, which are initialized later during
    // sim start-up!
    //
    //
    // Please put them in [Initialize()] instead!
    SceneInstanceId = ARRBaseActor::SSceneInstanceId;
    bReplicates = true;
    // Needs to be set to relevant otherwise it won't consistantly replicate
    bAlwaysRelevant = true;
}

void ARRBaseActor::PreInitializeComponents()
{
    Super::PreInitializeComponents();

    // SETUP + CONFIGURE ESSENTIAL GAME & SIM COMMON OBJECTS
    // NOTE: These are used ONLY for dynamic runtime actors.
    // In child BP actors, except for GameSingleton, other Game framework entities maybe not available yet
    GameMode = URRCoreUtils::GetGameMode<ARRGameMode>(this);
    GameState = URRCoreUtils::GetGameState<ARRGameState>(this);

    if (IsNetMode(NM_Standalone))
    {
        PlayerController = URRCoreUtils::GetPlayerController<ARRPlayerController>(SceneInstanceId, this);
    }

    GameSingleton = URRGameSingleton::Get();
    check(GameSingleton);
}

bool ARRBaseActor::Initialize()
{
    UClass* thisClass = GetClass();
    if (false == OnceFlagList.Contains(thisClass))
    {
        ARRBaseActor::OnceFlagList.Add(thisClass, MakeUnique<std::once_flag>());
    }
    std::call_once(*OnceFlagList[thisClass], [this]() { PrintSimConfig(); });

    // Entity Model Name
    if (ActorInfo.IsValid())
    {
        EntityModelName = ActorInfo->EntityModelName;
    }

    // Tick setup
    SetTickEnabled(ActorInfo ? ActorInfo->bIsTickEnabled : false);

    // NOTE: This is meant to be created as GameState creates Scene instances (with custom ~CommonClass), which are only at runtime,
    // thus could not be inside either [PreInitializeComponents()] or [PostInitializeComponents()]
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
