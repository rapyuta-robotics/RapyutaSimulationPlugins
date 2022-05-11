// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// std
#include <mutex>

// UE
#include "GameFramework/Pawn.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"

#include "RRBaseActor.generated.h"

class ARRGameMode;
class ARRGameState;
class URRGameSingleton;
class ARRPlayerController;

/* Base actor class for all Rapyuta Sim actors:
 * - Be assigned a Scene instance id
 * - Provide accessible handles to the common Game framework objects (GameInstance, GameMode, GameState) and
 * the corresponding [ActorCommon] of the Scene instance it belongs to.
 * - Does not necessarily have a USceneComponent, which is only required for spatially wise or mesh-based actors, as Root!
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRBaseActor : public APawn
{
    GENERATED_BODY()
public:
    // [std::once_flag] also applies even in case of consecutive PIE runs,
    // thus if running in PIE and the called function is required to run again each time (though still once per PIE) then,
    // another (context-specific) method (without using [static res]) should be considered!
    // Used for class having multiple-branch child classes (multiple-branch inheritance tree)
    static TMap<UClass*, TUniquePtr<std::once_flag>> OnceFlagList;
    // Used for class having single-branch child classes (linear inheritance tree)
    static std::once_flag OnceFlag;
    ARRBaseActor();
    ARRBaseActor(const FObjectInitializer& ObjectInitializer);
    TSharedPtr<FRRActorSpawnInfo> ActorInfo = nullptr;

    static int8 SSceneInstanceId;
    UPROPERTY()
    int8 SceneInstanceId = URRActorCommon::DEFAULT_SCENE_INSTANCE_ID;

    UPROPERTY()
    ARRGameMode* GameMode = nullptr;

    UPROPERTY()
    ARRGameState* GameState = nullptr;

    UPROPERTY()
    URRGameSingleton* GameSingleton = nullptr;

    UPROPERTY()
    ARRPlayerController* PlayerController = nullptr;

    UPROPERTY()
    URRActorCommon* ActorCommon = nullptr;

    UPROPERTY(VisibleAnywhere)
    FString EntityModelName;

    UPROPERTY()
    FTimerHandle GenericTimerHandle;

public:
    virtual bool Initialize();
    virtual bool HasInitialized(bool bIsLogged = false) const;
    virtual void Reset();
    void SetTickEnabled(bool bInIsTickEnabled);

protected:
    virtual void PrintSimConfig() const
    {
    }
};
