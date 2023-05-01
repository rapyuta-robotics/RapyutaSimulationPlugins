/**
 * @file RRBaseActor.h
 * @brief Base actor class for all Rapyuta Sim actors
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// std
#include <mutex>

// UE
#include "GameFramework/Pawn.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"

#include "RRBaseActor.generated.h"

class ARRROS2GameMode;
class ARRGameState;
class URRGameSingleton;
class ARRPlayerController;

/**  @brief Base actor class for all Rapyuta Sim actors:
 * - Be assigned a Scene instance id
 * - Provide accessible handles to the common Game framework objects (GameInstance, GameMode, GameState) and the corresponding
 * [ActorCommon] of the Scene instance it belongs to.
 * - Does not necessarily have a USceneComponent, which is only required for spatially wise or mesh-based actors, as Root!
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRBaseActor : public APawn
{
    GENERATED_BODY()
public:
    /**
     * @brief
     * [std::once_flag] also applies even in case of consecutive PIE runs,
     * thus if running in PIE and the called function is required to run again each time (though still once per PIE) then,
     * another (context-specific) method (without using [static res]) should be considered!
     * Used for class having multiple-branch child classes (multiple-branch inheritance tree)
     */
    static TMap<UClass*, TUniquePtr<std::once_flag>> OnceFlagList;

    //! Used for class having single-branch child classes (linear inheritance tree)
    static std::once_flag OnceFlag;

    /**
     * @brief Construct a new ARRBaseActor object
     * UE has a very special way of having [AActor] instantiated, in which
     * default object is always created as the editor is loaded up.
     * Hence, it's recommended not to put particular Actor's content instantiation or initialization inside ctor,
     * especially if those contents (eg child mesh components) rely on sim's global resources, which are initialized later during
     * sim start-up!
     *
     * Please put them in [Initialize()] instead!
     */
    ARRBaseActor();

    /**
     * @brief Construct a new ARRBaseActor object
     *
     * @param ObjectInitializer
     */
    ARRBaseActor(const FObjectInitializer& ObjectInitializer);

    /**
     * @brief Initialize default components being configurable in child BP classes.
     * Could only be called in constructor.
     */
    void SetupDefaultBase();

    TSharedPtr<FRRActorSpawnInfo> ActorInfo = nullptr;

    static int8 SSceneInstanceId;

    //! Used for SceneDirector
    UPROPERTY()
    ARRGameState* RRGameState = nullptr;

    //! Used for Asset loading
    UPROPERTY()
    URRGameSingleton* RRGameSingleton = nullptr;

    //! Pointer to GameMode for convinience.
    UPROPERTY()
    ARRROS2GameMode* RRROS2GameMode = nullptr;

    //! Used for DataSynth app.
    UPROPERTY()
    ARRPlayerController* RRPlayerController = nullptr;

    UPROPERTY()
    int8 SceneInstanceId = URRActorCommon::DEFAULT_SCENE_INSTANCE_ID;

    UPROPERTY()
    URRActorCommon* ActorCommon = nullptr;

    UPROPERTY(VisibleAnywhere)
    FString EntityModelName;
    void SetEntityModelName(const FString& InEntityModelName)
    {
        EntityModelName = InEntityModelName;
    }

    bool IsDataSynthEntity() const
    {
        return ActorInfo.IsValid() ? ActorInfo->bIsDataSynthEntity : false;
    }

    UPROPERTY()
    FTimerHandle GenericTimerHandle;

public:
    /**
     * @brief Set #GameMode #GameState #GameSingleton #PlayerController
     * (NOTE) This method, if being called, could only go with a RRGameMode-inheriting game mode setup!
     * Currently, ARRROS2GameMode & ARRGameMode are separate ones.
     * & Maps of ARRROS2GameMode do NOT YET have actors invoking this method.
     * It is up to the Child class, [Initialize()] could be run inside [BeginPlay()] or some place else in advance!
     *
     * @return true
     * @return false
     */
    virtual bool Initialize();

    virtual bool HasInitialized(bool bIsLogged = false) const;

    /**
     * @brief Rest. Calls #ActorInfo::ClearMeshInfo.
     *
     */
    virtual void Reset();

    /**
     * @brief Use #URRUObjectUtils::SetupActorTick
     *
     * @param bInIsTickEnabled
     */
    void SetTickEnabled(bool bInIsTickEnabled);

protected:
    /**
     * @brief Set #GameMode #GameState #GameSingleton #PlayerController
     *
     */
    virtual void PreInitializeComponents() override;
    virtual void PrintSimConfig() const
    {
    }
};
