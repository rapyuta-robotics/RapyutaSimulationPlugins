/**
 * @file RRActorCommon.h
 * @brief Asset utils
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// !NOTE: THIS FILE IS THE CORE INCLUDED FILE, THUS IT IS HIGHLY NON-RECOMMENDED TO INCLUDE OTHER FILES HERE-IN TO AVOID CYCLIC
// INCLUSION.
#include "CoreMinimal.h"

// Native
#include <mutex>
#include <type_traits>

// UE
#include "Engine.h"
#include "Math/Color.h"
#include "Misc/Paths.h"

// rclUE
#include "logUtilities.h"

// RapyutaSimulationPlugins
#include "RapyutaSimulationPlugins.h"

#include "RRActorCommon.generated.h"

#define RAPYUTA_SIM_VERBOSE (0)    // todo make this CVar

//! NOTES: These 2 DEBUG directives are used mainly for ref code annnotation, thus must NEVER be turned on here globally!
#define RAPYUTA_SIM_DEBUG (0)
#define RAPYUTA_SIM_VISUAL_DEBUG (0)

#define RAPYUTA_USE_SCENE_DIRECTOR (0)

#define RAPYUTA_RUNTIME_MESH_ENTITY_USE_STATIC_MESH (1)
#define RAPYUTA_RUNTIME_MESH_ENTITY_USE_PROCEDURAL_MESH (!RAPYUTA_RUNTIME_MESH_ENTITY_USE_STATIC_MESH)

class ARRGameState;
class ARRMeshActor;
class ARRMeshActor;
class ARRCamera;
class URRCoreUtils;

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRStreamingLevelInfo
{
    GENERATED_BODY()

    UPROPERTY()
    FString AssetPath;

    UPROPERTY()
    FTransform TargetTransform = FTransform::Identity;

    bool IsValid() const
    {
        return !AssetPath.IsEmpty();
    }

    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("- AssetPath: %s"), *AssetPath);
        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("- TargetTransform: %s"), *TargetTransform.ToString());
    }
};

/**
 * @brief Async job info (task, job name, latest capture batch id)
 * This struct contains TFuture, which has private ctor, thus is move-only and could not be a USTRUCT().
 * and also make itself by default non-copyable without custom copy ctor!
 */
struct RAPYUTASIMULATIONPLUGINS_API FRRAsyncJob
{
    template<typename TResult>
    struct RAPYUTASIMULATIONPLUGINS_API FRRSingleAsyncTask
    {
        //! [TSharedFuture] should be considered if [Task] is accessed from multiple threads!
        TFuture<TResult> Task;

        bool DoneStatus = false;

        FRRSingleAsyncTask()
        {
        }

        //! TFuture is move-only, only exposing a move-ctor
        FRRSingleAsyncTask(TFuture<TResult>&& InAsyncTask) : Task(MoveTemp(InAsyncTask))
        {
        }

        //! TFuture is move-only, thus this is to facilitate this class' Move operation!
        FRRSingleAsyncTask(FRRSingleAsyncTask&& Other)
        {
            *this = MoveTemp(Other);
        }

        FRRSingleAsyncTask& operator=(FRRSingleAsyncTask&& Other)
        {
            Task = MoveTemp(Other.Task);
            Other.Task.Reset();

            DoneStatus = Other.DoneStatus;
            Other.DoneStatus = false;
            return *this;
        }
    };

    explicit FRRAsyncJob(const FString& InJobName) : JobName(InJobName)
    {
    }
    FRRAsyncJob(FRRAsyncJob&& Other)
    {
        verify(AsyncTasks.Num() == Other.AsyncTasks.Num());
        for (int32 i = 0; i < AsyncTasks.Num(); ++i)
        {
            // This [MoveTemp] would help invoke [FRRSingleAsyncTask] move-ctor!
            AsyncTasks[i] = MoveTemp(Other.AsyncTasks[i]);
        }
        Other.AsyncTasks.Empty();
    }

    static constexpr const int8 TASK_INDEX_NONE = -1;

    FString JobName;

    //! This stores the up-to-the-moment capture batch id every time an async task is added to be scheduled for running!
    uint64 LatestCaptureBatchId = 0;

    TArray<FRRSingleAsyncTask<bool>> AsyncTasks;

    // For the future of UE C++
    // TFunction<void(ArgTypes&&...Args)> OnTaskDone;

    int32 GetTasksNum() const
    {
        return AsyncTasks.Num();
    }

    //! TFuture is move-only!
    void SetAsyncTaskAtLast(TFuture<bool>&& InAsyncTask)
    {
        verify(AsyncTasks.Num());
        AsyncTasks.Last().Task = MoveTemp(InAsyncTask);
    }

    void AddAsyncTask(const uint64 InCaptureBatchId, TFuture<bool>&& InAsyncTask)
    {
        LatestCaptureBatchId = InCaptureBatchId;
        AsyncTasks.Emplace(MoveTemp(InAsyncTask));
    }

    void AddDefaultAsyncTask()
    {
        AsyncTasks.Emplace(FRRSingleAsyncTask<bool>());
    }

    void Clear()
    {
        AsyncTasks.Reset();
        LatestCaptureBatchId = 0;
    }

    void MarkSingleTaskAsDone(int32 TaskIndex, const FString& TaskName)
    {
        if (TASK_INDEX_NONE == TaskIndex)
        {
            return;
        }

        if (AsyncTasks.IsValidIndex(TaskIndex))
        {
            AsyncTasks[TaskIndex].DoneStatus = true;
#if RAPYUTA_SIM_DEBUG
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("[%s] Async Task done [%d]/[%d]!"), *TaskName, (TaskIndex + 1), GetTasksNum());
#endif
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogTemp, Error, TEXT("[%s] FRRAsyncJob Invalid Async Task Index: %d/%d"), *TaskName, TaskIndex, GetTasksNum());
        }
    }

    bool IsDone() const
    {
        for (const auto& asyncTask : AsyncTasks)
        {
            if (!asyncTask.DoneStatus)
            {
                return false;
            }
        }
        return true;
    }

    void WaitUntilCompleted()
    {
        for (const auto& asyncTask : AsyncTasks)
        {
            asyncTask.Task.Get();
        }
    }
};

/**
 * @brief Group of homogeneous-mesh entities
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRHomoMeshEntityGroup
{
    GENERATED_BODY()
    FRRHomoMeshEntityGroup()
    {
    }

    FRRHomoMeshEntityGroup(TArray<ARRMeshActor*> InEntities) : Entities(MoveTemp(InEntities))
    {
    }

    UPROPERTY()
    TArray<ARRMeshActor*> Entities;

    //! Fetch #Entities as TArray<AActor*>
    TArray<AActor*> GetActors() const
    {
        TArray<AActor*> actors;
        for (auto& entity : Entities)
        {
            actors.Add((AActor*)entity);
        }
        return actors;
    }

    //! Fetch Entities[Index]
    ARRMeshActor* operator[](int32 Index) const
    {
        return Entities[Index];
    }

    //! Get num of Entities
    int32 Num() const
    {
        return Entities.Num();
    }
    //! Get Entities Group's common model name
    FString GetGroupModelName() const;

    //! Get unique name of the entities group
    FString GetGroupName() const;
};

/**
 * @brief For storing entity info in advance for the Logging task.
 * This might have been updated at the moment of Logging.
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRREntityLogInfo
{
    GENERATED_BODY()

    //  NOTE: CustomDepthStencilValue, being used as segmaskid, is inherent to each meshcomp & assign once at creation
    // & kept unchanged thus could be fetched directly from [Entity] at the logging moment
    FRREntityLogInfo()
    {
    }

    FRREntityLogInfo(TArray<AActor*> InEntities,
                     const FString& InGroupModelName,
                     const FString& InGroupName,
                     const FString& InSegMaskDepthStencilStr,
                     const uint64 InSceneId)
        : Entities(MoveTemp(InEntities)),
          GroupModelName(InGroupModelName),
          GroupName(InGroupName),
          SegMaskDepthStencilStr(InSegMaskDepthStencilStr),
          SceneId(InSceneId)
    {
    }

    UPROPERTY()
    TArray<AActor*> Entities;

    //! Common model name of the Entities group
    UPROPERTY()
    FString GroupModelName;

    //! Unique name of the entities group
    UPROPERTY()
    FString GroupName;

    //! Segmentation mask custom depth stencil hexa values in string format
    UPROPERTY()
    FString SegMaskDepthStencilStr;

    //! Id of the current scene entities group belong to
    UPROPERTY()
    uint64 SceneId = 0;

    //! World transform of the whole entities group
    UPROPERTY()
    FTransform WorldTransform = FTransform::Identity;

    //! Bounding box 3D vertices of the whole group in world coordinate
    UPROPERTY()
    TArray<FVector> BBVertices3DInWorld;

    //! Bounding box 3D vertices of the whole group in camera coordinate
    UPROPERTY()
    TArray<FVector> BBVertices3DInCamera;

    //! Bounding box 2D vertices of the whole group, as projected onto camera
    UPROPERTY()
    TArray<FVector2D> BBVertices2D;
};

template<int8 InBitDepth>
using FRRColor = typename std::conditional<(8 == InBitDepth),
        FColor,
        typename std::conditional<(16 == InBitDepth), FFloat16Color, FLinearColor>::type>::type;

// (NOTE) TImagePixelData could be used instead
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRColorArray
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FColor> Colors;

    //! @note Why UE does not allow UPROPERTY() on this??
    TArray<FFloat16Color> Float16Colors;

    UPROPERTY()
    TArray<FLinearColor> Float32Colors;

    template<int8 InBitDepth>
    const TArray<FRRColor<InBitDepth>>& GetImageData() const
    {
        if constexpr (8 == InBitDepth)
        {
            return Colors;
        }
        else if constexpr (16 == InBitDepth)
        {
            return Float16Colors;
        }
        else if constexpr (32 == InBitDepth)
        {
            return Float32Colors;
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Fatal, TEXT("unsupported bit-depth [%d]"), InBitDepth);
        }
    }

    template<int8 InBitDepth>
    TArray<FRRColor<InBitDepth>>& ImageData()
    {
        return *const_cast<TArray<FRRColor<InBitDepth>>*>(&GetImageData<InBitDepth>());
    }

    FORCEINLINE int64 Num(int8 InBitDepth) const
    {
        switch (InBitDepth)
        {
            case 8:
                return GetImageData<8>().Num();
            case 16:
                return GetImageData<16>().Num();
            case 32:
                return GetImageData<32>().Num();
            default:
                UE_LOG_WITH_INFO(LogTemp, Fatal, TEXT("FRRColorArray::Num(): unsupported bit-depth [%d]"), InBitDepth);
                return 0;
        }
    }

    FORCEINLINE bool HasData(int8 InBitDepth) const
    {
        return (Num(InBitDepth) > 0);
    }

    void ToggleAlpha(int8 InBitDepth, bool bAlphaEnabled)
    {
        switch (InBitDepth)
        {
            case 8:
                for (auto& color : ImageData<8>())
                {
                    color.A = bAlphaEnabled ? FColor::Black.A : FColor::Transparent.A;
                }
                break;
            case 16:
                for (auto& color : ImageData<16>())
                {
                    color.A = bAlphaEnabled ? FLinearColor::Black.A : FLinearColor::Transparent.A;
                }
                break;
            case 32:
                for (auto& color : ImageData<32>())
                {
                    color.A = bAlphaEnabled ? FLinearColor::Black.A : FLinearColor::Transparent.A;
                }
                break;
            default:
                UE_LOG_WITH_INFO(LogRapyutaCore, Fatal, TEXT("unsupported bit-depth [%d]"), InBitDepth);
                break;
        }
    }
};

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRActorSpawnInfo
{
    GENERATED_BODY()

    FRRActorSpawnInfo();
    FRRActorSpawnInfo(const FString& InEntityModelName,
                      const FString& InUniqueName,
                      const FTransform& InActorTransform,
                      const TArray<FTransform>& InMeshRelTransformList = TArray<FTransform>(),
                      const TArray<FString>& InMeshUniqueNameList = TArray<FString>(),
                      const TArray<FString>& InMaterialNameList = TArray<FString>(),
                      bool bInIsStationary = false,
                      bool bInIsPhysicsEnabled = false,
                      bool bInIsCollisionEnabled = false,
                      bool bInIsOverlapEventEnabled = false);

    void operator()(const FString& InEntityModelName,
                    const FString& InUniqueName,
                    const FTransform& InActorTransform = FTransform::Identity,
                    const TArray<FTransform>& InMeshRelTransformList = TArray<FTransform>(),
                    const TArray<FString>& InMeshUniqueNameList = TArray<FString>(),
                    const TArray<FString>& InMaterialNameList = TArray<FString>(),
                    bool bInIsStationary = false,
                    bool bInIsPhysicsEnabled = false,
                    bool bInIsCollisionEnabled = false,
                    bool bInIsOverlapEventEnabled = false);

    void ClearMeshInfo()
    {
        MeshUniqueNameList.Reset();
        MeshRelTransformList.Reset();
        MaterialNameList.Reset();
    }

    UPROPERTY()
    FString EntityModelName;

    /**
     * @brief
     * Actually GetName() is also unique as noted by UE, but we just do not want to rely on it.
     * Instead, WE CREATE [UniqueName] TO MAKE OUR ID CONTROL MORE INDPENDENT of UE INTERNAL NAME HANDLING.
     * Reasons: Sometimes,
     * + UE provided Name Id is updated as Label is updated...
     * + In pending-kill state, GetName() goes to [None]
     *
     */
    UPROPERTY()
    FString UniqueName;

    UPROPERTY()
    FTransform ActorTransform = FTransform::Identity;

    UPROPERTY()
    TArray<FTransform> MeshRelTransformList;

    UPROPERTY()
    TArray<FString> MeshUniqueNameList;

    UPROPERTY()
    TArray<FString> MaterialNameList;

    UPROPERTY()
    uint8 bIsTickEnabled : 1;

    UPROPERTY()
    uint8 bIsStationary : 1;

    UPROPERTY()
    uint8 bIsPhysicsEnabled : 1;

    UPROPERTY()
    uint8 bIsCollisionEnabled : 1;

    UPROPERTY()
    uint8 bIsSelfCollision : 1;

    UPROPERTY()
    uint8 bIsOverlapEventEnabled : 1;

    UPROPERTY()
    uint8 bIsDataSynthEntity : 1;

    // Spawn Configuration Info --
    // [MeshComList] now belongs to [ARRMeshActor], only which SpawnSimActor<T> spawns
    UPROPERTY()
    TSubclassOf<AActor> TypeClass = nullptr;

    bool IsValid(bool bIsLogged = false) const
    {
        return (false == EntityModelName.IsEmpty());
    }
};

DECLARE_DELEGATE_TwoParams(FOnMeshActorFullyCreated, bool /* bCreationResult */, ARRMeshActor*);

/**
 * @brief Scene instance's common object which houses Plugin-specific dynamic properties and implement objects-related API (Spawning, teleporting, etc.)
 * If USE_SCENE_DIRECTOR is disabled, there is only one single ActorCommon being shared among all RRBaseActor.
 * @note Mostly responsible for holding handles to #URRSceneInstance  (Main environment, camera, etc.)
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API URRActorCommon : public UObject
{
    GENERATED_BODY()
    //! !! [ActorCommon] OBJECT COULD ONLY BE ACCESSED FROM BEGINPLAY(),
    //! THUS SPECIFC SIM START-UP RELATED CONTENTS & HANDLINGS, WHICH HAPPENS BEFORE BEGINPLAY(), SHOULD BE MOVED TO GameMode or
    //! GameInstance.
    friend class URRCoreUtils;

private:
    static TMap<int8, URRActorCommon*> SActorCommonList;

public:
    static constexpr int8 DEFAULT_SCENE_INSTANCE_ID = 0;
    static URRActorCommon* GetActorCommon(int8 InSceneInstanceId = URRActorCommon::DEFAULT_SCENE_INSTANCE_ID,
                                          UClass* ActorCommonClass = nullptr,
                                          UObject* Outer = nullptr);
    URRActorCommon();
    static constexpr const TCHAR* SCRIPT_INI_PATH = TEXT("/Script/RapyutaSimulationPlugins.RRActorCommon");
    static std::once_flag OnceFlag;

    /**
     * @brief Callback for world cleanup end
     * @sa [Ref](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/FWorldDelegates/OnPostWorldCleanup)
     * @param InWorld
     * @param bInSessionEnded Whether to notify the viewport that the game session (PIE, Game, etc.) has ended.
     * @param bInCleanupResources Whether resources should be cleaned up
     */
    static void OnPostWorldCleanup(UWorld* InWorld, bool /*bInSessionEnded*/, bool /*bInCleanupResources*/);

public:
    static constexpr const TCHAR* SPACE_STR = TEXT(" ");
    static constexpr const TCHAR* DELIMITER_STR = TEXT(",");
    static constexpr const TCHAR* UNDERSCORE_STR = TEXT("_");
    static constexpr const char* NAN_STR = "NaN";
    static const std::string QUOTED_NAN_STR;
    static constexpr const char* NONE_STR = "None";    // or NAME_None ?
    static constexpr const char* NULL_STR = "Null";
    static const std::string QUOTED_NULL_STR;
    static constexpr const char* DOUBLE_QUOTE_CHAR = "\"";

    static constexpr int32 SIM_SCENE_COMPLETION_TIMEOUT_SECS = 10;

    static constexpr int8 IMAGE_BIT_DEPTH_INT8 = 8;
    static constexpr int8 IMAGE_BIT_DEPTH_FLOAT16 = 16;
    static constexpr int8 IMAGE_BIT_DEPTH_FLOAT32 = 32;

    static constexpr const TCHAR* MAP_ORIGIN_TAG = TEXT("map_origin");
    static constexpr const TCHAR* MAP_ROS_FRAME_ID = TEXT("map");

    //! Each scene instance house a series of scene, in which operations are performed.
    UPROPERTY()
    int8 SceneInstanceId = DEFAULT_SCENE_INSTANCE_ID;

    //! Location of the current scene instance this ActorCommon belongs to
    UPROPERTY()
    FVector SceneInstanceLocation = FVector::ZeroVector;

    //! Generate a new scene id
    //! Each scene is assigned with a unique id, regardless of which scene instance it belongs
    static uint64 SLatestSceneId;
    FORCEINLINE static uint64 GetNextSceneId()
    {
        return ++SLatestSceneId;
    }

    //! Current scene id
    UPROPERTY()
    uint64 CurrentSceneId = 0;

    //! Main environment actor if pre-setup in the level
    UPROPERTY()
    AActor* SceneFloor = nullptr;

    //! Main floor actor if pre-setup in the level
    //! Main wall actor if pre-setup in the level
    UPROPERTY()
    AActor* SceneWall = nullptr;

    //! Main camera actor used in the scene instance
    UPROPERTY()
    ARRCamera* SceneCamera = nullptr;

    UPROPERTY()
    TArray<ALight*> MainLights;

    //! Print sim config vars in INI
    virtual void PrintSimConfig() const;

    //! Callback on ARRGameState::StartSim() for this scene instance
    virtual void OnStartSim();

    //! Callback on ARRGameState::BeginPlay() for this scene instance
    virtual void OnBeginPlay()
    {
    }

    //! Callback on ARRGameState::EndPlay() for this scene instance
    virtual void OnEndPlay(const EEndPlayReason::Type EndPlayReason)
    {
    }

    /**
     * @brief Whether this scene instance's common object has been initialized
     * @param bIsLogged Whether to log on uninitialized contents
     */
    virtual bool HasInitialized(bool bIsLogged = false) const;
    //! Setup the scene instance's environment
    virtual void SetupEnvironment();
    //! Move the common main environment to another scene instance

    //! Callback on a mesh actor being fully created in the scene
    FOnMeshActorFullyCreated OnMeshActorFullyCreated;

    // NOTE:
    // + Currently UE only supports [0-255] CustomDepthStencil values,
    // which might probably be extended to a larger range then.
    // + Since deactivated actors do not appear in scene so their custom depth stencil values should be reused
    // for activated ones in any scene instance.
    // + Also, actors of different scene instances, due to always appearing in different scenes, could share the same value,
    // thus each scene instance must have its own [LatestCustomDepthStencilValue]

    // Temp as No of non-zero elements in [DATA_SYNTH_CUSTOM_DEPTH_STENCILS], due to UE5 segmask mismatch issue
    static constexpr int16 MAX_CUSTOM_DEPTH_STENCIL_VALUES_NUM = 76;
    static constexpr int8 DEFAULT_CUSTOM_DEPTH_STENCIL_VALUE_VOID = 0;

    //! This list contains Static Objects' [CustomDepthStencilValue]s that are assigned once and never change during Sim run!
    //! Example: Bucket's (DropPlatform is also a static object but its custom depth render is disabled due to not being segmasked)
    static TArray<int32> StaticCustomDepthStencilList;

    UPROPERTY()
    int32 LatestCustomDepthStencilValue = 0;

    //! Generate a new unique custom depth stencil value for a mesh actor's Segmentation mask
    int32 GenerateUniqueDepthStencilValue()
    {
        if (LatestCustomDepthStencilValue > MAX_CUSTOM_DEPTH_STENCIL_VALUES_NUM)
        {
            UE_LOG_WITH_INFO(LogTemp,
                             Error,
                             TEXT("SceneInstance[%d] [%d] More than %d CustomDepthStencil values having been assigned!"
                                  "Segmentation Mask will be duplicated!"),
                             SceneInstanceId,
                             LatestCustomDepthStencilValue,
                             MAX_CUSTOM_DEPTH_STENCIL_VALUES_NUM);
        }

        // Fetch the next non-static [CustomDepthStencilValue]
        do
        {
            ++LatestCustomDepthStencilValue;
        } while (StaticCustomDepthStencilList.Contains(LatestCustomDepthStencilValue));
        return LatestCustomDepthStencilValue;
    }
};

class ARRSceneDirector;
class ARRPlayerController;

/**
 * @brief Scene Instance, Eg: #ARRSceneDirector,  a scene unit among many residing in the same level.
 * This provide a feature to split the level into multiple scenes, each of which can be used for different purposes.
 * Each scene instance contains:
 * - Scene director: Init/Run/Continue Sim type-specific operations (Data synthesizer or Robot operations, etc. or compound)
 * - Sim common object: Eg: #URRActorCommon, which houses Plugin-specific dynamic properties and implement objects-related API (Spawning, teleporting, etc.)
 * - Sim player controller: Eg: #ARRPlayerController
 * - Each of those is accompanied by TSubclassOf<T>, allowing child Scene Instance class to
specify custom functional Types
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRSceneInstance : public UObject
{
    GENERATED_BODY()
public:
    virtual void ConfigureStaticClasses();

    virtual bool IsValid(bool bIsLogged = false) const;

    UPROPERTY()
    ARRPlayerController* PlayerController = nullptr;

    // Sim Common --
    UPROPERTY()
    TSubclassOf<URRActorCommon> ActorCommonClass;

    UPROPERTY()
    URRActorCommon* ActorCommon = nullptr;

    // Scene Director --
    UPROPERTY()
    TSubclassOf<ARRSceneDirector> SceneDirectorClass;

    UPROPERTY()
    ARRSceneDirector* SceneDirector = nullptr;
};
