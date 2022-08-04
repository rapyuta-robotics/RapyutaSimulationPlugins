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

// UE
#include "Engine.h"
#include "Math/Color.h"
#include "Misc/Paths.h"

// RapyutaSimulationPlugins
#include "RapyutaSimulationPlugins.h"

#include "RRActorCommon.generated.h"

#define RAPYUTA_SIM_DEBUG (0)
#define RAPYUTA_SIM_VISUAL_DEBUG (0)

#define RAPYUTA_DATA_SYNTH_USE_ENTITY_STATIC_MESH (0)
#define RAPYUTA_DATA_SYNTH_USE_ENTITY_PROCEDURAL_MESH (!RAPYUTA_DATA_SYNTH_USE_ENTITY_STATIC_MESH)

class ARRGameMode;
class ARRGameState;
class ARRMeshActor;
class ARRMeshActor;
class ARRCamera;
class URRCoreUtils;

/**
 * @brief todo
 * @todo should avoid hardcoded versioning in C++ file.
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRapyutaSimVersionInfo
{
    GENERATED_BODY()

    // Major version for output data format changes
    UPROPERTY()
    int32 Major = 0;

    // Minor version for backwards-compatible improvements
    UPROPERTY()
    int32 Minor = 0;

    // Bugfix versions
    UPROPERTY()
    int32 BugFix = 0;

    FString ToString() const
    {
        return FString::Printf(TEXT("%d.%d.%d"), Major, Minor, BugFix);
    }
};

static const FRapyutaSimVersionInfo RAPYUTA_SIM_VERSION_INFO({0, 0, 1});

/**
 * @brief todo
 *
 */
UENUM()
enum class ERRFileType : uint8
{
    NONE,
    UASSET,    // UE Asset file
    INI,
    YAML,

    // Image
    IMAGE_JPG,
    IMAGE_GRAYSCALE_JPG,
    IMAGE_PNG,
    IMAGE_TGA,
    IMAGE_EXR,
    IMAGE_HDR,

    // Light Profile
    LIGHT_PROFILE_IES,

    // Meta Data
    JSON,

    // 3D Description Format
    URDF,
    SDF,
    GAZEBO_WORLD,
    MJCF,    // MuJoCo

    // 3D CAD
    CAD_FBX,
    CAD_OBJ,
    CAD_STL,
    CAD_DAE,
    TOTAL
};

/**
 * @brief todo
 *
 */
UENUM()
enum class ERRShapeType : uint8
{
    INVALID,
    PLANE,
    BOX,
    CYLINDER,
    SPHERE,
    CAPSULE,
    MESH,
    TOTAL
};
/**
 * @brief todo
 *
 */
// This struct contains TFuture, which has private ctor, thus is move-only and could not be a USTRUCT().
// and also make itself by default non-copyable without custom copy ctor!
struct RAPYUTASIMULATIONPLUGINS_API FRRAsyncJob
{
    template<typename TResult>
    struct RAPYUTASIMULATIONPLUGINS_API FRRSingleAsyncTask
    {
        // [TSharedFuture] should be considered if [Task] is accessed from multiple threads!
        TFuture<TResult> Task;
        bool DoneStatus = false;

        FRRSingleAsyncTask()
        {
        }
        // TFuture is move-only, only exposing a move-ctor
        FRRSingleAsyncTask(TFuture<TResult>&& InAsyncTask) : Task(MoveTemp(InAsyncTask))
        {
        }

        // TFuture is move-only, thus this is to facilitate this class' Move operation!
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
    // This stores the up-to-the-moment capture batch id every time an async task is added to be scheduled for running!
    uint64 LatestCaptureBatchId = 0;
    TArray<FRRSingleAsyncTask<bool>> AsyncTasks;

    // For the future of UE C++
    // TFunction<void(ArgTypes&&...Args)> OnTaskDone;

    int32 GetTasksNum() const
    {
        return AsyncTasks.Num();
    }

    // TFuture is move-only!
    void SetAsyncTaskAtLast(TFuture<bool>&& InAsyncTask)
    {
        verify(AsyncTasks.Num());
        AsyncTasks.Last().Task = MoveTemp(InAsyncTask);
    }

    void AddAsyncTask(const uint64& InCaptureBatchId, TFuture<bool>&& InAsyncTask)
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
            UE_LOG(LogTemp, Display, TEXT("[%s] Async Task done [%d]/[%d]!"), *TaskName, (TaskIndex + 1), GetTasksNum());
#endif
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[%s] FRRAsyncJob Invalid Async Task Index: %d/%d"), *TaskName, TaskIndex, GetTasksNum());
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

// For storing entity info in advance for the Logging task. This might have been updated at the moment of Logging.
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRREntityLogInfo
{
    GENERATED_BODY()

    FRREntityLogInfo()
    {
    }

    FRREntityLogInfo(AActor* InEntity, const uint64& InSceneId) : Entity(InEntity), SceneId(InSceneId)
    {
    }

    UPROPERTY()
    AActor* Entity = nullptr;

    UPROPERTY()
    uint64 SceneId = 0;
};

template<int8 InBitDepth>
using FRRColor = typename TChooseClass<(8 == InBitDepth),
                                       FColor,
                                       typename TChooseClass<(16 == InBitDepth), FFloat16Color, FLinearColor>::Result>::Result;

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRColorArray
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FColor> Colors;

    // Why UE does not allow UPROPERTY() on this??
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
            UE_LOG(LogRapyutaCore, Fatal, TEXT("FRRColorArray::GetImageData(): unsupported bit-depth [%d]"), InBitDepth);
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
                UE_LOG(LogTemp, Fatal, TEXT("FRRColorArray::Num(): unsupported bit-depth [%d]"), InBitDepth);
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
                UE_LOG(LogRapyutaCore, Fatal, TEXT("FRRColorArray::ToggleAlpha(): unsupported bit-depth [%d]"), InBitDepth);
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

    // Actually GetName() is also unique as noted by UE, but we just do not want to rely on it.
    // Instead, WE CREATE [UniqueName] TO MAKE OUR ID CONTROL MORE INDPENDENT of UE INTERNAL NAME HANDLING.
    // Reasons: Sometimes,
    // + UE provided Name Id is updated as Label is updated...
    // + In pending-kill state, GetName() goes to [None]
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
 * @brief Scene instance's common object
 * @note Mostly responsible for holding handles to common scene objects (Main environment, camera, etc.)
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API URRActorCommon : public UObject
{
    GENERATED_BODY()
    // !! [ActorCommon] OBJECT COULD ONLY BE ACCESSED FROM BEGINPLAY(),
    // THUS SPECIFC SIM START-UP RELATED CONTENTS & HANDLINGS, WHICH HAPPENS BEFORE BEGINPLAY(), SHOULD BE MOVED TO GameMode or
    // GameInstance.
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
     * @note [Ref](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/FWorldDelegates/OnPostWorldCleanup)
     * @param InWorld
     * @param bInSessionEnded Whether to notify the viewport that the game session (PIE, Game, etc.) has ended.
     * @param bInCleanupResources Whether resources should be cleaned up
     */
    static void OnPostWorldCleanup(UWorld* InWorld, bool /*bInSessionEnded*/, bool /*bInCleanupResources*/);

public:
#define EMPTY_STR (TEXT(""))    // Using TCHAR* = TEXT("") -> could causes linking error in some case!
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

    //! Game mode handle
    UPROPERTY()
    ARRGameMode* GameMode = nullptr;

    //! Game state handle
    UPROPERTY()
    ARRGameState* GameState = nullptr;

    //! Each scene instance house a series of scene, in which operations are performed.
    UPROPERTY()
    int8 SceneInstanceId = DEFAULT_SCENE_INSTANCE_ID;

    //! Location of the current scene instance this ActorCommon belongs to
    UPROPERTY()
    FVector SceneInstanceLocation = FVector::ZeroVector;

    //! Generate a new scene id
    // Each scene is assigned with a unique id, regardless of which scene instance it belongs
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
    AActor* MainEnvironment = nullptr;

    //! Main floor actor if pre-setup in the level
    UPROPERTY()
    AActor* MainFloor = nullptr;

    //! Main wall actor if pre-setup in the level
    UPROPERTY()
    AActor* MainWall = nullptr;

    //! Main camera actor used in the scene instance
    UPROPERTY()
    ARRCamera* MainCamera = nullptr;

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

    //! Callback on ARRGameState::Tick() for this scene instance
    virtual void OnTick(float DeltaTime)
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
    void MoveEnvironmentToSceneInstance(int8 InSceneInstanceId);

    //! Callback on a mesh actor being fully created in the scene
    FOnMeshActorFullyCreated OnMeshActorFullyCreated;

    //! (NOTE) Currently UE only supports [0-255] CustomDepthStencil values,
    //! which might probably be extended to a larger range then.
    UPROPERTY()
    int32 LatestCustomDepthStencilValue = 0;

    //! Generate a new unique custom depth stencil value for a mesh actor's Segmentation mask
    int32 GenerateUniqueDepthStencilValue()
    {
        if (255 <= LatestCustomDepthStencilValue)
        {
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("More than 255 CustomDepthStencil values having been assigned!"
                        "Segmentation Mask will be duplicated!"));
        }
        return ++LatestCustomDepthStencilValue;
    }
};

class ARRSceneDirector;
class ARRPlayerController;

/**
 * @brief Scene Instance, a scene unit among many residing in the same level
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
