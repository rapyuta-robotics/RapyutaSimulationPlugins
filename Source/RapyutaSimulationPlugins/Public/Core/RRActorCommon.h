// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

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

class URRStaticMeshComponent;
class ARRGameMode;
class ARRGameState;
class URRCoreUtils;

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

UENUM()
enum class ERRFileType : uint8
{
    NONE,
    UASSET,    // UE Asset file
    INI,
    IMAGE_JPG,
    IMAGE_PNG,
    IMAGE_EXR,

    // 3D Description Format
    URDF,
    SDF,
    GAZEBO_WORLD,
    GAZEBO_YAML,
    MJCF,    // MuJoCo
    TOTAL
};

UENUM()
enum class ERRShapeType : uint8
{
    INVALID,
    PLANE,
    BOX,
    CYLINDER,
    SPHERE,
    MESH,
    TOTAL
};

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
            UE_LOG(LogRapyutaCore, Display, TEXT("[%s] Async Task done [%d]/[%d]!"), *TaskName, (TaskIndex + 1), GetTasksNum());
#endif
        }
        else
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("[%s] Invalid Async Task Index: %d/%d"), *TaskName, TaskIndex, GetTasksNum());
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
                UE_LOG(LogRapyutaCore, Fatal, TEXT("FRRColorArray::Num(): unsupported bit-depth [%d]"), InBitDepth);
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

// SIM ACTOR SPAWN INFO --
//
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRActorSpawnInfo
{
    GENERATED_BODY()

    FRRActorSpawnInfo()
    {
    }

    FRRActorSpawnInfo(const FString& InUniqueName,
                      const FTransform& InTransform,
                      const TArray<FString>& InMeshMaterialNameList = TArray<FString>(),
                      bool bIsStationary = false,
                      bool bIsPhysicsEnabled = false,
                      bool bIsCollisionEnabled = false);

    FRRActorSpawnInfo(const FString& InUniqueName,
                      const FTransform& InTransform,
                      const FString& InMeshUniqueName,
                      const TArray<FString>& InMeshMaterialNameList = TArray<FString>(),
                      bool bIsStationary = false,
                      bool bIsPhysicsEnabled = false,
                      bool bIsCollisionEnabled = false);

    FRRActorSpawnInfo(const FString& InUniqueName,
                      const FTransform& InTransform,
                      const TArray<FString>& InMeshUniqueNameList,
                      const TArray<FString>& InMeshMaterialNameList = TArray<FString>(),
                      bool bIsStationary = false,
                      bool bIsPhysicsEnabled = false,
                      bool bIsCollisionEnabled = false);

    void operator()(const FString& InUniqueName,
                    const FTransform& InTransform = FTransform::Identity,
                    const FString& InMeshUniqueName = FString(),
                    const TArray<FString>& InMeshMaterialNameList = TArray<FString>(),
                    bool bIsStationary = false,
                    bool bIsPhysicsEnabled = false,
                    bool bIsCollisionEnabled = false);

    void operator()(const FString& InMeshUniqueName);
    void ClearMeshInfo()
    {
        MeshUniqueNameList.Reset();
        MeshMaterialNameList.Reset();
    }

    // Actually GetName() is also unique as noted by UE, but we just do not want to rely on it.
    // Instead, WE CREATE [UniqueName] TO MAKE OUR ID CONTROL MORE INDPENDENT of UE INTERNAL NAME HANDLING.
    // Reasons: Sometimes,
    // + UE provided Name Id is updated as Label is updated...
    // + In pending-kill state, GetName() goes to [None]
    UPROPERTY()
    FString UniqueName;

    UPROPERTY()
    FTransform Transform;

    UPROPERTY()
    TArray<FString> MeshUniqueNameList;

    UPROPERTY()
    TArray<FString> MeshMaterialNameList;

    UPROPERTY()
    bool IsTickEnabled = false;

    UPROPERTY()
    bool IsStationary = false;

    UPROPERTY()
    bool IsPhysicsEnabled = true;

    UPROPERTY()
    bool IsCollisionEnabled = true;

    UPROPERTY(EditAnywhere, Category = "Info")
    bool IsSelfCollision = false;

    // Spawn Configuration Info --
    // [MeshComList] now belongs to [ARRMeshActor], only which SpawnSimActor<T> spawns
    UPROPERTY()
    TSubclassOf<AActor> TypeClass = nullptr;

    bool IsValid(bool bIsLogged = false) const
    {
        return true;
    }
};

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
    static constexpr const TCHAR* SCRIPT_INI_PATH = TEXT("/Script/RapyutaSim.RRActorCommon");
    static std::once_flag OnceFlag;

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

    UPROPERTY()
    ARRGameMode* GameMode = nullptr;

    UPROPERTY()
    ARRGameState* GameState = nullptr;

    UPROPERTY()
    int8 SceneInstanceId = DEFAULT_SCENE_INSTANCE_ID;

    UPROPERTY()
    FVector SceneInstanceLocation = FVector::ZeroVector;

    virtual void PrintSimConfig() const;

    virtual void OnStartSim();
    virtual void OnBeginPlay()
    {
    }

    virtual void OnEndPlay(const EEndPlayReason::Type EndPlayReason)
    {
    }

    virtual void OnTick(float DeltaTime)
    {
    }

    virtual bool HasInitialized(bool bIsLogged = false) const;
    virtual void SetupEnvironment();

    UPROPERTY()
    TArray<AActor*> SceneLoggedActors;

    UFUNCTION()
    void ClearScene()
    {
    }

    // UE only support custom depth stencil value in range [0-255]
    UFUNCTION()
    static uint8 GenerateUniqueDepthStencilValue()
    {
        static uint8 sLatestCustomDepthStencilValue = 0;
        if (255 <= sLatestCustomDepthStencilValue)
        {
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("There are more than 255 CustomDepthStencil values having been assigned!"
                        "Depth Segmentation Mask info would be duplicated!"));
        }
        return ++sLatestCustomDepthStencilValue;
    }
};

class ARRSceneDirector;
class ARRPlayerController;
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
