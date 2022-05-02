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

#define RAPYUTA_DATA_SYNTH_USE_ENTITY_STATIC_MESH (0)
#define RAPYUTA_DATA_SYNTH_USE_ENTITY_PROCEDURAL_MESH (!RAPYUTA_DATA_SYNTH_USE_ENTITY_STATIC_MESH)

class ARRGameMode;
class ARRGameState;
class ARRMeshActor;
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
    YAML,

    // Image
    IMAGE_JPG,
    IMAGE_PNG,
    IMAGE_EXR,
    IMAGE_HDR,

    // 3D Description Format
    URDF,
    SDF,
    GAZEBO_WORLD,
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
    CAPSULE,
    MESH,
    TOTAL
};

// SIM ACTOR SPAWN INFO --
//
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
                      bool bInIsCollisionEnabled = false);

    void operator()(const FString& InEntityModelName,
                    const FString& InUniqueName,
                    const FTransform& InActorTransform = FTransform::Identity,
                    const TArray<FTransform>& InMeshRelTransformList = TArray<FTransform>(),
                    const TArray<FString>& InMeshUniqueNameList = TArray<FString>(),
                    const TArray<FString>& InMaterialNameList = TArray<FString>(),
                    bool bInIsStationary = false,
                    bool bInIsPhysicsEnabled = false,
                    bool bInIsCollisionEnabled = false);

    void ClearMeshInfo()
    {
        MeshUniqueNameList.Reset();
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

    UPROPERTY(EditAnywhere, Category = "Info")
    uint8 bIsSelfCollision : 1;

    // Spawn Configuration Info --
    // [MeshComList] now belongs to [ARRMeshActor], only which SpawnSimActor<T> spawns
    UPROPERTY()
    TSubclassOf<AActor> TypeClass = nullptr;

    bool IsValid(bool bIsLogged = false) const
    {
        return (false == EntityModelName.IsEmpty());
    }
};

DECLARE_DELEGATE_ThreeParams(FOnMeshActorFullyCreated,
                             bool /* bCreationResult */,
                             ARRMeshActor*,
                             const FString& /* Actor's EntityModelName*/);

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

    static void OnPostWorldCleanup(UWorld* World, bool bSessionEnded, bool bCleanupResources);

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

    UPROPERTY()
    AActor* MainEnvironment = nullptr;

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
    void MoveEnvironmentToSceneInstance(int8 InSceneInstanceId);

    FOnMeshActorFullyCreated OnMeshActorFullyCreated;

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
