/**
 * @file RRCoreUtils.h
 * @brief Core utils.
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// Native
#include <string>

// UE
#if WITH_EDITOR
#include "Editor.h"
#include "Subsystems/UnrealEditorSubsystem.h"
#include "UnrealEd.h"
#endif
#include "Delegates/Delegate.h"
#include "Engine/LatentActionManager.h"
#include "Engine/LevelStreaming.h"
#include "Engine/TextureLightProfile.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "ImageUtils.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Guid.h"
#include "Misc/MonitoredProcess.h"
#include "Stats/Stats.h"
#include "UObject/Object.h"
#include "UObject/ObjectMacros.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRGeneralUtils.h"
#include "Core/RRTextureData.h"
#include "Core/RRTypeUtils.h"

#include "RRCoreUtils.generated.h"

class ARRGameState;
class URRGameInstance;
class URRStaticMeshComponent;
class ARRBaseActor;
class UCameraComponent;

/**
 * @brief Core utils
 * @todo add documentation
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRCoreUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    // -------------------------------------------------------------------------------------------------------------
    // GENERAL UTILS ===============================================================================================
    //
    template<class T, std::size_t N>
    constexpr std::size_t static GetArraySize(const T (&)[N])
    {
        return N;
    }

    template<bool bInBoolPreprocessor>
    FORCEINLINE static constexpr const TCHAR* GetBoolPreprocessorText()
    {
        if constexpr (bInBoolPreprocessor)
        {
            return TEXT("TRUE");
        }
        else
        {
            return TEXT("FALSE");
        }
    }

    // TYPE-RELATED UTILS --
    //
    // STRING UTILS --
    FORCEINLINE static FString GetSanitizedXMLString(const FString& InXMLString)
    {
        FString outXMLString = InXMLString;
        // Remove linebreaks and tabs
        outXMLString.TrimStartAndEndInline();
        outXMLString = outXMLString.Replace(TEXT("\n"), URRActorCommon::SPACE_STR);
        outXMLString = outXMLString.Replace(TEXT("\r"), EMPTY_STR);
        outXMLString = outXMLString.Replace(TEXT("\t"), URRActorCommon::SPACE_STR);
        return outXMLString;
    }

    FORCEINLINE static FString StdToFString(const std::string& InStdString)
    {
        return FString(InStdString.c_str());
    }

    FORCEINLINE static std::string FToStdString(const FString& InUEString)
    {
        return std::string(TCHAR_TO_UTF8(*InUEString));
    }

    // SIM CONSOLE COMMANDS --
    static constexpr const TCHAR* CMD_SIM_QUIT = TEXT("quit");
    static constexpr const TCHAR* CMD_STATS_START = TEXT("stat startfile");
    static constexpr const TCHAR* CMD_STATS_STOP = TEXT("stat stopfile");
    static constexpr const TCHAR* CMD_MEMORY_REPORT = TEXT("memreport");
    static constexpr const TCHAR* CMD_MEMORY_REPORT_FULL = TEXT("memreport -full");
    static constexpr const TCHAR* CMD_GC_DUMP_POOL_STATS = TEXT("gc.DumpPoolStats");
    static constexpr const TCHAR* CMD_RHI_GPU_CAPTURE_OPTIONS_ENABLE = TEXT("r.RHISetGPUCaptureOptions 1");
    static constexpr const TCHAR* CMD_RENDER_DOC_CAPTURE_FRAME = TEXT("renderdoc.CaptureFrame");
    static constexpr const TCHAR* CMD_SHADOW_MAP_CACHING_TURN_OFF = TEXT("r.Shadow.CacheWholeSceneShadows 0");
    static constexpr const TCHAR* CMD_AO_USE_HISTORY_DISABLE = TEXT("r.AOUseHistory 0");
    static constexpr const TCHAR* CMD_OCCLUDED_PRIMITIVES_VISUALIZE = TEXT("r.VisualizeOccludedPrimitives 1");
    static constexpr const TCHAR* CMD_CUSTOM_DEPTH_STENCIL_ENABLE = TEXT("r.CustomDepth 3");
    // https://docs.unrealengine.com/4.26/en-US/TestingAndOptimization/PerformanceAndProfiling/ForwardRenderer
    static constexpr const TCHAR* CMD_FORWARD_SHADING_ENABLE = TEXT("r.ForwardShading 1");
    static constexpr const TCHAR* CMD_HIGHRES_SCREENSHOT_DELAY = TEXT("r.HighResScreenshotDelay");
    static constexpr const TCHAR* CMD_AUDIO_MIXER_DISABLE = TEXT("au.IsUsingAudioMixer 0");

    // SIM FILE EXTENSIONS --

    //! Sim file extentions. Corresponds to #ERRFileType.
    static constexpr const TCHAR* SimFileExts[] = {
        TEXT(""),    // ERRFileType::NONE
        // UE & General
        TEXT(".uasset"),    // ERRFileType::UASSET
        TEXT(".pak"),       // ERRFileType::PAK
        TEXT(".ini"),       // ERRFileType::INI
        TEXT(".yaml"),      // ERRFileType::YAML
        TEXT(".zip"),       // ERRFileType::ZIP

        // Image
        TEXT(".jpg"),    // ERRFileType::IMAGE_JPG
        TEXT(".jpg"),    // ERRFileType::IMAGE_GRAYSCALE_JPG
        TEXT(".png"),    // ERRFileType::IMAGE_PNG
        TEXT(".tga"),    // ERRFileType::IMAGE_TGA
        TEXT(".exr"),    // ERRFileType::IMAGE_EXR
        TEXT(".hdr"),    // ERRFileType::IMAGE_HDR

        // Light Profile
        TEXT(".ies"),    // ERRFileType::LIGHT_PROFILE_IES

        // Meta data
        TEXT(".json"),    // ERRFileType::JSON

        // 3D Description formats
        TEXT(".urdf"),     // ERRFileType::URDF
        TEXT(".sdf"),      // ERRFileType::SDF
        TEXT(".world"),    // ERRFileType::GAZEBO_WORLD
        TEXT(".mjcf"),     // ERRFileType::MJCF

        // 3D CAD
        TEXT(".fbx"),    // ERRFileType::CAD_FBX
        TEXT(".obj"),    // ERRFileType::CAD_OBJ
        TEXT(".stl"),    // ERRFileType::CAD_STL
        TEXT(".dae"),    // ERRFileType::CAD_DAE
    };

    /**
     * @brief Return the file extension for the given file type from #SimFileExts.
     *
     * @param InFileType
     * @return FORCEINLINE const* TCHAR*
     */
    FORCEINLINE static const TCHAR* GetSimFileExt(const ERRFileType InFileType)
    {
        const uint8 fileTypeIdx = static_cast<uint8>(InFileType);
        verify((fileTypeIdx >= static_cast<uint8>(ERRFileType::NONE)) && (fileTypeIdx < static_cast<uint8>(ERRFileType::TOTAL)));
        return SimFileExts[fileTypeIdx];
    }
    static FString GetFileTypeFilter(const ERRFileType InFileType);

    FORCEINLINE static ERRFileType GetFileType(const FString& InFilePath)
    {
        for (uint8 i = 0; i < static_cast<uint8>(ERRFileType::TOTAL); ++i)
        {
            const ERRFileType& fileType = static_cast<ERRFileType>(i);
            if (InFilePath.EndsWith(URRCoreUtils::GetSimFileExt(fileType)))
            {
                return fileType;
            }
        }
        return ERRFileType::NONE;
    }

    FORCEINLINE static bool IsFileType(const FString& InFilePath, const TArray<ERRFileType>& InFileTypes)
    {
        for (const auto& fileType : InFileTypes)
        {
            if (InFilePath.EndsWith(URRCoreUtils::GetSimFileExt(fileType)))
            {
                return true;
            }
        }
        return false;
    }

// SIM COMMAND LINE EXECUTION --
#define CCMDLINE_ARG_FORMAT (TEXT("%s="))
    template<typename TCmdlet>
    static bool IsRunningSimCommandlet()
    {
        return IsRunningCommandlet() && GetRunningCommandletClass()->IsChildOf<TCmdlet>();
    }

    static void ExecuteConsoleCommand(const UObject* InContextObject, const FString& InCommandText)
    {
        if (IsRunningCommandlet())
        {
#if WITH_EDITOR
            GEngine->Exec(URRCoreUtils::GetEditorWorld(), *InCommandText);
#endif
        }
        else
        {
            GetPlayerController<APlayerController>(0, InContextObject)->ConsoleCommand(InCommandText);
            // OR: GEngine->Exec(World, *InCommandText);
        }
    }

    // T must be primitive type only!
    template<typename T>
    static FORCEINLINE constexpr bool GetCommandLineArgumentValue(const TCHAR* InArgName, T& OutArgValue, bool bIsLogged = false)
    {
        bool bResult = false;
        if constexpr (std::is_same<T, bool>::value)
        {
            bResult = FParse::Bool(FCommandLine::Get(), *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }
        else
        {
            bResult = FParse::Value(FCommandLine::Get(), *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }

        if (!bResult && bIsLogged)
        {
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("Command line argument not found under the name [%s]"), InArgName);
        }
        return bResult;
    }

    template<typename T>
    static FORCEINLINE constexpr bool ParseCommandLineParams(const FString& InParams,
                                                             const TCHAR* InArgName,
                                                             T& OutArgValue,
                                                             bool bIsLogged = false)
    {
        bool bResult = false;
        if constexpr (std::is_same<T, bool>::value)
        {
            bResult = FParse::Bool(*InParams, *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }
        else
        {
            bResult = FParse::Value(*InParams, *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }

        if (!bResult && bIsLogged)
        {
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[%s] does not contain an argument named [%s]"), *InParams, InArgName);
        }
        return bResult;
    }

    // SIM WORLDS --
    static constexpr const TCHAR* PIXEL_STREAMER_PLAYER_NAME = TEXT("pixelstreamer");
    static UWorld* GetEditorWorld()
    {
        UWorld* editorWorld = nullptr;
#if WITH_EDITOR
        if (UUnrealEditorSubsystem* UnrealEditorSubsystem = GEditor->GetEditorSubsystem<UUnrealEditorSubsystem>())
        {
            // NOTE: This also returns [GEditor->GetEditorWorldContext().World()]
            editorWorld = UnrealEditorSubsystem->GetEditorWorld();
        }
#endif
        return editorWorld;
    }

    static bool IsPIE()
    {
#if WITH_EDITOR
        return GEditor && (GEditor->PlayWorld || GIsPlayInEditorWorld);
#else
        return false;
#endif
    }

    template<typename T>
    static T* GetGameMode(const UObject* InContextObject = nullptr)
    {
        return Cast<T>(InContextObject->GetWorld()->GetAuthGameMode());
    }

    template<typename T>
    static T* GetGameInstance(const UObject* InContextObject = nullptr)
    {
        verify(IsInGameThread());
        return Cast<T>(UGameplayStatics::GetGameInstance(InContextObject));
    }

    template<typename T>
    static T* GetGameState(const UObject* InContextObject = nullptr)
    {
        return Cast<T>(InContextObject->GetWorld()->GetGameState());
    }

    template<typename T>
    static T* GetPlayerController(int8 InSceneInstanceId, const UObject* InContextObject = nullptr)
    {
        verify(IsInGameThread());
        return Cast<T>(UGameplayStatics::GetPlayerControllerFromID(InContextObject, InSceneInstanceId));
    }

    template<typename T>
    static void GetPlayerControllerList(TArray<T>& OutPlayerControllerList, const UObject* InContextObject = nullptr)
    {
        if (UWorld* World = GEngine->GetWorldFromContextObject(InContextObject, EGetWorldErrorMode::LogAndReturnNull))
        {
            for (FConstPlayerControllerIterator Iterator = World->GetPlayerControllerIterator(); Iterator; ++Iterator)
            {
                T* playerController = Cast<T>(Iterator->Get());
                if (playerController)
                {
                    OutPlayerControllerList.Add(playerController);
                }
            }
        }
    }

    template<typename T>
    static T* CreatePlayerController(int32 ControllerId, const UObject* InContextObject)
    {
        return Cast<T>(UGameplayStatics::CreatePlayer(InContextObject, ControllerId, true));
    }

    template<typename T>
    static void CreatePlayerControllerList(TArray<T>& OutPlayerControllerList, int32 InNumOfPlayers, const UObject* InContextObject)
    {
        for (int32 i = 0; i < InNumOfPlayers; i++)
        {
            T* newPlayer = Cast<T>(UGameplayStatics::CreatePlayer(InContextObject, i, true));
            verify(newPlayer);
            OutPlayerControllerList.Add(newPlayer);
        }
    }
    static bool HasPlayerControllerListInitialized(const UObject* InContextObject, bool bIsLogged = false);

    //! This value could be configured in [DefaultEngine.ini]
    static int32 GetMaxSplitscreenPlayers(const UObject* InContextObject);

    template<typename TRRObject>
    static bool IsDefaultSimSceneInstance(TRRObject* InSimObject)
    {
        return (URRActorCommon::DEFAULT_SCENE_INSTANCE_ID == InSimObject->SceneInstanceId);
    }

    static FString GetSimDefaultConfigFileName(const UObject* InContextObject)
    {
        return InContextObject->GetDefaultConfigFilename();
    }

    static FString GetSimConfigFileName(const UObject* InContextObject)
    {
        return InContextObject->GetClass()->GetConfigName();
    }

    static bool IsSimProfiling();

    // GameState & PlayerController should be able to be recognized polymorphically!

    /**
     * @brief Check #ARRGameState is initialized or not.
     *
     * @param InContextObject
     * @param bIsLogged
     * @return true
     * @return false
     */
    static bool HasSimInitialized(const UObject* InContextObject, bool bIsLogged = false);

    /**
     * @brief Get the Scene Instance. Works only with #ARRGameState
     *
     * @param InContextObject
     * @param InSceneInstanceId
     * @return URRSceneInstance*
     */
    static URRSceneInstance* GetSceneInstance(const UObject* InContextObject, int8 InSceneInstanceId);

    static ARRSceneDirector* GetSceneDirector(const UObject* InContextObject, int8 InSceneInstanceId);
    static FVector GetSceneInstanceLocation(int8 InSceneInstanceId);

    static bool HasEnoughDiskSpace(const FString& InPath, uint64 InRequiredMemorySizeInBytes);

    static bool ShutDownSim(const UObject* InContextObject, uint64 InSimCompletionTimeoutInSecs);
    static void ExecuteSimQuitCommand(const UObject* InContextObject);

    static uint32 GetNewGuid()
    {
        return GetTypeHash(FGuid::NewGuid());
    }

    /**
     * @brief Create a Streaming Level object
     * Each level could be streamed into an unique [ULevelStreaming]
     * Refer to ULevelStreamingDynamic::LoadLevelInstance() for creating multiple streaming instances of the same level
     * @sa [ULevelStreamingDynamic](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/ULevelStreamingDynamic/)
     * @param InContextObject
     * @param InLevelInfo
     * @return ULevelStreamingDynamic*
     */
    static ULevelStreamingDynamic* CreateStreamingLevel(const UObject* InContextObject, const FRRStreamingLevelInfo& InLevelInfo)
    {
        bool bSucceeded = false;
        ULevelStreamingDynamic* streamLevel =
            ULevelStreamingDynamic::LoadLevelInstance(InContextObject->GetWorld(),
                                                      InLevelInfo.AssetPath,
                                                      InLevelInfo.TargetTransform.GetTranslation(),
                                                      InLevelInfo.TargetTransform.Rotator(),
                                                      /*out*/ bSucceeded);
        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("%d Streaming level creation from path: %s"), bSucceeded, *InLevelInfo.AssetPath);
        return streamLevel;
    }

    /**
     * @brief LoadStreamLevel
     * @sa [Loading and Unloading Levels using C++](https://docs.unrealengine.com/5.1/en-US/loading-and-unloading-levels-using-cplusplus-in-unreal-engine/)
     *
     * @param InContextObject
     * @param InLevelName
     * @param InTargetObject
     * @param InExecuteFunctionName
     */
    static void StreamLevel(const UObject* InContextObject,
                            const FString& InLevelName,
                            UObject* InTargetObject = nullptr,
                            const FName& InExecuteFunctionName = NAME_None)
    {
        static int32 sLatentActionUUID = 0;
        FLatentActionInfo latentActionInfo;
        latentActionInfo.CallbackTarget = InTargetObject;
        latentActionInfo.ExecutionFunction = InExecuteFunctionName;
        latentActionInfo.UUID = ++sLatentActionUUID;
        latentActionInfo.Linkage = 0;
        UGameplayStatics::LoadStreamLevel(InContextObject, *InLevelName, true, true, latentActionInfo);
    }

    /**
     * @brief UnloadStreamLevel
     * @sa [Loading and Unloading Levels using C++](https://docs.unrealengine.com/5.1/en-US/loading-and-unloading-levels-using-cplusplus-in-unreal-engine/)
     * @param InContextObject
     * @param InLevelName
     */
    static void UnstreamLevel(const UObject* InContextObject, const FName& InLevelName)
    {
        UGameplayStatics::UnloadStreamLevel(InContextObject, InLevelName, FLatentActionInfo(), true);
    }

    static void MoveStreamingLevelBetweenSceneInstances(ULevelStreaming* InStreamingLevel,
                                                        int8 InStartSceneInstanceId,
                                                        int8 InTargetSceneInstanceId)
    {
        if (InStartSceneInstanceId != InTargetSceneInstanceId)
        {
            const FVector startSceneInstanceLoc = GetSceneInstanceLocation(InStartSceneInstanceId);
            const FVector targetSceneInstanceLoc = GetSceneInstanceLocation(InTargetSceneInstanceId);
            InStreamingLevel->GetLoadedLevel()->ApplyWorldOffset(targetSceneInstanceLoc - startSceneInstanceLoc, false);
        }
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // FILE/DIR UTILS --
    //
    static bool LoadFullFilePaths(const FString& FolderPath,
                                  TArray<FString>& OutFilePaths,
                                  const TArray<ERRFileType>& InFileTypes,
                                  const bool bInRecursive = true);

    static bool CreateDirectoryIfNotExisting(const FString& DirPath)
    {
        FString dirFullPath = FPaths::IsRelative(DirPath) ? FPaths::ConvertRelativePathToFull(DirPath) : DirPath;
        if (FPaths::DirectoryExists(dirFullPath))
        {
            return true;
        }
        else
        {
            bool res = FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(*dirFullPath);
            if (!res)
            {
                UE_LOG_WITH_INFO(LogTemp, Error, TEXT("Failed to create dir %s"), *dirFullPath)
            }
            return res;
        }
    }

    static void VerifyDirPathAbsoluteAndExist(const FString& InDirPath)
    {
        if (false == InDirPath.IsEmpty())
        {
            if (FPaths::IsRelative(InDirPath))
            {
                UE_LOG_WITH_INFO(LogTemp, Fatal, TEXT("[%s] directory path is required to be an absolute path."), *InDirPath);
            }
            else if (false == FPaths::DirectoryExists(InDirPath))
            {
                UE_LOG_WITH_INFO(LogTemp, Fatal, TEXT("[%s] directory does not exist."), *InDirPath);
            }
        }
        // else up to various situations, being empty would be acceptable or not.
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // TIMER UTILS --
    //
    // SIM TIME --
    FORCEINLINE static float GetSeconds()
    {
        // `GetWorld()->GetTimeSeconds()` relies on a context object's World and thus is less robust.
        // typedef FUnixTime FPlatformTime, used by:
        // + UEngine::UpdateTimeAndHandleMaxTickRate() to set FApp::CurrentTime
        // + UGameplayStatics::GetAccurateRealTime()
        return FPlatformTime::Seconds();
    }

    template<typename T, typename = TEnableIf<TIsFloatingPoint<T>::Value>>
    static void MarkCurrentTime(T& OutTimestamp)
    {
        OutTimestamp = GetSeconds();
    }

    template<typename T, typename = TEnableIf<TIsFloatingPoint<T>::Value>>
    static T GetElapsedTimeSecs(const T& InLastTimestamp)
    {
        return GetSeconds() - InLastTimestamp;
    }

    /**
     * @brief
     * It was observed that with high polling frequency as [0.01] or sometimes [0.1] second, we got crash on AutomationTest
     * module. Thus, [IntervalTimeInSec] as [0.5] sec is used for now.
     * @param InCond
     * @param InPassedCondAct
     * @param InTimeoutInSec
     * @param InIntervalTimeInSec
     * @return true
     * @return false
     */
    static bool WaitUntilThenAct(TFunctionRef<bool()> InCond,
                                 TFunctionRef<void()> InPassedCondAct,
                                 float InTimeoutInSec,
                                 float InIntervalTimeInSec = 0.5f);

    /**
     * @brief
     *
     * @param InCondition
     * @param InActionUponTimeout
     * @param InBeginTimeInSec
     * @param InTimeoutInSec
     * @return true
     * @return false
     */
    static bool CheckWithTimeOut(const TFunctionRef<bool()>& InCondition,
                                 const TFunctionRef<void()>& InActionUponTimeout,
                                 float InBeginTimeInSec,
                                 float InTimeoutInSec);

    static void StopRegisteredTimer(UWorld* InWorld, FTimerHandle& InTimerHandle)
    {
        check(IsValid(InWorld));
        // Also invalidate the timer here-in!
        InWorld->GetTimerManager().ClearTimer(InTimerHandle);
    }

    template<typename T, typename TDelegate>
    FORCEINLINE static FTimerHandle PlanToExecuteOnNextTick(T* InObj, typename TDelegate::template TMethodPtr<T> InMethod)
    {
        check(IsValid(InObj));
        return InObj->GetWorld()->GetTimerManager().SetTimerForNextTick(InObj, InMethod);
    }

    FORCEINLINE static FTimerHandle PlanToExecuteOnNextTick(UWorld* InWorld, const FTimerDelegate& InTimerDelegate)
    {
        check(IsValid(InWorld));
        return InWorld->GetTimerManager().SetTimerForNextTick(InTimerDelegate);
    }

    FORCEINLINE static FTimerHandle PlanToExecuteOnNextTick(UWorld* InWorld, TFunction<void()> InCallback)
    {
        check(IsValid(InWorld));
        return InWorld->GetTimerManager().SetTimerForNextTick(MoveTemp(InCallback));
    }

    template<typename T>
    FORCEINLINE static void RegisterRepeatedExecution(T* InObj,
                                                      FTimerHandle& InTimerHandle,
                                                      typename FTimerDelegate::template TMethodPtr<T>::FMethodPtr InMethod,
                                                      float InRate = 0.5f)
    {
        check(IsValid(InObj));
        InObj->GetWorld()->GetTimerManager().SetTimer(InTimerHandle, InObj, InMethod, InRate, true);
    }

    template<typename T>
    FORCEINLINE static void RegisterRepeatedExecution(T* InObj,
                                                      FTimerHandle& InTimerHandle,
                                                      TFunction<void()> Callback,
                                                      float InRate = 0.5f)
    {
        check(IsValid(InObj));
        InObj->GetWorld()->GetTimerManager().SetTimer(InTimerHandle, MoveTemp(Callback), InRate, true);
    }

    static void StopRegisteredExecution(UWorld* InWorld, FTimerHandle& InTimerHandle)
    {
        check(IsValid(InWorld));
        // Also invalidate it here-in!
        InWorld->GetTimerManager().ClearTimer(InTimerHandle);
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // PROCESS UTILS --
    //
    static int32 RunMonitoredProcess(FMonitoredProcess* InProcess,
                                     const float InTimeOutSecs,
                                     const FString& InProcessName = EMPTY_STR)
    {
        // Launch [InProcess]
        if (false == InProcess->Launch())
        {
            UE_LOG_WITH_INFO_SHORT(LogTemp, Error, TEXT("[%s] Process failed being launched"), *InProcessName);
            return -1;
        }

        // Wait for it to finish with [InTimeOutSecs]
        const float lastMarkedTime = URRCoreUtils::GetSeconds();
        bool bCancelled = false;
        while (InProcess->Update())
        {
            // Already slept in [Update()]
            if ((!bCancelled) && (URRCoreUtils::GetElapsedTimeSecs(lastMarkedTime) > InTimeOutSecs))
            {
                bCancelled = true;
                UE_LOG_WITH_INFO(LogTemp,
                                 Error,
                                 TEXT("[%s] Process is about to be terminated after timeout [%f secs]"),
                                 *InProcessName,
                                 InTimeOutSecs);

                // NOTE: This would trigger process cancelling, which should be waited for completion before this while loop exits
                InProcess->Cancel();
            }
        }
        return InProcess->GetReturnCode();
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // IMAGE UTILS --
    //
    // SIM IMAGE WRAPPERS --
    static constexpr const TCHAR* CIMAGE_WRAPPER_MODULE_NAME = TEXT("ImageWrapper");
    static IImageWrapperModule* SImageWrapperModule;
    static TMap<ERRFileType, TSharedPtr<IImageWrapper>> SImageWrappers;
    static void LoadImageWrapperModule();

    /**
     * @brief Load image to texture
     * @param InFullFilePath
     * @param InTextureName
     * @param bInSaveToAsset if True -> use ImportObject(), otherwise FImageUtils::ImportFileAsTexture2D()
     * @return UTexture2D*
     */
    static UTexture2D* LoadImageToTexture(const FString& InFullFilePath,
                                          const FString& InTextureName,
                                          const bool bInSaveToAsset = false);

    static bool LoadImagesFromFolder(const FString& InImageFolderPath,
                                     const TArray<ERRFileType>& InImageFileTypes,
                                     TArray<UTexture*>& OutImageTextureList,
                                     bool bIsLogged = false);

    static FRRLightProfileData SLightProfileData;
    static UTextureLightProfile* LoadIESProfile(const FString& InFullFilePath, const FString& InLightProfileName);
    static bool LoadIESProfilesFromFolder(const FString& InFolderPath,
                                          TArray<UTextureLightProfile*>& OutLightProfileList,
                                          bool bIsLogged = false);

    static TFunction<void(uint8*, const FUpdateTextureRegion2D*)> CleanupLightProfileData;

    static bool IsValidBitDepth(int32 InBitDepth)
    {
        return (URRActorCommon::IMAGE_BIT_DEPTH_INT8 == InBitDepth) || (URRActorCommon::IMAGE_BIT_DEPTH_FLOAT16 == InBitDepth) ||
               (URRActorCommon::IMAGE_BIT_DEPTH_FLOAT32 == InBitDepth);
    }

    template<int8 InBitDepth>
    FORCEINLINE static void GetCompressedImageData(const ERRFileType InImageFileType,
                                                   const FRRColorArray& InImageData,
                                                   const FIntPoint& ImageSize,
                                                   const int8 BitDepth,    // normally 8
                                                   const ERGBFormat RGBFormat,
                                                   TArray64<uint8>& OutCompressedData)
    {
        TSharedPtr<IImageWrapper> imageWrapper = URRCoreUtils::SImageWrappers[InImageFileType];
        verify(imageWrapper.IsValid());
        const auto& bitmap = InImageData.GetImageData<InBitDepth>();

        if (ERGBFormat::Gray == RGBFormat)
        {
            // Ref :FLandscapeWeightmapFileFormat_Png::Export
            verify(URRActorCommon::IMAGE_BIT_DEPTH_INT8 == InBitDepth);
            TArray<uint8> grayBitmap;
            for (const auto& color : bitmap)
            {
                grayBitmap.Add(color.R);
            }
            imageWrapper->SetRaw(grayBitmap.GetData(),
                                 grayBitmap.Num(),
                                 ImageSize.X,
                                 ImageSize.Y,
                                 ERGBFormat::Gray,
                                 URRActorCommon::IMAGE_BIT_DEPTH_INT8);
        }
        else
        {
            imageWrapper->SetRaw(
                bitmap.GetData(), bitmap.Num() * bitmap.GetTypeSize(), ImageSize.X, ImageSize.Y, RGBFormat, BitDepth);
        }

        // Get compressed data because uncompressed is the same fidelity, but much larger
        // EImageCompressionQuality::Default will make the Quality as 85, which is not optimal
        // Besides, this Quality value only matters to JPG, PNG compression is always lossless
        // Please refer to FJpegImageWrapper, FPngImageWrapper for details
        OutCompressedData = imageWrapper->GetCompressed(
            (ERRFileType::IMAGE_JPG == InImageFileType) ? 100 : static_cast<int32>(EImageCompressionQuality::Default));
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // CAMERA UTILS --
    //
    // -------------------------------------------------------------------------------------------------------------------------
    // GRAPHICS UTILS --
    //
    FORCEINLINE static bool ScreenMsg(const FColor& InColor, const FString& InMessage, float InTimeToDisplay = 50000.f)
    {
        if (GEngine)
        {
            GEngine->AddOnScreenDebugMessage(-1, InTimeToDisplay, InColor, *InMessage);
            return true;
        }
        return false;
    }

    /**
     * @brief Print message
     * @param InMessage
     * @param bInError
     * @param InTimeOnScreen (secs)
     */
    FORCEINLINE static void PrintMessage(const FString& InMessage, bool bInError = false, float InTimeOnScreen = 100.f)
    {
        if (bInError)
        {
            UE_LOG(LogTemp, Error, TEXT("%s"), *InMessage);
        }
        else
        {
            UE_LOG(LogTemp, Log, TEXT("%s"), *InMessage);
        }
        ScreenMsg(bInError ? FColor::Red : FColor::Yellow, InMessage, InTimeOnScreen);
#if RAPYUTA_SIM_DEBUG
        FPlatformMisc::MessageBoxExt(EAppMsgType::Ok, *InMessage, bInError ? TEXT("Error") : TEXT("Info"));
#endif
    }
};
