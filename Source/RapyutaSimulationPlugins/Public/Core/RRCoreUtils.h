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
#include "UnrealEd.h"
#endif
#include "Delegates/Delegate.h"
#include "Engine/World.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "ImageUtils.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Guid.h"
#include "Stats/Stats.h"
#include "UObject/Object.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
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

    // SIM FILE EXTENSIONS --
    static const TMap<ERRFileType, const TCHAR*> SimFileExts;
    static FString GetFileTypeFilter(const ERRFileType InFileType);

    FORCEINLINE static ERRFileType GetFileType(const FString& InFilePath)
    {
        ERRFileType imageFileType = ERRFileType::NONE;

        for (uint8 i = 0; i < static_cast<uint8>(ERRFileType::TOTAL); ++i)
        {
            const ERRFileType& fileType = static_cast<ERRFileType>(i);
            if (InFilePath.EndsWith(URRCoreUtils::SimFileExts[fileType]))
            {
                imageFileType = fileType;
            }
        }
        return imageFileType;
    }

    FORCEINLINE static bool IsFileType(const FString& InFilePath, const TArray<ERRFileType>& InFileTypes)
    {
        for (const auto& fileType : InFileTypes)
        {
            if (InFilePath.EndsWith(URRCoreUtils::SimFileExts[fileType]))
            {
                return true;
            }
        }
        return false;
    }

// SIM COMMAND LINE EXECUTION --
#define CCMDLINE_ARG_FORMAT (TEXT("%s="))
    static constexpr const TCHAR* CCMDLINE_ARG_INT_PHYSX_DISPATCHER_NUM = TEXT("physxDispatcher");
    static void ExecuteConsoleCommand(const UObject* InContextObject, const FString& InCommandText)
    {
        GetPlayerController<APlayerController>(0, InContextObject)->ConsoleCommand(InCommandText);
        // OR either one of these:
        // + GEngine->Exec(World, *InCommandText);
        // + World->Exec(World, *InCommandText);
    }

    // T must be primitive type only!
    template<typename T>
    static FORCEINLINE constexpr bool GetCommandLineArgumentValue(const TCHAR* InArgName, T& OutArgValue, bool bIsLogged = false)
    {
        bool bResult = false;
        if constexpr (TIsSame<T, bool>::Value)
        {
            bResult = FParse::Bool(FCommandLine::Get(), *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }
        else
        {
            bResult = FParse::Value(FCommandLine::Get(), *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }

        if (!bResult && bIsLogged)
        {
            UE_LOG(LogTemp, Error, TEXT("Command line argument not found under the name [%s]"), InArgName);
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
        if constexpr (TIsSame<T, bool>::Value)
        {
            bResult = FParse::Bool(*InParams, *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }
        else
        {
            bResult = FParse::Value(*InParams, *FString::Printf(CCMDLINE_ARG_FORMAT, InArgName), OutArgValue);
        }

        if (!bResult && bIsLogged)
        {
            UE_LOG(LogTemp, Error, TEXT("[%s] does not contain an argument named [%s]"), *InParams, InArgName);
        }
        return bResult;
    }

    // SIM WORLDS --
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

    // This value could be configured in [DefaultEngine.ini]
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

    static bool HasPlayerControllerListInitialized(const UObject* InContextObject, bool bIsLogged = false);

    // GameState & PlayerController should be able to be recognized polymorphically!
    static bool HasSimInitialized(const UObject* InContextObject, bool bIsLogged = false);
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

    // -------------------------------------------------------------------------------------------------------------------------
    // FILE/DIR UTILS --
    //
    static bool LoadFullFilePaths(const FString& FolderPath, TArray<FString>& OutFilePaths, const TArray<ERRFileType>& InFileTypes);

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
                UE_LOG(LogTemp, Error, TEXT("Failed to create dir %s"), *dirFullPath)
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
                UE_LOG(LogTemp, Fatal, TEXT("[%s] directory path is required to be an absolute path."), *InDirPath);
            }
            else if (false == FPaths::DirectoryExists(InDirPath))
            {
                UE_LOG(LogTemp, Fatal, TEXT("[%s] directory does not exist."), *InDirPath);
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
        return FPlatformTime::Seconds();
    }

    // It was observed that with high polling frequency as [0.01] or sometimes [0.1] second, we got crash on AutomationTest
    // module. Thus, [IntervalTimeInSec] as [0.5] sec is used for now.
    static bool WaitUntilThenAct(TFunctionRef<bool()> InCond,
                                 TFunctionRef<void()> InPassedCondAct,
                                 float InTimeoutInSec,
                                 float InIntervalTimeInSec = 0.5f);

    static bool CheckWithTimeOut(const TFunctionRef<bool()>& InCondition,
                                 const TFunctionRef<void()>& InAction,
                                 const FDateTime& InBeginTime,
                                 float InTimeoutInSec);

    static void StopRegisteredTimer(UWorld* InWorld, FTimerHandle& InTimerHandle)
    {
        check(IsValid(InWorld));
        // Also invalidate the timer here-in!
        InWorld->GetTimerManager().ClearTimer(InTimerHandle);
    }

    template<typename T>
    FORCEINLINE static FTimerHandle PlanToExecuteOnNextTick(T* InObj,
                                                            typename FTimerDelegate::TUObjectMethodDelegate<T>::FMethodPtr InMethod)
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
                                                      typename FTimerDelegate::TUObjectMethodDelegate<T>::FMethodPtr InMethod,
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
    // IMAGE UTILS --
    //
    // SIM IMAGE WRAPPERS --
    static constexpr const TCHAR* CIMAGE_WRAPPER_MODULE_NAME = TEXT("ImageWrapper");
    static IImageWrapperModule* SImageWrapperModule;
    static TMap<ERRFileType, TSharedPtr<IImageWrapper>> SImageWrappers;
    static void LoadImageWrapperModule();

    static UTexture2D* LoadImageToTexture(const FString& InFullFilePath, const FString& InTextureName)
    {
        UTexture2D* loadedTexture = FImageUtils::ImportFileAsTexture2D(InFullFilePath);
        if (loadedTexture)
        {
            loadedTexture->Rename(*InTextureName);
        }

        return loadedTexture;
    }

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
        imageWrapper->SetRaw(bitmap.GetData(), bitmap.GetAllocatedSize(), ImageSize.X, ImageSize.Y, RGBFormat, BitDepth);

        // Get compressed data because uncompressed is the same fidelity, but much larger
        // EImageCompressionQuality::Default will make the Quality as 85, which is not optimal
        // Besides, this Quality value only matters to JPG, PNG compression is always lossless
        // Please refer to FJpegImageWrapper, FPngImageWrapper for details
        OutCompressedData = imageWrapper->GetCompressed(100);
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
};
