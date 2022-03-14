// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRCoreUtils.h"

// UE
#include "CoreMinimal.h"
#include "Engine/GameViewportClient.h"
#include "ImageUtils.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRBaseActor.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRGameState.h"
#include "Core/RRPlayerController.h"
#include "Core/RRSceneDirector.h"
#include "Core/RRStaticMeshComponent.h"
#include "Core/RRThreadUtils.h"
#include "Core/RRTypeUtils.h"
#include "Core/RRUObjectUtils.h"

uint16 URRCoreUtils::SLatestSessionIndex = 0;
uint64 URRCoreUtils::SCapturedRGBImagesTotalCount = 0;

IImageWrapperModule* URRCoreUtils::SImageWrapperModule = nullptr;
TMap<ERRFileType, TSharedPtr<IImageWrapper>> URRCoreUtils::SImageWrappers;

const TMap<ERRFileType, const TCHAR*> URRCoreUtils::SimFileExts = {
    {ERRFileType::NONE, EMPTY_STR},
    {ERRFileType::UASSET, TEXT(".uasset")},
    {ERRFileType::INI, TEXT(".ini")},
    {ERRFileType::IMAGE_JPG, TEXT(".jpg")},
    {ERRFileType::IMAGE_PNG, TEXT(".png")},
    {ERRFileType::IMAGE_EXR, TEXT(".exr")},
    {ERRFileType::URDF, TEXT(".urdf")},
    {ERRFileType::SDF, TEXT(".sdf")},
    {ERRFileType::MJCF, TEXT(".mjcf")},
};

FString URRCoreUtils::GetFileTypeFilter(const ERRFileType InFileType)
{
    return FString::Printf(TEXT("%s (*%s)|*%s"),
                           *URRTypeUtils::GetEnumValueAsString(TEXT("ERRFileType"), InFileType),
                           SimFileExts[InFileType],
                           SimFileExts[InFileType]);
}

void URRCoreUtils::LoadImageWrapperModule()
{
    if (!SImageWrapperModule)
    {
        SImageWrapperModule = FModuleManager::LoadModulePtr<IImageWrapperModule>(CIMAGE_WRAPPER_MODULE_NAME);
    }

    if (SImageWrappers.Num() == 0)
    {
        verify(SImageWrapperModule);
        SImageWrappers.Add(ERRFileType::IMAGE_JPG, SImageWrapperModule->CreateImageWrapper(EImageFormat::JPEG));
        SImageWrappers.Add(ERRFileType::IMAGE_PNG, SImageWrapperModule->CreateImageWrapper(EImageFormat::PNG));
        SImageWrappers.Add(ERRFileType::IMAGE_EXR, SImageWrapperModule->CreateImageWrapper(EImageFormat::EXR));
    }
}

void URRCoreUtils::OnWorldCleanup(UWorld* World, bool bSessionEnded, bool bCleanupResources)
{
    URRActorCommon::SActorCommonList.Empty();
    SLatestSessionIndex = 0;
    SCapturedRGBImagesTotalCount = 0;
}

int32 URRCoreUtils::GetMaxSplitscreenPlayers(const UObject* InContextObject)
{
    UGameInstance* gameInstance = GetGameInstance<UGameInstance>(InContextObject);
    check(gameInstance);
    UGameViewportClient* gameViewportClient = gameInstance->GetGameViewportClient();
    return gameViewportClient ? gameViewportClient->MaxSplitscreenPlayers : 1;
}

bool URRCoreUtils::HasPlayerControllerListInitialized(const UObject* InContextObject, bool bIsLogged)
{
    ARRGameState* gameState = GetGameState<ARRGameState>(InContextObject);
    for (int8 i = 0; i < gameState->SCENE_INSTANCES_NUM; ++i)
    {
        const auto* playerController = URRCoreUtils::GetPlayerController<ARRPlayerController>(i, InContextObject);
        check(playerController);
        if (!playerController->HasInitialized(bIsLogged))
        {
            return false;
        }
    }
    return true;
}

bool URRCoreUtils::HasSimInitialized(const UObject* InContextObject, bool bIsLogged)
{
    const auto* gameState = URRCoreUtils::GetGameState<ARRGameState>(InContextObject);
    check(gameState);

    // [GameState]
    if (!gameState->HasInitialized(bIsLogged))
    {
        return false;
    }

    // [PlayerController]
    return HasPlayerControllerListInitialized(InContextObject, bIsLogged);
}

URRSceneInstance* URRCoreUtils::GetSceneInstance(const UObject* InContextObject, int8 InSceneInstanceId)
{
    auto gameState = URRCoreUtils::GetGameState<ARRGameState>(InContextObject);
    check(gameState);
    return gameState->GetSceneInstance<URRSceneInstance>(InSceneInstanceId);
}

ARRSceneDirector* URRCoreUtils::GetSceneDirector(const UObject* InContextObject, int8 InSceneInstanceId)
{
    return GetSceneInstance(InContextObject, InSceneInstanceId)->SceneDirector;
}

FVector URRCoreUtils::GetSceneInstanceLocation(int8 InSceneInstanceId)
{
    return URRActorCommon::GetActorCommon(InSceneInstanceId)->SceneInstanceLocation;
}

float URRCoreUtils::GetDistanceToSimSceneOrigin(int8 InSceneInstanceId, const FVector& InTargetLocation)
{
    // This calculation is not accurate, since we are not taking into account both rotation of the camera
    // and the actual furthest point from the camera. Since this feature is currently not needed, it is left as is.
    // To take rotation into account: dist = distFromOrigin / (tan(theta) / tan(FoV / 2) - 1)
    return FVector::Dist(URRCoreUtils::GetSceneInstanceLocation(InSceneInstanceId), InTargetLocation);
}

bool URRCoreUtils::HasEnoughDiskSpace(const FString& InPath, uint64 InRequiredMemorySizeInBytes)
{
    uint64 totalDiskSize = 0;
    uint64 freeDiskSize = 0;
    FPlatformMisc::GetDiskTotalAndFreeSpace(InPath, totalDiskSize, freeDiskSize);

    bool hasEnoughMemory = (freeDiskSize >= InRequiredMemorySizeInBytes);
    if (!hasEnoughMemory)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%ld] Not enough memory: FreeSizeOfDisk [%ld] < [%ld] InRequiredMemorySizeInBytes"),
               totalDiskSize,
               totalDiskSize,
               InRequiredMemorySizeInBytes);
        URRCoreUtils::ExecuteConsoleCommand(URRActorCommon::GetActorCommon(), URRCoreUtils::CMD_MEMORY_REPORT_FULL);
    }

    return hasEnoughMemory;
}

bool URRCoreUtils::HaveAllSimSceneInstancesCompleted(ARRGameState* InGameState)
{
    for (int8 i = 0; i < InGameState->SceneInstanceList.Num(); ++i)
    {
        if (false == InGameState->SceneInstanceList[i]->SceneDirector->HasOperationCompleted(true))
        {
            return false;
        }
    }

    return (InGameState->GetTotalPlannedCaptureCount() == URRCoreUtils::GetCapturedRGBImagesTotalCount());
}

bool URRCoreUtils::ShutDownSim(const UObject* InContextObject, uint64 InSimCompletionTimeoutInSecs)
{
    // END ALL SIM MODE INSTANCES' OPERATIONS --
    auto* gameState = URRCoreUtils::GetGameState<ARRGameState>(InContextObject);
    Async(
        EAsyncExecution::Thread,
        // Wait for Sim's full completion
        [gameState, InSimCompletionTimeoutInSecs]()
        {
            // [URRThreadFunctionLibrary::WaitUntilAllGraphTasksCompleteInThread(ENamedThreads::GameThread)]
            // could probably be used but it does not run with a time-out.
            return URRCoreUtils::WaitUntilThenAct([gameState]()
                                                  { return URRCoreUtils::HaveAllSimSceneInstancesCompleted(gameState); },
                                                  []() {},
                                                  InSimCompletionTimeoutInSecs,
                                                  5.f);
        },
        // Issue Sim Ending command
        [InContextObject]()
        {
            URRThreadUtils::DoTaskInGameThread(
                [InContextObject]()
                {
                    // STOP STAT --
                    if (Stats::IsThreadCollectingData())
                    {
                        URRCoreUtils::ExecuteConsoleCommand(InContextObject, URRCoreUtils::CMD_STATS_STOP);
                    }

                    // LOG NO OF GENERATED CAPTURED IMAGES --
                    auto* gameState = URRCoreUtils::GetGameState<ARRGameState>(InContextObject);
                    const uint64& plannedCaptureCount = gameState->GetTotalPlannedCaptureCount();
                    const uint64& generatedCount = URRCoreUtils::GetCapturedRGBImagesTotalCount();
                    if (plannedCaptureCount == generatedCount)
                    {
                        UE_LOG(LogRapyutaCore,
                               Display,
                               TEXT("SIM GENERATED [%ld] VS [%ld] PLANNED IMAGES, NOW SHUTTING DOWN..."),
                               generatedCount,
                               plannedCaptureCount);
                    }
                    else
                    {
                        UE_LOG(LogRapyutaCore,
                               Error,
                               TEXT("SIM GENERATED [%ld] VS [%ld] PLANNED IMAGES, NOW SHUTTING DOWN..."),
                               generatedCount,
                               plannedCaptureCount);
                    }

                    // SHUT DOWN SIM --
                    URRCoreUtils::ExecuteSimQuitCommand(InContextObject);
                });
        });
    return true;
}

void URRCoreUtils::ExecuteSimQuitCommand(const UObject* InContextObject)
{
    URRCoreUtils::ExecuteConsoleCommand(InContextObject, URRCoreUtils::CMD_SIM_QUIT);
}

bool URRCoreUtils::ComposeSimVersionFile()
{
    URRActorCommon* actorCommon = URRActorCommon::GetActorCommon();
    const FString& versionFilePath = actorCommon->GameState->GetSimVersionFilePath();
    bool result = FFileHelper::SaveStringToFile(RAPYUTA_SIM_VERSION_INFO.ToString(), *versionFilePath);
    if (!result)
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[%s] Failed writing Sim version [%s] to file [%s]"),
               *RAPYUTA_SIM_VERSION_INFO.ToString(),
               *versionFilePath)
    }
    return result;
}

bool URRCoreUtils::WaitUntilThenAct(TFunctionRef<bool()> InCond,
                                    TFunctionRef<void()> InPassedCondAct,
                                    float InTimeoutInSec,
                                    float InIntervalTimeInSec)
{
    FDateTime begin(FDateTime::UtcNow());
    double elapsed_time = 0;
    // Wait with a timeout
    bool bResult = false;
    while (!bResult && elapsed_time < InTimeoutInSec)
    {
        bResult = InCond();
        // Sleep takes seconds, not msec
        URRThreadUtils::Sleep(InIntervalTimeInSec);
        elapsed_time = FTimespan(FDateTime::UtcNow() - begin).GetTotalSeconds();
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaCore, Display, TEXT("Waiting Thread Id: %ld"), URRThreadUtils::GetCurrentThreadId());
#endif
    }
    // Either InCond() is met or [elapsed_ticks] is over [InTimeoutInSec]

    if (bResult)
    {
        InPassedCondAct();
    }
    return bResult;
}

bool URRCoreUtils::LoadFullFilePaths(const FString& InFolderPath,
                                     TArray<FString>& OutFilePaths,
                                     const TArray<ERRFileType>& InFileTypes)
{
    bool result = false;

    if (FPaths::DirectoryExists(InFolderPath))
    {
        IFileManager& fileManager = IFileManager::Get();
        for (const auto& fileType : InFileTypes)
        {
            TArray<FString> filePaths;
            fileManager.FindFilesRecursive(
                filePaths, *InFolderPath, *FString::Printf(TEXT("*%s"), URRCoreUtils::SimFileExts[fileType]), true, false);
            OutFilePaths.Append(filePaths);
        }

        result = (OutFilePaths.Num() > 0);
        if (!result)
        {
            const FString& fileTypesStr = FString::JoinBy(
                InFileTypes, TEXT(","), [](const ERRFileType& InFileType) { return URRCoreUtils::SimFileExts[InFileType]; });
            UE_LOG(LogRapyutaCore,
                   Error,
                   TEXT("Failed to find any files of extension [%s] inside [%s]"),
                   *fileTypesStr,
                   *InFolderPath);
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("[%s] Directory NOT exist!"), *FPaths::ConvertRelativePathToFull(InFolderPath));
    }
    return result;
}
