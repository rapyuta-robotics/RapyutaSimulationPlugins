// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRCoreUtils.h"

// UE
#include "CoreMinimal.h"
#include "Engine/GameViewportClient.h"
#include "HAL/PlatformProcess.h"
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

IImageWrapperModule* URRCoreUtils::SImageWrapperModule = nullptr;
TMap<ERRFileType, TSharedPtr<IImageWrapper>> URRCoreUtils::SImageWrappers;

const TMap<ERRFileType, const TCHAR*> URRCoreUtils::SimFileExts = {{ERRFileType::NONE, EMPTY_STR},
                                                                   // UE & General
                                                                   {ERRFileType::UASSET, TEXT(".uasset")},
                                                                   {ERRFileType::INI, TEXT(".ini")},
                                                                   {ERRFileType::YAML, TEXT(".yaml")},

                                                                   // Image
                                                                   {ERRFileType::IMAGE_JPG, TEXT(".jpg")},
                                                                   {ERRFileType::IMAGE_PNG, TEXT(".png")},
                                                                   {ERRFileType::IMAGE_EXR, TEXT(".exr")},
                                                                   {ERRFileType::IMAGE_HDR, TEXT(".hdr")},

                                                                   // 3D Description formats
                                                                   {ERRFileType::URDF, TEXT(".urdf")},
                                                                   {ERRFileType::SDF, TEXT(".sdf")},
                                                                   {ERRFileType::GAZEBO_WORLD, TEXT(".world")},
                                                                   {ERRFileType::MJCF, TEXT(".mjcf")},

                                                                   // 3D CAD
                                                                   {ERRFileType::CAD_FBX, TEXT(".fbx")},
                                                                   {ERRFileType::CAD_OBJ, TEXT(".obj")},
                                                                   {ERRFileType::CAD_STL, TEXT(".stl")},
                                                                   {ERRFileType::CAD_DAE, TEXT(".dae")}};

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

bool URRCoreUtils::HasEnoughDiskSpace(const FString& InPath, uint64 InRequiredMemorySizeInBytes)
{
    uint64 totalDiskSize = 0;
    uint64 freeDiskSize = 0;
    FPlatformMisc::GetDiskTotalAndFreeSpace(InPath, totalDiskSize, freeDiskSize);

    bool bEnoughMemory = (freeDiskSize >= InRequiredMemorySizeInBytes);
    if (!bEnoughMemory)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("Not enough memory: FreeSizeOfDisk [%ld] < [%ld] InRequiredMemorySizeInBytes"),
               totalDiskSize,
               InRequiredMemorySizeInBytes);
        URRCoreUtils::ExecuteConsoleCommand(URRActorCommon::GetActorCommon(), URRCoreUtils::CMD_MEMORY_REPORT_FULL);
    }

    return bEnoughMemory;
}

bool URRCoreUtils::ShutDownSim(const UObject* InContextObject, uint64 InSimCompletionTimeoutInSecs)
{
    // END ALL SCENE INSTANCES' OPERATIONS --
    auto* gameState = URRCoreUtils::GetGameState<ARRGameState>(InContextObject);
    Async(
        EAsyncExecution::Thread,
        // Wait for Sim's full completion
        [gameState, InSimCompletionTimeoutInSecs]()
        {
            return URRCoreUtils::WaitUntilThenAct(
                [gameState]() { return gameState->HaveAllSceneInstancesCompleted(); }, []() {}, InSimCompletionTimeoutInSecs, 5.f);
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

// -------------------------------------------------------------------------------------------------------------------------
// FILE/DIR UTILS --
//
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

// -------------------------------------------------------------------------------------------------------------------------
// TIMER UTILS --
//
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
        FPlatformProcess::Sleep(InIntervalTimeInSec);
        elapsed_time = FTimespan(FDateTime::UtcNow() - begin).GetTotalSeconds();
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogRapyutaInternal, Display, TEXT("Waiting Thread Id: %ld"), URRThreadUtils::GetCurrentThreadId());
#endif
    }
    // Either InCond() is met or [elapsed_ticks] is over [InTimeoutInSec]

    if (bResult)
    {
        InPassedCondAct();
    }
    return bResult;
}

bool URRCoreUtils::CheckWithTimeOut(const TFunctionRef<bool()>& InCondition,
                                    const TFunctionRef<void()>& InAction,
                                    const FDateTime& InBeginTime,
                                    float InTimeoutInSec)
{
    if (InCondition())
    {
        return true;
    }

    double elapsed_time = FTimespan(FDateTime::UtcNow() - InBeginTime).GetTotalSeconds();
    if (elapsed_time > InTimeoutInSec)
    {
        InAction();
    }
    return false;
}
