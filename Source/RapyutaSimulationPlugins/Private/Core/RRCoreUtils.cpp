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

const TMap<ERRFileType, const TCHAR*> URRCoreUtils::SimFileExts = {
    {ERRFileType::NONE, EMPTY_STR},
    {ERRFileType::UASSET, TEXT(".uasset")},
    {ERRFileType::INI, TEXT(".ini")},
    {ERRFileType::IMAGE_JPG, TEXT(".jpg")},
    {ERRFileType::IMAGE_PNG, TEXT(".png")},
    {ERRFileType::IMAGE_EXR, TEXT(".exr")},
    {ERRFileType::URDF, TEXT(".urdf")},
    {ERRFileType::SDF, TEXT(".sdf")},
    {ERRFileType::GAZEBO_WORLD, TEXT(".world")},
    {ERRFileType::YAML, TEXT(".yaml")},
    {ERRFileType::MJCF, TEXT(".mjcf")},
};

FString URRCoreUtils::GetFileTypeFilter(const ERRFileType InFileType)
{
    return FString::Printf(TEXT("%s (*%s)|*%s"),
                           *URRTypeUtils::GetEnumValueAsString(TEXT("ERRFileType"), InFileType),
                           SimFileExts[InFileType],
                           SimFileExts[InFileType]);
}

void URRCoreUtils::OnWorldCleanup(UWorld* World, bool bSessionEnded, bool bCleanupResources)
{
    URRActorCommon::SActorCommonList.Empty();
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
