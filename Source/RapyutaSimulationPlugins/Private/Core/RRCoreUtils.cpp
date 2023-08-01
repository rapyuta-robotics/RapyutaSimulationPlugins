// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRCoreUtils.h"

// UE
#include "CoreMinimal.h"
#include "Engine/GameViewportClient.h"
#include "HAL/PlatformProcess.h"
#include "IESConverter.h"
#include "ImageUtils.h"
#if WITH_EDITOR
#include "Factories/TextureFactory.h"
#endif

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRAssetUtils.h"
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
FRRLightProfileData URRCoreUtils::SLightProfileData;
TFunction<void(uint8*, const FUpdateTextureRegion2D*)> URRCoreUtils::CleanupLightProfileData =
    [](uint8*, const FUpdateTextureRegion2D*)
{
    FMemory::Free(SLightProfileData.ImageData);
    SLightProfileData.ImageData = nullptr;
};

FString URRCoreUtils::GetFileTypeFilter(const ERRFileType InFileType)
{
    return FString::Printf(TEXT("%s (*%s)|*%s"),
                           *URRTypeUtils::GetEnumValueAsString(TEXT("ERRFileType"), InFileType),
                           GetSimFileExt(InFileType),
                           GetSimFileExt(InFileType));
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
        SImageWrappers.Add(ERRFileType::IMAGE_GRAYSCALE_JPG, SImageWrapperModule->CreateImageWrapper(EImageFormat::GrayscaleJPEG));
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
    if (false == gameState->IsNetMode(NM_Standalone))
    {
        return true;
    }
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
    if (false == gameState->IsNetMode(NM_Standalone))
    {
        return true;
    }

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
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Warning,
                         TEXT("Not enough memory: FreeSizeOfDisk [%ld] < [%ld] InRequiredMemorySizeInBytes"),
                         totalDiskSize,
                         InRequiredMemorySizeInBytes);
        URRCoreUtils::ExecuteConsoleCommand(URRActorCommon::GetActorCommon(), URRCoreUtils::CMD_MEMORY_REPORT_FULL);
    }

    return bEnoughMemory;
}

bool URRCoreUtils::IsSimProfiling()
{
    return URRGameSingleton::Get()->BSIM_PROFILING;
}

bool URRCoreUtils::ShutDownSim(const UObject* InContextObject, uint64 InSimCompletionTimeoutInSecs)
{
    // END ALL SCENE INSTANCES' OPERATIONS --
    Async(
        EAsyncExecution::Thread,
        // Wait for Sim's full completion
        [InContextObject, InSimCompletionTimeoutInSecs]()
        {
            return URRCoreUtils::WaitUntilThenAct(
                [InContextObject]()
                {
                    auto* gameState = URRCoreUtils::GetGameState<ARRGameState>(InContextObject);
                    if (IsValid(gameState))
                    {
                        return gameState->HaveAllSceneInstancesCompleted();
                    }
                    else
                    {
                        UE_LOG_WITH_INFO(
                            LogRapyutaCore,
                            Warning,
                            TEXT("[ShutDownSim failed] GameState has gone invalid. Please make sure no level transition was in "
                                 "progress!"));
                        return false;
                    }
                },
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
#if STATS
                    // STOP STAT --
                    if (FThreadStats::IsCollectingData())
                    {
                        URRCoreUtils::ExecuteConsoleCommand(InContextObject, URRCoreUtils::CMD_STATS_STOP);
                    }
#endif

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
                                     const TArray<ERRFileType>& InFileTypes,
                                     const bool bInRecursive)
{
    bool bResult = false;

    if (FPaths::DirectoryExists(InFolderPath))
    {
        IFileManager& fileManager = IFileManager::Get();
        for (const auto& fileType : InFileTypes)
        {
            TArray<FString> filePaths;
            if (bInRecursive)
            {
                fileManager.FindFilesRecursive(
                    filePaths, *InFolderPath, *FString::Printf(TEXT("*%s"), URRCoreUtils::GetSimFileExt(fileType)), true, false);
            }
            else
            {
                fileManager.FindFiles(filePaths, *InFolderPath, URRCoreUtils::GetSimFileExt(fileType));
            }
            OutFilePaths.Append(filePaths);
        }

        bResult = (OutFilePaths.Num() > 0);
        if (!bResult)
        {
            const FString& fileTypesStr = FString::JoinBy(
                InFileTypes, TEXT(","), [](const ERRFileType& InFileType) { return URRCoreUtils::GetSimFileExt(InFileType); });
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Warning,
                             TEXT("NO files of extension [%s] %s inside [%s]"),
                             *fileTypesStr,
                             bInRecursive ? EMPTY_STR : TEXT("directly"),
                             *InFolderPath);
        }
    }
    else
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Error, TEXT("[%s] Directory NOT exist!"), *FPaths::ConvertRelativePathToFull(InFolderPath));
    }
    return bResult;
}

// -------------------------------------------------------------------------------------------------------------------------
// TIMER UTILS --
//
bool URRCoreUtils::WaitUntilThenAct(TFunctionRef<bool()> InCond,
                                    TFunctionRef<void()> InPassedCondAct,
                                    float InTimeoutInSec,
                                    float InIntervalTimeInSec)
{
    const float startTime = URRCoreUtils::GetSeconds();
    // Wait with a timeout
    bool bResult = false;
    while (!bResult && (URRCoreUtils::GetElapsedTimeSecs(startTime) < InTimeoutInSec))
    {
        bResult = InCond();
        // Sleep takes seconds, not msec
        FPlatformProcess::Sleep(InIntervalTimeInSec);
    }
    // Now, either InCond() has been met or [GetElapsedTimeSecs()] is over [InTimeoutInSec]

    if (bResult)
    {
        InPassedCondAct();
    }
    return bResult;
}

bool URRCoreUtils::CheckWithTimeOut(const TFunctionRef<bool()>& InCondition,
                                    const TFunctionRef<void()>& InActionUponTimeout,
                                    float InBeginTimeInSec,
                                    float InTimeoutInSec)
{
    if (InCondition())
    {
        return true;
    }

    if (URRCoreUtils::GetElapsedTimeSecs(InBeginTimeInSec) > InTimeoutInSec)
    {
        InActionUponTimeout();
    }
    return false;
}

// -------------------------------------------------------------------------------------------------------------------------
// IMAGE UTILS --
//
UTexture2D* URRCoreUtils::LoadImageToTexture(const FString& InFullFilePath, const FString& InTextureName, const bool bInSaveToAsset)
{
    if (bInSaveToAsset)
    {
#if WITH_EDITOR
        UPackage* texturePkg = URRAssetUtils::CreatePackageForSavingToAsset(*URRGameSingleton::Get()->GetDynamicAssetPath(
            ERRResourceDataType::UE_TEXTURE, InTextureName, RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME));
        static UTextureFactory* sTextureFactory =
            NewObject<UTextureFactory>(GetTransientPackage(), TEXT("TextureFactory"), RF_MarkAsRootSet);
        UTexture2D* texture = ImportObject<UTexture2D>(
            texturePkg, *InTextureName, RF_Public | RF_Standalone, *InFullFilePath, texturePkg, sTextureFactory);

        // Save [texture] to uasset file on disk, logged here-in
        return URRAssetUtils::SavePackageToAsset(texturePkg, texture) ? texture : nullptr;
#else
        UE_LOG_WITH_INFO(LogTemp, Error, TEXT("Texture saving to uasset is Editor-only feature"));
        return nullptr;
#endif
    }
    else
    {
        // NOTE: This use [UTexture2D::CreateTransient()], which is ineligible for saving to asset
        UTexture2D* texture = FImageUtils::ImportFileAsTexture2D(InFullFilePath);
        if (texture)
        {
            texture->Rename(*InTextureName);
        }
        return texture;
    }
}

bool URRCoreUtils::LoadImagesFromFolder(const FString& InImageFolderPath,
                                        const TArray<ERRFileType>& InImageFileTypes,
                                        TArray<UTexture*>& OutImageTextureList,
                                        bool bIsLogged)
{
    TArray<FString> imageFilePaths;
    bool bResult = LoadFullFilePaths(InImageFolderPath, imageFilePaths, InImageFileTypes);

    if (bResult)
    {
        for (const auto& imagePath : imageFilePaths)
        {
            // FPaths::GetCleanFilename() could be used but rather not due to being more expensive.
            // Also, imageFolderPath could be single or compound relative path, which must be unique to be texture name.
            FString&& textureName = imagePath.RightChop(InImageFolderPath.Len());
            if (UTexture2D* texture = LoadImageToTexture(imagePath, textureName))
            {
                OutImageTextureList.Add(texture);
            }
            else
            {
                // Continue the loading regardless of some being failed.
                bResult = false;
                UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Failed to load image to texture: [%s]"), *imagePath);
            }
        }
    }

    if (!bResult && bIsLogged)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Failed to load all images from [%s] into textures!"), *InImageFolderPath);
    }

    return bResult;
}

// Ref: DatasmithRuntime::GetTextureDataForIes() & CreateIESTexture()
UTextureLightProfile* URRCoreUtils::LoadIESProfile(const FString& InFullFilePath, const FString& InLightProfileName)
{
    TArray<uint8> buffer;
    if (!(FFileHelper::LoadFileToArray(buffer, *InFullFilePath) && buffer.Num() > 0))
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Failed loading file to array [%s]"), *InFullFilePath);
        return nullptr;
    }

    // checks for .IES extension to avoid wasting loading large assets just to reject them during header parsing
    FIESConverter iesConverter(buffer.GetData(), buffer.Num());
    if (false == iesConverter.IsValid())
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Error, TEXT("IESConverter failed creating buffer from image loaded from [%s]"), *InFullFilePath);
        return nullptr;
    }

    // Fille loaded data into [SLightProfileData]
    SLightProfileData.Width = iesConverter.GetWidth();
    SLightProfileData.Height = iesConverter.GetHeight();
    SLightProfileData.Brightness = iesConverter.GetBrightness();
    SLightProfileData.BytesPerPixel = 8;    // RGBA16F
    SLightProfileData.Pitch = SLightProfileData.Width * SLightProfileData.BytesPerPixel;
    SLightProfileData.TextureMultiplier = iesConverter.GetMultiplier();
    const TArray<uint8>& rawData = iesConverter.GetRawData();
    SLightProfileData.ImageData = (uint8*)FMemory::Malloc(rawData.Num() * sizeof(uint8), 0x20);
    FMemory::Memcpy(SLightProfileData.ImageData, rawData.GetData(), rawData.Num() * sizeof(uint8));
    ensure(SLightProfileData.ImageData);

    // [SLightProfileData] -> [lightProfile]
    UTextureLightProfile* lightProfile = NewObject<UTextureLightProfile>(GetTransientPackage(), *InLightProfileName, RF_Transient);
    if (!lightProfile)
    {
        return nullptr;
    }

#if WITH_EDITORONLY_DATA
    FAssetImportInfo importInfo;
    importInfo.Insert(FAssetImportInfo::FSourceFile(InLightProfileName));
    lightProfile->AssetImportData->SourceData = MoveTemp(importInfo);

    lightProfile->Source.Init(SLightProfileData.Width,
                              SLightProfileData.Height,
                              /*NumSlices=*/1,
                              1,
                              TSF_RGBA16F,
                              SLightProfileData.ImageData);
    CleanupLightProfileData(nullptr, nullptr);
#endif

    lightProfile->LODGroup = TEXTUREGROUP_IESLightProfile;
    lightProfile->AddressX = TA_Clamp;
    lightProfile->AddressY = TA_Clamp;
    lightProfile->CompressionSettings = TC_HDR;
#if WITH_EDITORONLY_DATA
    lightProfile->MipGenSettings = TMGS_NoMipmaps;
#endif
    lightProfile->Brightness = SLightProfileData.Brightness;
    lightProfile->TextureMultiplier = SLightProfileData.TextureMultiplier;

    // Update the texture with these new settings
    lightProfile->UpdateResource();

#if !WITH_EDITOR
    SLightProfileData.Region = FUpdateTextureRegion2D(0, 0, 0, 0, SLightProfileData.Width, SLightProfileData.Height);
    lightProfile->UpdateTextureRegions(0,
                                       1,
                                       &SLightProfileData.Region,
                                       SLightProfileData.Pitch,
                                       SLightProfileData.BytesPerPixel,
                                       SLightProfileData.ImageData,
                                       CleanupLightProfileData);
#endif
    return lightProfile;
}

bool URRCoreUtils::LoadIESProfilesFromFolder(const FString& InFolderPath,
                                             TArray<UTextureLightProfile*>& OutLightProfileList,
                                             bool bIsLogged)
{
    TArray<FString> filePaths;
    bool bResult = LoadFullFilePaths(InFolderPath, filePaths, {ERRFileType::LIGHT_PROFILE_IES});
    if (bResult)
    {
        for (const auto& iesProfilePath : filePaths)
        {
            // FPaths::GetCleanFilename() could be used but rather not due to being more expensive.
            // Also, InFolderPath could be single or compound relative path, which must be unique to be light profile name.
            FString&& iesProfileName = iesProfilePath.RightChop(InFolderPath.Len());
            if (UTextureLightProfile* iesProfile = LoadIESProfile(iesProfilePath, iesProfileName))
            {
                OutLightProfileList.Add(iesProfile);
            }
            else
            {
                // Continue the loading regardless of some being failed.
                bResult = false;
                UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Failed to load ies profile to light texture [%s]"), *iesProfilePath);
            }
        }
    }

    if (!bResult && bIsLogged)
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Error, TEXT("Failed to load all ies profiles from [%s] into light textures!"), *InFolderPath);
    }

    return bResult;
}
