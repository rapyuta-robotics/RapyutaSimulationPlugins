// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Core/RRPakLoader.h"

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Core/RRAssetUtils.h"
#include "Core/RRGameSingleton.h"

// Internal folder name to use as base for the assets found in the mounted PAK files
// NOTE: This is a hard-coded name defined by UE!
static constexpr const TCHAR* UE_PAK_MOUNTED_BASE_FOLDER_NAME = TEXT("Paks");

bool URRPakLoader::Initialize()
{
#if WITH_EDITOR
    UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Warning, TEXT("Pak loader is NOT used in Editor"));
    return false;
#else
    RRGameSingleton = URRGameSingleton::Get();
    PakManager = static_cast<FPakPlatformFile*>(FPlatformFileManager::Get().FindPlatformFile(FPakPlatformFile::GetTypeName()));
    if (PakManager)
    {
        UE_LOG(LogRapyutaCore, Log, TEXT("Use existing PAK platform"));
    }
    else
    {
        PakManager = new FPakPlatformFile();
        if (!PakManager->Initialize(&FPlatformFileManager::Get().GetPlatformFile(), nullptr))
        {
            UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("Failed to initialize PAK platform"));
            return false;
        }

        // [PakManager], in case of already existing, got this done by [FPlatformFileManager::InitializeNewAsyncIO()]
        PakManager->InitializeNewAsyncIO();

        // Set [PakManager] as [FPlatformFileManager]'s TopmostPlatformFile
        FPlatformFileManager::Get().SetPlatformFile(*PakManager);
        UE_LOG(LogRapyutaCore, Log, TEXT("Use custom PAK platform file"));
    }
    UE_LOG(LogRapyutaCore, Log, TEXT("[PakManager] initialized"));
    return true;
#endif
}

void URRPakLoader::MountPAKFiles(const TArray<FString>& InPAKPaths)
{
    for (const FString& sourcePakPath : InPAKPaths)
    {
        UE_LOG(LogRapyutaCore, Log, TEXT("Mount PAK path [%s]"), *sourcePakPath);

        // 0- CREATE a PAK file and check its contents
        TRefCountPtr<FPakFile> pakFilePtr = new FPakFile(PakManager->GetLowerLevel(), *sourcePakPath, false);
        FPakFile& pakFile = *pakFilePtr;

        if (false == pakFile.Check())
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("Pak file [%s] is invalid"), *sourcePakPath);
            continue;
        }

        // NOTE: A pak's original mount point is its author's local PC disk path saved during packing
        const FString& originalMountPoint = pakFile.GetMountPoint();
        UE_LOG(LogRapyutaCore, Log, TEXT("- Original mount point: %s"), *originalMountPoint);
        FString pakFolderRelPath;
        originalMountPoint.Split(FApp::GetProjectName(), nullptr, &pakFolderRelPath);

        // 1- MOUNT the Pak at the exactly same relative location under package dir
        // NOTE: [FPaths::ProjectDir()] is also package dir.
        // This is a mount point, thus must not use [FPaths::ConvertRelativePathToFull()]
        const FString newMountPoint = FPaths::RemoveDuplicateSlashes(FPaths::ProjectDir() / pakFolderRelPath);
        UE_LOG(LogRapyutaCore, Log, TEXT("- New mount point: %s"), *newMountPoint);

        pakFile.SetMountPoint(*newMountPoint);
        if (!PakManager->Mount(*sourcePakPath, 0, *newMountPoint))
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("Failed to mount package [%s] on [%s]"), *sourcePakPath, *newMountPoint);
            continue;
        }
#if RAPYUTA_SIM_DEBUG
        // NOTE: This is not needed, only kept for ref
        FPackageName::RegisterMountPoint(TEXT("/Game/"), newMountPoint);
#endif

        // 2- VERIFY Pak contents' mounted paths
        // THESE MOUNTED RESOURCE-PATHS ARE THEN CONVERTED TO SOFT-OBJECT-PATHS as BEING LOADED BY [AssetManager's FStreamableManager]
        // -> THUS STARTING FROM PACKAGE DIR, THEY MUST BE EXACTLY THE SAME AS IN THE PROJECT DIR WHEN BEING PACKED.
        // EG: <PackageDir>/Plugins/<PluginDir>/Content/DynamicContents/<ResourceTypeDir>/<ResourceFile>
        TArray<FString> pakContentPathList;
        pakFile.FindPrunedFilesAtPath(pakContentPathList, *pakFile.GetMountPoint(), true, false, true);
        UE_LOG(LogRapyutaCore, Display, TEXT("[%s] has been mounted to files:"), *sourcePakPath);
        for (const auto& resourceMountedPath : pakContentPathList)
        {
            UE_LOG(LogRapyutaCore, Log, TEXT("- [%s]"), *FPaths::ConvertRelativePathToFull(resourceMountedPath));
        }
    }

    // 3- Force rescan of all assets after mounting PAKs -> assets
    URRAssetUtils::GetAssetRegistry().ScanPathsSynchronous(
        URRGameSingleton::GetDynamicAssetsBasePathList(ERRResourceDataType::UE_PAK), true);

#if RAPYUTA_SIM_VERBOSE
    TArray<FString> allAssetPaths;
    URRAssetUtils::GetAssetRegistry().GetAllCachedPaths(allAssetPaths);

    UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Warning, TEXT("All PAK-mounted asset paths fetched from AssetRegistry"));
    for (const FString& path : allAssetPaths)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("- %s"), *path);
    }
#endif
}

bool URRPakLoader::LoadPAKFiles(const FString& InPakFolderPath)
{
    if (!ensure(PakManager))
    {
        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("PakManager seems not yet initialized"));
        return false;
    }

    // FETCH [pakPaths] & MOUNT TO FILES ON DISK
    TArray<FString> pakPaths;
    if (URRCoreUtils::LoadFullFilePaths(InPakFolderPath, pakPaths, {ERRFileType::PAK}))
    {
        UE_LOG(LogRapyutaCore, Log, TEXT("Found %d paks in folder [%s]"), pakPaths.Num(), *InPakFolderPath);

        // MOUNT [pakPaths]
        MountPAKFiles(pakPaths);
    }
    return true;
}
