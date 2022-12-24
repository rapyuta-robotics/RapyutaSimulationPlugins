/**
 * @file RRAssetUtils.h
 * @brief Asset utils
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/ObjectLibrary.h"
#include "Engine/StaticMesh.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Materials/Material.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

// RapyutaSim
#include "RapyutaSimulationPlugins.h"

#include "RRAssetUtils.generated.h"

/**
 * @brief Asset utils.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRAssetUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    static IAssetRegistry& GetAssetRegistry()
    {
        FAssetRegistryModule& assetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
        return assetRegistryModule.Get();
    }

    static bool IsAssetPackageValid(const FAssetData& InAssetData, bool bIsLogged = false)
    {
        // Package's availability
        UPackage* assetPackage = InAssetData.GetPackage();
        if (nullptr == assetPackage)
        {
            if (bIsLogged)
            {
                UE_LOG(LogRapyutaCore, Error, TEXT("[%s] asset's package is not available!"), *InAssetData.AssetName.ToString());
            }
            return false;
        }

        // Package's validity
        FString packageFileName;
        if (false == FPackageName::DoesPackageExist(assetPackage->GetName(),
#if ENGINE_MAJOR_VERSION < 5
                                                    nullptr,
#endif
                                                    &packageFileName))
        {
            if (bIsLogged)
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("[%s] asset's package [%s] does not exist!"),
                       *InAssetData.AssetName.ToString(),
                       *packageFileName);
            }
            return false;
        }

        // Package's compatibility with this UE version
        // If a package has never been loaded, a file reader is necessary to find the package file summary for its saved engine
        // version.
        TUniquePtr<FArchive> packageReader = TUniquePtr<FArchive>(IFileManager::Get().CreateFileReader(*packageFileName));
        if (nullptr == packageReader)
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("Failed creating Package reader for package [%s]!"), *packageFileName);
            return false;
        }

        FPackageFileSummary packageSummary;
        *packageReader << packageSummary;

        // Package file UE version
#if ENGINE_MAJOR_VERSION < 5
        const int32 packageFileVersionUE = packageSummary.GetFileVersionUE4();
#else
        const int32 packageFileVersionUE = packageSummary.GetFileVersionUE().ToValue();
#endif

        // Check TOO OLD
        if (packageFileVersionUE < static_cast<int32>(VER_UE4_OLDEST_LOADABLE_PACKAGE))
        {
            if (bIsLogged)
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("[%s] asset was saved by a previous UE version which is not backward compatible with this one."
                            "Min Required version: [%d] vs Package version: [%d]"),
                       *InAssetData.AssetName.ToString(),
                       static_cast<int32>(VER_UE4_OLDEST_LOADABLE_PACKAGE),
                       packageFileVersionUE);
            }
            return false;
        }

        // Check TOO NEW
#if ENGINE_MAJOR_VERSION < 5
        const int32 packageFileVersionUEGlobal = GPackageFileUE4Version;
#else
        const int32 packageFileVersionUEGlobal = GPackageFileUEVersion.ToValue();
#endif

        if (packageFileVersionUE > packageFileVersionUEGlobal)
        {
            if (bIsLogged)
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("[%s] asset was saved by a newer UE version [%d], which is not forward compatible "
                            "with the current one[%d]"),
                       *InAssetData.AssetName.ToString(),
                       packageFileVersionUE,
                       packageFileVersionUEGlobal);
            }
            return false;
        }

        // Check Incompatibility
        if (false == FEngineVersion::Current().IsCompatibleWith(packageSummary.CompatibleWithEngineVersion))
        {
            if (bIsLogged)
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("[%s] asset's package's version [%s] is incompatible with the current UE version[%s]!"),
                       *InAssetData.AssetName.ToString(),
                       *packageSummary.CompatibleWithEngineVersion.ToString(),
                       *FEngineVersion::Current().ToString());
            }
            return false;
        }
        return true;
    }

    static bool IsAssetDataListValid(const TArray<FAssetData>& InAssetDataList, bool bLoadedCheck, bool bIsLogged = false)
    {
        bool bIsAssetValid = true;
        for (const auto& assetData : InAssetDataList)
        {
            // Check [assetData]'s validity
            if (false == assetData.IsValid())
            {
                bIsAssetValid = false;
                if (bIsLogged)
                {
                    UE_LOG(LogRapyutaCore, Error, TEXT("[%s] asset data info is invalid!"), *assetData.AssetName.ToString());
                }
            }
            else if (false == IsAssetPackageValid(assetData, bIsLogged))
            {
                bIsAssetValid = false;
            }
            else if (bLoadedCheck && (false == assetData.IsAssetLoaded()))
            {
                bIsAssetValid = false;
                if (bIsLogged)
                {
                    UE_LOG(LogRapyutaCore, Error, TEXT("[%s] asset failed to be loaded!"), *assetData.AssetName.ToString());
                }
            }

            // Print [assetData] if invalid
            if (false == bIsAssetValid)
            {
                if (bIsLogged)
                {
                    assetData.PrintAssetData();
                }
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Load asset with UObjectLibrary.
     *
     * @tparam T
     * @param InAssetsPath
     * @param OutAssetDataList
     * @param bHasBPAsset
     * @param bIsFullLoad
     * @sa [UObjectLibrary](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UObjectLibrary/)
     * @sa [AsyncLoading](https://docs.unrealengine.com/en-US/Programming/Assets/AsyncLoading/index.html)
     */
    template<typename T>
    static void LoadAssetDataList(const FString& InAssetsPath,
                                  TArray<FAssetData>& OutAssetDataList,
                                  bool bHasBPAsset = false,
                                  bool bIsFullLoad = false)
    {
        UObjectLibrary* objectLibrary = UObjectLibrary::CreateLibrary(T::StaticClass(), bHasBPAsset, GIsEditor);
        if (GIsEditor)
        {
            objectLibrary->bIncludeOnlyOnDiskAssets = false;
        }

        objectLibrary->bRecursivePaths = true;
        objectLibrary->LoadAssetDataFromPath(InAssetsPath);
        if (bIsFullLoad)
        {
            objectLibrary->LoadAssetsFromAssetData();
        }

        objectLibrary->GetAssetDataList(OutAssetDataList);
        verify(IsAssetDataListValid(OutAssetDataList, bIsFullLoad, true));
    }

    //(NOTE) This must not be invoked at Sim initialization since it would flush Async loaders away!
    // Besides, [ConstructorHelpers::FObjectFinder<T> asset(AssetPathName); will call [StaticFindObject()] instead
    // and requires to be run inside a ctor.
    // This loads synchronously, thus should be avoided if possible.
    template<typename T>
    FORCEINLINE static T* LoadObjFromAssetPath(UObject* Outer, const FString& InAssetPath)
    {
        if (InAssetPath.IsEmpty())
        {
            UE_LOG(LogTemp, Error, TEXT("[LoadObjFromAssetPath]::EMPTY PATH!"))
            return nullptr;
        }
        // This invokes [StaticLoadObject()]
        return LoadObject<T>(Outer, *InAssetPath);
    }
};
