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
#include "UObject/Package.h"
#include "UObject/SavePackage.h"

#if WITH_EDITOR
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#endif

// RapyutaSim
#include "Core/RRObjectCommon.h"
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
        static FAssetRegistryModule& assetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
        return assetRegistryModule.Get();
    }

#if WITH_EDITOR
    static IAssetTools& GetAssetToolsModule()
    {
        static FAssetToolsModule& assetToolsModule = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools");
        return assetToolsModule.Get();
    }
#endif

    static bool IsAssetPackageValid(const FAssetData& InAssetData, bool bIsLogged = false)
    {
        // Package's availability
        UPackage* assetPackage = InAssetData.GetPackage();
        if (nullptr == assetPackage)
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(
                    LogRapyutaCore, Error, TEXT("[%s] asset's package is not available!"), *InAssetData.AssetName.ToString());
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
                UE_LOG_WITH_INFO(LogRapyutaCore,
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
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Failed creating Package reader for package [%s]!"), *packageFileName);
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
                UE_LOG_WITH_INFO(
                    LogRapyutaCore,
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
                UE_LOG_WITH_INFO(LogRapyutaCore,
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
                UE_LOG_WITH_INFO(LogRapyutaCore,
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
                    UE_LOG_WITH_INFO(
                        LogRapyutaCore, Error, TEXT("[%s] asset data info is invalid!"), *assetData.AssetName.ToString());
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
                    UE_LOG_WITH_INFO(
                        LogRapyutaCore, Error, TEXT("[%s] asset failed to be loaded!"), *assetData.AssetName.ToString());
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

    /**
     * @brief
     * This must not be invoked at Sim initialization since it would flush Async loaders away!
     * Besides, [ConstructorHelpers::FObjectFinder<T> asset(AssetPathName); will call [StaticFindObject()] instead
     * and requires to be run inside a ctor.
     * This loads synchronously, thus should be avoided if possible.
     * @tparam T
     * @param Outer
     * @param InAssetPath
     * @return FORCEINLINE*
     */
    template<typename T>
    FORCEINLINE static T* LoadObjFromAssetPath(UObject* Outer, const FString& InAssetPath)
    {
        if (InAssetPath.IsEmpty())
        {
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[EMPTY PATH!"))
            return nullptr;
        }
        // This invokes [StaticLoadObject()]
        return LoadObject<T>(Outer, *InAssetPath);
    }

    /**
     * @brief Fetch UObject from asset, called in Constructor only
     * @tparam T
     * @param InAssetPath
     * @return T*
     */
    template<typename T>
    FORCEINLINE static T* FetchObjectFromAsset(const TCHAR* InAssetPath)
    {
        return ConstructorHelpers::FObjectFinderOptional<T>(InAssetPath).Get();
    }

    /**
     * @brief Fetch UClass from asset, called in Constructor only
     * @tparam T
     * @param InClassAssetPath, which requires trailing "_C"
     * @return TSubclassOf<T>
     */
    template<typename T>
    FORCEINLINE static TSubclassOf<T> FetchClassFromAsset(const TCHAR* InClassAssetPath)
    {
        // NOTE: For class asset, "_C" is required at the end of class path
        ConstructorHelpers::FClassFinder<T> classFinder(InClassAssetPath);
        return classFinder.Class;
    }

    /**
     * @brief Save object in memory to asset file on disk stored by a module
     * @param InObject
     * @param InAssetDataType
     * @param InAssetUniqueName Unique name for the output asset
     * @param InModuleName
     * @param bInStripEditorOnlyContent
     * @return true if succeeded
     */
    static bool SaveObjectToAssetInModule(UObject* InObject,
                                          const ERRResourceDataType InAssetDataType,
                                          const FString& InAssetUniqueName,
                                          const TCHAR* InModuleName,
                                          bool bInStripEditorOnlyContent = false);
    /**
     * @brief Save object in memory to asset file on disk
     * @param InObject
     * @param InAssetPath Base package path of the output asset
     * @param bInStripEditorOnlyContent
     * @return true if succeeded
     * @sa https://forums.unrealengine.com/t/calling-upackage-savepackage-causes-fatal-assert-in-staticfindobjectfast/447917
     * @sa https://forums.unrealengine.com/t/dynamically-created-primary-assets-not-registering-with-asset-manager/210255
     * @sa https://forums.unrealengine.com/t/how-to-work-with-cooked-content-in-editor/265094
     */
    static bool SaveObjectToAsset(UObject* InObject, const FString& InAssetPath, bool bInStripEditorOnlyContent = false);

    /**
     * @brief Save package to asset file on disk
     * @param InObject
     * @return true if succeeded
     */
    static bool SavePackageToAsset(UPackage* InPackage, UObject* InObject);

    /**
     * @brief Find generated UClass from blueprint class name
     * @param InBlueprintClassName
     * @return UClass*
     */
    // Ref: EditorUtilitySubsystem
    FORCEINLINE static UClass* FindBlueprintClass(const FString& InBlueprintClassName)
    {
        IAssetRegistry& assetRegistry = GetAssetRegistry();
        if (assetRegistry.IsLoadingAssets())
        {
            assetRegistry.SearchAllAssets(true);
        }

        FString targetName = InBlueprintClassName;
        targetName.RemoveFromEnd(TEXT("_C"), ESearchCase::CaseSensitive);

        // Note: For the assets to be listed in Package build, Go to ProjectSettings to configure [PrimaryAssetTypesToScan].
        // Configuring PackagePaths here does not work
        // https://maladius.com/posts/asset_manager_1
        FARFilter filter;
        filter.bRecursivePaths = true;
        filter.bRecursiveClasses = true;
        filter.ClassPaths.Add(UBlueprintCore::StaticClass()->GetClassPathName());

        // Find the blueprint asset of [InBlueprintClassName]
        UClass* foundClass = nullptr;
        assetRegistry.EnumerateAssets(
            filter,
            [&foundClass, targetName](const FAssetData& AssetData)
            {
                if ((AssetData.AssetName.ToString() == targetName) || (AssetData.GetObjectPathString() == targetName))
                {
                    if (UBlueprint* bp = Cast<UBlueprint>(AssetData.GetAsset()))
                    {
                        foundClass = bp->GeneratedClass;
                        return false;
                    }
                }

                return true;
            });

        return foundClass;
    }

    /**
     * @brief Create a child blueprint class from parent UClass.
     * Ref: [FKismetEditorUtilities::CreateBlueprintFromClass()]
     * @param InParentClass
     * @param InBlueprintClassName
     * @param InCDOFunc
     * @param bInSaveBP Whether or not saving the output BP to disk
     * @param InBPBasePath Base UE path for saving BP
     * @return UClass*
     */
    static UClass* CreateBlueprintClass(UClass* InParentClass,
                                        const FString& InBlueprintClassName,
                                        const TFunction<void(UObject* InCDO)>& InCDOFunc = nullptr,
                                        const bool bInSaveBP = false,
                                        const FString& InBPBasePath = TEXT(""));
    /**
     * @brief Create a blueprint from an AActor
     * Ref: [FKismetEditorUtilities::CreateBlueprintFromActor()]
     * @param InActor
     * @param InBlueprintClassName
     * @param bInSaveBP Whether or not saving the output BP to disk
     * @param InCDOFunc Callback to init CDO
     * @return UBlueprint*
     */
    static UBlueprint* CreateBlueprintFromActor(AActor* InActor,
                                                const FString& InBlueprintClassName,
                                                const bool bInSaveBP,
                                                const TFunction<void(UObject* InCDO)>& InCDOFunc = nullptr);
};
