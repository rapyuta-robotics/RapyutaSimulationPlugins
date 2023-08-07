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
 * - Save asset with [UPackage](https://docs.unrealengine.com/4.27/en-US/API/Runtime/CoreUObject/UObject/UPackage/)
 * - Load asset meta data with [UObjectLibrary](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UObjectLibrary/)
 * @sa [AsyncLoading](https://docs.unrealengine.com/5.1/en-US/asynchronous-asset-loading-in-unreal-engine/)
 * @sa [AssetRegistry](https://docs.unrealengine.com/5.1/en-US/asset-registry-in-unreal-engine/)
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
        // FModuleManager::LoadModuleChecked are used internally
        static FAssetToolsModule& assetToolsModule = FAssetToolsModule::GetModule();
        return assetToolsModule.Get();
    }
#endif

    /**
     * @brief Get full asset file path on disk of an UE asset
     * @param InAssetPath UE asset path
     * @param OutAssetFilePath Output asset file path on disk (local path)
     * @return true if convertible
     */
    static bool GetAssetFilePathOnDisk(const FString& InAssetPath, FString& OutAssetFilePath)
    {
        return FPackageName::TryConvertLongPackageNameToFilename(
            InAssetPath, OutAssetFilePath, FPackageName::GetAssetPackageExtension());
    }

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
     * @sa [AsyncLoading](https://docs.unrealengine.com/5.1/en-US/asynchronous-asset-loading-in-unreal-engine/)
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
        ensure(IsAssetDataListValid(OutAssetDataList, bIsFullLoad, true));
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
     * @return T*
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
     * @brief Check whether an asset has been created at a path
     * @param InAssetPath Full UE asset path (eg: /Game/Contents/DynamicContents/SK_Obj)
     * @param bOnDiskAssetOnly
     * @return bool
     */
    static bool DoesAssetExist(const FString& InAssetPath, bool bOnDiskAssetOnly = false)
    {
        const auto& assetPackageName = FName(*InAssetPath);
        return bOnDiskAssetOnly ? GetAssetRegistry().GetAssetPackageDataCopy(assetPackageName).IsSet()
                                : GetAssetRegistry().HasAssets(assetPackageName);
    }

    /**
     * @brief Create a package meant to be saved to asset file on disk
     * @param InPackageName
     * @param InPackageFlags
     * @return UPackage*
     */
    static UPackage* CreatePackageForSavingToAsset(const TCHAR* InPackageName,
                                                   const EPackageFlags InPackageFlags = (PKG_NewlyCreated | PKG_RuntimeGenerated));

    /**
     * @brief Save object in memory to asset file on disk stored by a module
     * @param InObject
     * @param InAssetDataType
     * @param InAssetUniqueName Unique name for the output asset
     * @param InModuleName
     * @param bSaveDuplicatedObject
     * @param bInStripEditorOnlyContent
     * @param bInAsyncSave default true to avoid block-waiting on data writing to disk
     * @param bInAlwaysOverwrite default false to avoid liberty of overwriting
     * @return true if succeeded
     */
    static UObject* SaveObjectToAssetInModule(UObject* InObject,
                                              const ERRResourceDataType InAssetDataType,
                                              const FString& InAssetUniqueName,
                                              const TCHAR* InModuleName,
                                              bool bSaveDuplicatedObject = false,
                                              bool bInStripEditorOnlyContent = false,
                                              bool bInAsyncSave = true,
                                              bool bInAlwaysOverwrite = false);
    /**
     * @brief Save object in memory to asset file on disk
     * @param InObject
     * @param InAssetPath Full UE asset path of the output asset (eg: /Game/Contents/DynamicContents/SK_Obj)
     * @param bSaveDuplicatedObject
     * @param bInStripEditorOnlyContent
     * @param bInAsyncSave default true to avoid block-waiting on data writing to disk
     * @param bInAlwaysOverwrite default false to avoid liberty of overwriting
     * @return true if succeeded
     * @sa https://forums.unrealengine.com/t/calling-upackage-savepackage-causes-fatal-assert-in-staticfindobjectfast/447917
     * @sa https://forums.unrealengine.com/t/dynamically-created-primary-assets-not-registering-with-asset-manager/210255
     * @sa https://forums.unrealengine.com/t/how-to-work-with-cooked-content-in-editor/265094
     */
    static UObject* SaveObjectToAsset(UObject* InObject,
                                      const FString& InAssetPath,
                                      bool bSaveDuplicatedObject = false,
                                      bool bInStripEditorOnlyContent = false,
                                      bool bInAsyncSave = true,
                                      bool bInAlwaysOverwrite = false);

    /**
     * @brief Save package to asset file on disk
     * @param InObject
     * @param bInAsyncSave default true to avoid block-waiting on data writing to disk
     * @param bInAlwaysOverwrite default false to avoid liberty of overwriting
     * @return true if succeeded
     */
    static bool SavePackageToAsset(UPackage* InPackage,
                                   UObject* InObject,
                                   bool bInAsyncSave = true,
                                   bool bInAlwaysOverwrite = false);

    /**
     * @brief Find generated UClass from blueprint class name
     * @param InBlueprintClassName Either name or object path to blueprint class
     * @return UClass*
     * @note Refs: EditorUtilitySubsystem
     * @sa https://maladius.com/posts/asset_manager_1
     * @sa http://kantandev.com/articles/finding-all-classes-blueprints-with-a-given-base
     */
    static UClass* FindBlueprintClass(const FString& InBlueprintClassName);

    /* @brief Find UClass from its asset path name
     * @param InClassPathName, eg: '/Script/RapyutaSimulationPlugins.TurtlebotBurger'
     */
    static UClass* FindClassFromPathName(const FString& InClassPathName)
    {
        const FTopLevelAssetPath classAssetPath(InClassPathName);
        if (UClass* foundClass = FindObject<UClass>(classAssetPath))
        {
            return foundClass;
        }
        else
        {
            TArray<FAssetData> outAssets;
            GetAssetRegistry().GetAssetsByClass(classAssetPath, outAssets);
            return (outAssets.Num() > 0) ? outAssets[0].GetClass() : nullptr;
        }
    }

    /**
     * @brief Create a child blueprint class from parent UClass.
     * @param InParentClass
     * @param InBlueprintClassName
     * @param InCDOFunc Function to get Class Default Object(CDO) as arg.
     * @param bInSaveBP Whether or not saving the output BP to disk
     * @param InBPBasePath Base UE path for saving BP
     * @return UClass*
     * @sa [FKismetEditorUtilities::CreateBlueprintFromClass()]
     * @sa [Objects](https://docs.unrealengine.com/5.1/en-US/objects-in-unreal-engine/)
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
