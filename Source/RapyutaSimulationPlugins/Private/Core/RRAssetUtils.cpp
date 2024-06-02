// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRAssetUtils.h"

// Native
#include <type_traits>

// UE
#include "CoreMinimal.h"

#if WITH_EDITOR
#include "BlueprintCompilationManager.h"
#include "PackageHelperFunctions.h"
#if RAPYUTA_SIM_DEBUG
#include "EditorAssetLibrary.h"
#endif
#include "Kismet2/KismetEditorUtilities.h"
#include "KismetCompilerModule.h"
#endif

// RapyutaSimulationPlugins
#include "Core/RRGameSingleton.h"
#include "Core/RRThreadUtils.h"

using URRBlueprint = typename std::conditional<WITH_EDITOR, UBlueprint, UBlueprintGeneratedClass>::type;

UClass* URRAssetUtils::FindBlueprintClass(const FString& InBlueprintClassName)
{
    IAssetRegistry& assetRegistry = GetAssetRegistry();
    if (assetRegistry.IsLoadingAssets())
    {
        assetRegistry.SearchAllAssets(true);
    }

#if WITH_EDITOR
    // [UBlueprint]'s child-BP's type name does not have ending _C
    FString targetBPClassName = InBlueprintClassName;
    targetBPClassName.RemoveFromEnd(TEXT("_C"), ESearchCase::IgnoreCase);
#else
    // [UBlueprintGeneratedClass]'s child-BP's type name always ends with _C
    FString targetBPClassName =
        InBlueprintClassName.EndsWith(TEXT("_C")) ? InBlueprintClassName : FString::Printf(TEXT("%s_C"), *InBlueprintClassName);
#endif

    // Note: For the assets to be listed in Package build, Go to ProjectSettings to configure [PrimaryAssetTypesToScan].
    // Configuring PackagePaths here does not work
    // https://maladius.com/posts/asset_manager_1
    FARFilter filter;
    filter.bRecursivePaths = true;
    filter.bRecursiveClasses = true;
    filter.ClassPaths.Add(URRBlueprint::StaticClass()->GetClassPathName());

    // Find the blueprint asset of [InBlueprintClassName]
    UClass* foundBPClass = nullptr;
    assetRegistry.EnumerateAssets(
        filter,
        [&foundBPClass, &targetBPClassName](const FAssetData& InAssetData)
        {
            // NOTE: This lambda search for [InAssetData] that has name or object path as [targetBPClassName]
            // -> Return whether or not the searching should continue
            if ((InAssetData.AssetName.ToString() == targetBPClassName) || (InAssetData.GetObjectPathString() == targetBPClassName))
            {
                if (auto* bp = Cast<URRBlueprint>(InAssetData.GetAsset()))
                {
                    if constexpr (std::is_same<URRBlueprint, UBlueprint>::value)
                    {
                        foundBPClass = Cast<UBlueprint>(bp)->GeneratedClass;
                    }
                    else if constexpr (std::is_same<URRBlueprint, UBlueprintGeneratedClass>::value)
                    {
                        foundBPClass = Cast<UClass>(bp);
                    }
                    else
                    {
                        UE_LOG_WITH_INFO(LogRapyutaCore,
                                         Error,
                                         TEXT("[URRBlueprint] must be either [UBlueprint] or [UBlueprintGeneratedClass]"));
                    }
                    return false;
                }
            }
            return true;
        });

    return foundBPClass;
}

UClass* URRAssetUtils::CreateBlueprintClass(UClass* InParentClass,
                                            const FString& InBlueprintClassName,
                                            const TFunction<void(UObject* InCDO)>& InCDOFunc,
                                            const bool bInSaveBP,
                                            const FString& InBPBasePath)
{
#if WITH_EDITOR
    static IKismetCompilerInterface& kismetCompilerModule =
        FModuleManager::LoadModuleChecked<IKismetCompilerInterface>(KISMET_COMPILER_MODULENAME);
    static FAssetToolsModule& assetToolsModule = FAssetToolsModule::GetModule();

    // 1 - Create blueprint class & generated class
    UClass* blueprintClass = nullptr;
    UClass* blueprintGeneratedClass = nullptr;
    kismetCompilerModule.GetBlueprintTypesForClass(InParentClass, blueprintClass, blueprintGeneratedClass);

    // 2- Create blueprint package for the class asset
    FString bpPackageName = InBPBasePath.IsEmpty() ? URRGameSingleton::Get()->GetDynamicBPAssetPath(InBlueprintClassName)
                                                   : InBPBasePath / InBlueprintClassName;
    FString bpAssetName;
    assetToolsModule.Get().CreateUniqueAssetName(bpPackageName, TEXT(""), bpPackageName, bpAssetName);
    UPackage* bpPackage = CreatePackage(*bpPackageName);
    ensure(bpPackage);
    bpPackage->SetPackageFlags(PKG_NewlyCreated | PKG_RuntimeGenerated);

    // 3- Create and init a new Blueprint
    UBlueprint* blueprint = FKismetEditorUtilities::CreateBlueprint(
        InParentClass, bpPackage, *InBlueprintClassName, EBlueprintType::BPTYPE_Normal, blueprintClass, blueprintGeneratedClass);
    if (blueprint)
    {
        // 4- Make sure blueprint is not early GCed
        blueprint->SetFlags(EObjectFlags::RF_Standalone);

        // Notify the asset registry
        FAssetRegistryModule::AssetCreated(blueprint);

        // Mark the package dirty
        bpPackage->MarkPackageDirty();

        // 4.1- Compile the blueprint, required before creating its CDO
        // Skip validation of the class default object here for reasons:
        // (1) The new CDO may fail validation because this is a new Blueprint having no modified defaults.
        // (2) Default value propagation to the new Blueprint's CDO may be deferred until after compilation (e.g. reparenting).
        // Skip the Blueprint search data update, which will be handled by an OnAssetAdded() delegate in the FiB manager.
        const EBlueprintCompileOptions bpCompileOptions =
            EBlueprintCompileOptions::SkipGarbageCollection | EBlueprintCompileOptions::SkipDefaultObjectValidation |
            EBlueprintCompileOptions::SkipFiBSearchMetaUpdate | EBlueprintCompileOptions::SkipNewVariableDefaultsDetection;
        FKismetEditorUtilities::CompileBlueprint(blueprint, bpCompileOptions, nullptr);

        // NOTE: This is different from [blueprintGeneratedClass] above, which is [UBlueprintGeneratedClass::StaticClass()]
        auto& bpGeneratedClass = blueprint->GeneratedClass;
        if (nullptr == bpGeneratedClass)
        {
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("After compiling [blueprint->GeneratedClass] is null"));
            return nullptr;
        }
#if RAPYUTA_SIM_DEBUG
        UE_LOG(LogTemp,
               Warning,
               TEXT("bpGeneratedClass[%s] # blueprintGeneratedClass[%s]"),
               *bpGeneratedClass->GetName(),
               *blueprintGeneratedClass->GetName());
#endif

        // 4.2- Create its CDO
        UObject* cdo = bpGeneratedClass->GetDefaultObject();

        // 5- Call [InCDOFunc] on CDO
        if (InCDOFunc)
        {
            InCDOFunc(cdo);
            // 5.1- Compile BP again after modifying CDO
            FKismetEditorUtilities::CompileBlueprint(blueprint, bpCompileOptions, nullptr);
        }

        // 6- Save [bpPackage] to disk
        if (bInSaveBP)
        {
            URRAssetUtils::SavePackageToAsset(bpPackage, blueprint);
        }

        return bpGeneratedClass;
    }
#else
    UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("UClass runtime creation is Editor-only"));
#endif
    return nullptr;
}

UBlueprint* URRAssetUtils::CreateBlueprintFromActor(AActor* InActor,
                                                    const FString& InBlueprintClassName,
                                                    const bool bInSaveBP,
                                                    const TFunction<void(UObject* InCDO)>& InCDOFunc)
{
#if WITH_EDITOR
    // 1- Create blueprint package as child class of [InActor]'s GetClass()
    FString bpPackageName = URRGameSingleton::Get()->GetDynamicBPAssetPath(InBlueprintClassName);
    FString bpAssetName;
    URRAssetUtils::GetAssetToolsModule().CreateUniqueAssetName(bpPackageName, TEXT(""), bpPackageName, bpAssetName);

    // 1.1 - Create the blueprint from this robot
    FKismetEditorUtilities::FCreateBlueprintFromActorParams params;
    params.bReplaceActor = false;
    params.bKeepMobility = true;
    params.bDeferCompilation = false;
    params.bOpenBlueprint = false;
    // NOTE: [UPackage] for the blueprint is already created here-in
    UBlueprint* blueprint = FKismetEditorUtilities::CreateBlueprintFromActor(bpPackageName, InActor, params);

    // 2- Compile the blueprint, required before creating its CDO
    const EBlueprintCompileOptions bpCompileOptions =
        EBlueprintCompileOptions::SkipGarbageCollection | EBlueprintCompileOptions::SkipDefaultObjectValidation |
        EBlueprintCompileOptions::SkipFiBSearchMetaUpdate | EBlueprintCompileOptions::SkipNewVariableDefaultsDetection;
    FKismetEditorUtilities::CompileBlueprint(blueprint, bpCompileOptions);

    auto& bpGeneratedClass = blueprint->GeneratedClass;
    if (nullptr == bpGeneratedClass)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("After compiling [blueprint->GeneratedClass] is null"));
        return nullptr;
    }

    // 3- Create its CDO
    UObject* cdo = bpGeneratedClass->GetDefaultObject();

    // 3.1- Init its CDO
    if (InCDOFunc)
    {
        InCDOFunc(cdo);
        // 4.1- Compile BP again after modifying CDO
        FKismetEditorUtilities::CompileBlueprint(blueprint, bpCompileOptions);
    }

    // 4- Save [blueprint]'s package to uasset on disk
    if (bInSaveBP)
    {
        URRAssetUtils::SavePackageToAsset(blueprint->GetPackage(), blueprint);
    }
#else
    UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("UBlueprint runtime creation requires WITH_EDITOR"));
#endif
    return nullptr;
}

UPackage* URRAssetUtils::CreatePackageForSavingToAsset(const TCHAR* InPackageName, const EPackageFlags InPackageFlags)
{
#if WITH_EDITOR
    UPackage* package = CreatePackage(InPackageName);
    // NOTE: [GetMetaData()] is required to avoid error "Illegal call to StaticFindObjectFast()" during package saving later
    package->GetMetaData();
    package->SetPackageFlags(InPackageFlags);
    return package;
#else
    UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Package creation is Editor-only"));
    return nullptr;
#endif
}

UObject* URRAssetUtils::SaveObjectToAssetInModule(UObject* InObject,
                                                  const ERRResourceDataType InAssetDataType,
                                                  const FString& InAssetUniqueName,
                                                  const TCHAR* InModuleName,
                                                  bool bSaveDuplicatedObject,
                                                  bool bInStripEditorOnlyContent,
                                                  bool bInAsyncSave,
                                                  bool bInAlwaysOverwrite)
{
#if WITH_EDITOR
    return URRAssetUtils::SaveObjectToAsset(
        InObject,
        URRGameSingleton::Get()->GetDynamicAssetPath(InAssetDataType, InAssetUniqueName, InModuleName),
        bSaveDuplicatedObject,
        bInStripEditorOnlyContent,
        bInAsyncSave,
        bInAlwaysOverwrite);
#else
    UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Editor-only api!"));
    return nullptr;
#endif
}

UObject* URRAssetUtils::SaveObjectToAsset(UObject* InObject,
                                          const FString& InAssetPath,
                                          bool bSaveDuplicatedObject,
                                          bool bInStripEditorOnlyContent,
                                          bool bInAsyncSave,
                                          bool bInAlwaysOverwrite)
{
#if WITH_EDITOR
    // Compose package name
    FString uniquePackageName, uniqueAssetName;
    if (DoesAssetExist(InAssetPath))
    {
#if RAPYUTA_SIM_DEBUG && WITH_EDITOR
        URRAssetUtils::GetAssetToolsModule().CreateUniqueAssetName(InAssetPath, TEXT(""), uniquePackageName, uniqueAssetName);
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Warning,
                         TEXT("[%s] ASSET ALREADY EXISTS -> [%s] is used as the saved asset name for [%s]"),
                         *InAssetPath,
                         *uniqueAssetName,
                         *InObject->GetName());
#else
        if (bInAlwaysOverwrite)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Warning,
                             TEXT(" [%s]->[%s] ASSET ALREADY EXISTS -> WILL BE OVERWRITTEN"),
                             *InObject->GetName(),
                             *InAssetPath);
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Error,
                             TEXT("[%s] ASSET ALREADY EXISTS -> cannot create asset of that name from [%s]"),
                             *InAssetPath,
                             *InObject->GetName());
            return nullptr;
        }
#endif
    }
    uniquePackageName = InAssetPath;
    uniqueAssetName = FPaths::GetBaseFilename(InAssetPath);

    // Create package wrapping [savedObject]
    UPackage* package = CreatePackageForSavingToAsset(
        *uniquePackageName,
        PKG_NewlyCreated | PKG_RuntimeGenerated | (bInStripEditorOnlyContent ? PKG_FilterEditorOnly : PKG_None));

    if (nullptr == package)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("[%s] UNABLE TO CREATE PACKAGE FOR [%s]"), *InAssetPath, *InObject->GetName());
        return nullptr;
    }

    // Configure [savedObject]
    UObject* savedObject = nullptr;
    if (bSaveDuplicatedObject)
    {
        savedObject = StaticDuplicateObject(InObject, package, *uniqueAssetName);
    }
    else
    {
        // NOTE: Re-referencing might be required if [InObject] has no other assets referencing it. Ref: [RetrieveReferencers()]
        InObject->Rename(*uniqueAssetName, package);
        savedObject = InObject;
    }
    savedObject->SetFlags(EObjectFlags::RF_Public | EObjectFlags::RF_Standalone);
    FAssetRegistryModule::AssetCreated(savedObject);

    // Save [package] to uasset file on disk
    return SavePackageToAsset(package, savedObject, bInAsyncSave, bInAlwaysOverwrite) ? savedObject : nullptr;
#else
    UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Editor-only api!"));
    return nullptr;
#endif
}

bool URRAssetUtils::SavePackageToAsset(UPackage* InPackage, UObject* InObject, bool bInAsyncSave, bool bInAlwaysOverwrite)
{
#if WITH_EDITOR
    // Mark both dirty
    if (InObject)
    {
        InObject->MarkPackageDirty();
    }
    InPackage->MarkPackageDirty();

    // Save package args
    FSavePackageArgs saveArgs;
    saveArgs.TopLevelFlags = RF_Public | RF_Standalone;
    saveArgs.SaveFlags = (bInAsyncSave ? SAVE_Async : SAVE_None) | SAVE_NoError | SAVE_KeepDirty | SAVE_FromAutosave |
                         SAVE_KeepEditorOnlyCookedPackages;
    saveArgs.Error = GWarn;
    saveArgs.bWarnOfLongFilename = false;

    // Final output asset (package) full file name
    const FString packageFileName =
        FPackageName::LongPackageNameToFilename(InPackage->GetName(), FPackageName::GetAssetPackageExtension());
#if RAPYUTA_SIM_VERBOSE
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("Saving Asset [%s] Package [%s] -> PackageFile [%s]"),
           InObject ? *InObject->GetName() : EMPTY_STR,
           *InPackage->GetName(),
           *packageFileName);
#endif
    const bool bAlreadyExistOnDisk = FPaths::FileExists(packageFileName);
    if ((false == bInAlwaysOverwrite) && bAlreadyExistOnDisk)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("[%s] FAILED SAVING OBJECT TO UASSET, ALREADY EXISTS"), *packageFileName);
        return false;
    }
    else
    {
        // Make sure [InPackage] is [InObject]'s package
        if (InObject->GetPackage() != InPackage)
        {
            InObject->Rename(nullptr, InPackage);
        }

        // Make sure [InObject] has RF_Transient cleared
        InObject->ClearFlags(EObjectFlags::RF_Transient);

        // NOTE: [UPackage::Save()] is in-Engine api, while [SavePackageHelper(InPackage, packageFileName))] is in-Editor
        // Use [result] to print error code later
        const FSavePackageResultStruct result = UPackage::Save(InPackage, InObject, *packageFileName, saveArgs);
        if (ESavePackageResult::Success == result.Result)
        {
            if (bInAsyncSave)
            {
                URRThreadUtils::DoAsyncTaskInThread<void>([]() { UPackage::WaitForAsyncFileWrites(); },
                                                          [packageFileName, bAlreadyExistOnDisk]() {
                                                              UE_LOG(LogRapyutaCore,
                                                                     Warning,
                                                                     TEXT("SAVED UASSET %s - Overwritten %d"),
                                                                     *packageFileName,
                                                                     bAlreadyExistOnDisk);
                                                          });
            }
            else
            {
                UE_LOG_WITH_INFO(LogRapyutaCore,
                                 Log,
                                 TEXT("[%s] -> %s"),
                                 InObject ? *InObject->GetName() : *InPackage->GetName(),
                                 *packageFileName);
            }
            return true;
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Error,
                             TEXT("[%s] FAILED SAVING OBJECT TO UASSET - Error[%d]"),
                             InObject ? *InObject->GetName() : *InPackage->GetName(),
                             (uint8)result.Result);
            return false;
        }
    }
#else
    UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Editor-only api!"));
    return false;
#endif
}
