// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRAssetUtils.h"

// UE
#include "CoreMinimal.h"

#if WITH_EDITOR
#include "BlueprintCompilationManager.h"
#if RAPYUTA_SIM_DEBUG
#include "EditorAssetLibrary.h"
#endif
#include "Kismet2/KismetEditorUtilities.h"
#include "KismetCompilerModule.h"
#endif

// RapyutaSimulationPlugins
#include "Core/RRGameSingleton.h"

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
    FString bpPackageName =
        FString::Printf(TEXT("%s/%s"),
                        (InBPBasePath.IsEmpty() ? *URRGameSingleton::Get()->ASSETS_RUNTIME_BP_SAVE_BASE_PATH : *InBPBasePath),
                        *InBlueprintClassName);
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
    UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("UClass runtime creation requires WITH_EDITOR"));
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
    FString bpPackageName =
        FString::Printf(TEXT("%s/%s"), *URRGameSingleton::Get()->ASSETS_RUNTIME_BP_SAVE_BASE_PATH, *InBlueprintClassName);
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

bool URRAssetUtils::SaveObjectToAssetInModule(UObject* InObject,
                                              const ERRResourceDataType InAssetDataType,
                                              const FString& InAssetUniqueName,
                                              const TCHAR* InModuleName,
                                              bool bInStripEditorOnlyContent)
{
    URRGameSingleton* gameSingleton = URRGameSingleton::Get();
    const FString baseAssetsPath = gameSingleton->GetDynamicAssetsBasePath(InModuleName);
    return URRAssetUtils::SaveObjectToAsset(
        InObject,
        baseAssetsPath / gameSingleton->GetAssetsFolderName(InAssetDataType) / InAssetUniqueName,
        bInStripEditorOnlyContent);
}

bool URRAssetUtils::SaveObjectToAsset(UObject* InObject, const FString& InAssetPath, bool bInStripEditorOnlyContent)
{
    // Compose package name
    FString uniquePackageName, uniqueAssetName;
    URRAssetUtils::GetAssetToolsModule().CreateUniqueAssetName(InAssetPath, TEXT(""), uniquePackageName, uniqueAssetName);

    // Create package wrapping [savedObject]
    UPackage* package = CreatePackage(*uniquePackageName);
    // NOTE: [GetMetaData()] is required to avoid error "Illegal call to StaticFindObjectFast()"
    package->GetMetaData();
    package->SetPackageFlags(PKG_NewlyCreated | PKG_RuntimeGenerated |
                             (bInStripEditorOnlyContent ? PKG_FilterEditorOnly : PKG_None));
    UObject* savedObject = StaticDuplicateObject(InObject, package, *uniqueAssetName);
    savedObject->SetFlags(EObjectFlags::RF_Public | EObjectFlags::RF_Standalone);
    savedObject->MarkPackageDirty();
    FAssetRegistryModule::AssetCreated(savedObject);
    package->MarkPackageDirty();

    // Save [package] to uasset file on disk
    return SavePackageToAsset(package, savedObject);
}

bool URRAssetUtils::SavePackageToAsset(UPackage* InPackage, UObject* InObject)
{
    // Save package args
    FSavePackageArgs saveArgs;
    saveArgs.TopLevelFlags = RF_Public | RF_Standalone;
    saveArgs.SaveFlags = SAVE_Async | SAVE_NoError | SAVE_KeepDirty | SAVE_FromAutosave | SAVE_KeepEditorOnlyCookedPackages;
    saveArgs.Error = GWarn;
    saveArgs.bWarnOfLongFilename = false;

    // Final output asset (package) full file name
    const FString packageFileName =
        FPackageName::LongPackageNameToFilename(InPackage->GetName(), FPackageName::GetAssetPackageExtension());
#if RAPYUTA_SIM_VERBOSE
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("Saving Asset [%s] Package [%s] -> PackageFile [%s]"),
           *savedObject->GetName(),
           *InPackage->GetName(),
           *packageFileName);
#endif
#if RAPYUTA_SIM_DEBUG
    // NOTE: This check unclear yet causes a crash on Subsystem, to be investigated
    if (UEditorAssetLibrary::DoesAssetExist(InPackage->GetName()))
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[%s] FAILED SAVING OBJECT TO UASSET, ALREADY EXISTING"),
               InObject ? *InObject->GetName() : *InPackage->GetName());
        return false;
    }
    else
#endif
    {
        // NOTE: [UPackage::Save()] is Runtime, while [SavePackageHelper(InPackage, packageFileName))] is Editor
        const FSavePackageResultStruct result = UPackage::Save(InPackage, InObject, *packageFileName, saveArgs);
        if (ESavePackageResult::Success == result.Result)
        {
            UPackage::WaitForAsyncFileWrites();
            UE_LOG(LogRapyutaCore,
                   Log,
                   TEXT("[%s] SAVED TO UASSET %s"),
                   InObject ? *InObject->GetName() : *InPackage->GetName(),
                   *packageFileName);
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
}
