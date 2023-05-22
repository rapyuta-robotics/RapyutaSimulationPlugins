// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "Core/RRAssetUtils.h"

// UE
#include "CoreMinimal.h"
// RapyutaSimulationPlugins
#include "Core/RRGameSingleton.h"

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
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[%s] FAILED SAVING OBJECT TO UASSET - Error[%d]"),
               InObject ? *InObject->GetName() : *InPackage->GetName(),
               (uint8)result.Result);
        return false;
    }
}
