/**
 * @file RRPakLoader.h
 * @brief Pak loader
 * @copyright Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "IPlatformFilePak.h"

// RapyutaSimulationPlugins
#include "RapyutaSimulationPlugins.h"

#include "RRPakLoader.generated.h"

class URRGameSingleton;

/**
 * @brief Pak loader
 * - Load pak files into [FPakPlatformFile](https://docs.unrealengine.com/5.2/en-US/API/Runtime/PakFile/FPakPlatformFile)
 * - Mount pak contents (as cooked resource assets) to a folder to be early loaded by URRGameSingleton during sim initialization
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRPakLoader : public UObject
{
    GENERATED_BODY()
public:
    /**
     * @brief Initialize #PakManager
     * @return true/false
     */
    bool Initialize();

    /**
     * @brief Load PAK files
     * @param InPakFolderPath
     */
    UFUNCTION()
    bool LoadPAKFiles(const FString& InPakFolderPath);

private:
    //! Pak file manager, responsible for loading & mounting paks
    FPakPlatformFile* PakManager = nullptr;

    UPROPERTY()
    TObjectPtr<URRGameSingleton> RRGameSingleton = nullptr;

    /**
     * @brief Mount PAK paths to files on disk
     * @param InPAKPaths
     */
    UFUNCTION()
    void MountPAKFiles(const TArray<FString>& InPAKPaths);
};
