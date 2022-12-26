/**
 * @file RapyutaSimulationPlugins.h
 * @brief Unreal Engine Mudule class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

#define RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME (TEXT("RapyutaSimulationPlugins"))
#define RAPYUTA_SIMULATION_PLUGINS_MODULE_FOLDER_NAME RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME

/**
 * @brief Unreal Engine Module class. This is used to initialize a module after
 * it's been loaded, and also to clean it up before the module is unloaded.
 * @sa[Unreal Engine IModuleInterface](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Modules/IModuleInterface/)
 */
class FRapyutaSimulationPluginsModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};

DECLARE_LOG_CATEGORY_EXTERN(LogRapyutaCore, Log, All);
