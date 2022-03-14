// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

#define RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME (TEXT("RapyutaSimulationPlugins"))
#define RAPYUTA_SIMULATION_PLUGINS_MODULE_FOLDER_NAME RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME
class FRapyutaSimulationPluginsModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};

DECLARE_LOG_CATEGORY_EXTERN(LogRapyutaCore, Log, All);
