// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"
#define UE_RAPYUTA_ASSETS_MODULE_NAME (TEXT("UE_rapyuta_assets"))
class FRapyutaSimulationPluginsModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
