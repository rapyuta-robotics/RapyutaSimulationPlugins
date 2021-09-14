// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "RapyutaSimulationPlugins.h"

#define LOCTEXT_NAMESPACE "FRapyutaSimulationPluginsModule"

void FRapyutaSimulationPluginsModule::StartupModule()
{
    // This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void FRapyutaSimulationPluginsModule::ShutdownModule()
{
    // This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
    // we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
DEFINE_LOG_CATEGORY(LogRapyutaCore);
IMPLEMENT_MODULE(FRapyutaSimulationPluginsModule, RapyutaSimulationPlugins)
