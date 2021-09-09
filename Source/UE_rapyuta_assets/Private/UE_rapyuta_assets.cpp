// Copyright (c) Rapyuta Robotics Co., Ltd.

#include "UE_rapyuta_assets.h"

#define LOCTEXT_NAMESPACE "FUE_rapyuta_assetsModule"

void FUE_rapyuta_assetsModule::StartupModule()
{
    // This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void FUE_rapyuta_assetsModule::ShutdownModule()
{
    // This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
    // we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUE_rapyuta_assetsModule, UE_rapyuta_assets)
