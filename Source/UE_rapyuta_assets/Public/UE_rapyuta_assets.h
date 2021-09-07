// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FUE_rapyuta_assetsModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
