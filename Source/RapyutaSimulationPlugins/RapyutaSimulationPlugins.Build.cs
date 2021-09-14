// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

using UnrealBuildTool;

public class RapyutaSimulationPlugins : ModuleRules
{
    public RapyutaSimulationPlugins(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "AIModule", "rclUE" });

        PrivateDependencyModuleNames.AddRange(new string[] { });
            

    }
}
