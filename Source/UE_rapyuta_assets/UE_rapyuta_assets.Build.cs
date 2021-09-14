// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

using UnrealBuildTool;

public class UE_rapyuta_assets : ModuleRules
{
    public UE_rapyuta_assets(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "AIModule", "rclUE" });

        PrivateDependencyModuleNames.AddRange(new string[] { });
            

    }
}
