// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

using UnrealBuildTool;

public class RapyutaSimulationPlugins : ModuleRules
{
    public RapyutaSimulationPlugins(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp17;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "Renderer", "RHI", "AIModule", "PhysicsCore",
                                                            "rclUE"});

        PrivateDependencyModuleNames.AddRange(new string[] { });
            

    }
}
