// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

using System;
using System.IO;
using UnrealBuildTool;

public class RapyutaSimulationPlugins : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }
    private string ThirdPartyPath
    {
        get { return Path.GetFullPath(Path.Combine(ModulePath, "../../ThirdParty/")); }
    }

    private string GetLibPath(string InLibName)
    {
        return Path.Combine(ThirdPartyPath, InLibName);
    }

    private void AddLib(string InLibPath, string InLibSymLinkName, string InLibName = "")
    {
        string libFullPath = Path.Combine(InLibPath, "release/lib", InLibSymLinkName);
        if (InLibSymLinkName.EndsWith(".a") || InLibSymLinkName.EndsWith(".so"))
        {
            PublicAdditionalLibraries.Add(libFullPath);
        }

        if (InLibName.Contains(".so"))
        {
            RuntimeDependencies.Add(Path.Combine("$(BinaryOutputDir)", InLibName), libFullPath, StagedFileType.NonUFS);
        }
    }

    public RapyutaSimulationPlugins(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp17;
        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "Renderer", "RHI", "AIModule", "PhysicsCore", "ProceduralMeshComponent",
                                                            "rclUE"});

        PrivateDependencyModuleNames.AddRange(new string[] { });

        // assimp
        bool bIsAssimpLibDynamic = false;
        string AssimpPath = GetLibPath("assimp");
        PublicIncludePaths.Add(Path.Combine(AssimpPath, "release/include"));
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            if (bIsAssimpLibDynamic)
            {
                // (Note) Release-build assimp version is not reliable yet, use debug one for now
                AddLib(AssimpPath, "libassimpd.so", "libassimpd.so.5");
            }
            else
            {
                AddLib(AssimpPath, "libassimp.a");
            }
        }

    }
}
