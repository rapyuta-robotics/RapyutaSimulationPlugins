// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

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

    private void AddLib(ReadOnlyTargetRules InTarget, string InLibPath, string InLibSymLinkName, string InLibName = "")
    {
        string libFullPath = Path.Combine(InLibPath, "release/lib", InLibSymLinkName);
        if (UnrealTargetPlatform.Linux == InTarget.Platform)
        {
            if (InLibSymLinkName.EndsWith(".a") || InLibSymLinkName.EndsWith(".so"))
            {
                PublicAdditionalLibraries.Add(libFullPath);
            }

            if (InLibName.Contains(".so"))
            {
                RuntimeDependencies.Add(Path.Combine("$(BinaryOutputDir)", InLibName), libFullPath, StagedFileType.NonUFS);
            }
        }
    }

    public RapyutaSimulationPlugins(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Latest;
        bEnableExceptions = true;

        // Runtime modules
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "RenderCore", "Renderer", "RHI", "PhysicsCore",
                                                            "ImageWrapper", "XmlParser", "Json", "PakFile", "IESFile",
                                                            "AIModule", "NavigationSystem", "TimeManagement", "UMG",
                                                            "ChaosVehicles",
                                                            "ProceduralMeshComponent", "MeshDescription", "StaticMeshDescription", "MeshConversion",
                                                            "rclUE"});

        PrivateDependencyModuleNames.AddRange(new string[] { });

        // Developer/Editor modules
        if (Target.bBuildEditor)
        {
            PrivateDependencyModuleNames.AddRange(
                new string[]
                {
                    "UnrealEd",
                    "AssetTools",
                    "PhysicsUtilities"
                }
            );
        }

        // assimp
        bool bIsAssimpLibDynamic = true;
        string AssimpPath = GetLibPath("assimp");
        PublicIncludePaths.Add(Path.Combine(AssimpPath, "release/include"));
        if (UnrealTargetPlatform.Linux == Target.Platform)
        {
            if (bIsAssimpLibDynamic)
            {
                AddLib(Target, AssimpPath, "libassimp.so", "libassimp.so.5");
            }
            else
            {
                AddLib(Target, AssimpPath, "libassimp.a");
            }
        }

        // ignition math
        string IgnMathPath = GetLibPath("ign-math");
        PublicIncludePaths.Add(Path.Combine(IgnMathPath, "release/include/ignition/math6"));
        bool bIsIgnMathLibDynamic = false;
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            if (bIsIgnMathLibDynamic)
            {
                AddLib(Target, IgnMathPath, "libignition-math6.so", "libignition-math6.so.6");
            }
            else
            {
                AddLib(Target, IgnMathPath, "libignition-math6.a");
            }
        }

        // ignition utils
        string IgnUtilsPath = GetLibPath("ign-utils");
        PublicIncludePaths.Add(Path.Combine(IgnUtilsPath, "release/include/ignition/utils1"));
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            AddLib(Target, IgnUtilsPath, "libignition-utils1.so", "libignition-utils1.so.1");
        }

        // sdformat
        string SdformatPath = GetLibPath("sdformat");
        PublicIncludePaths.Add(Path.Combine(SdformatPath, "release/include/sdformat-12.0"));
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            AddLib(Target, SdformatPath, "libsdformat12.so", "libsdformat12.so.12");
        }

        // two_point_interpolation
        string TwoPointsInterpolationPath = GetLibPath("two_points_interpolation_cpp");
        PublicIncludePaths.Add(TwoPointsInterpolationPath);

        // filter
        string FilterPath = GetLibPath("filter_cpp");
        PublicIncludePaths.Add(FilterPath);

    }
}
