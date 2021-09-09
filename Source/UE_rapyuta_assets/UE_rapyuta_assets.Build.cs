// Copyright (c) Rapyuta Robotics Co., Ltd.

using UnrealBuildTool;

public class UE_rapyuta_assets : ModuleRules
{
	public UE_rapyuta_assets(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"rclUE"
				// ... add other public dependencies that you statically link with here ...
			}
			);
			

	}
}
