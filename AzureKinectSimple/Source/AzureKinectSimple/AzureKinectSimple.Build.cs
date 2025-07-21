using System;
using System.IO;
using UnrealBuildTool;

public class AzureKinectSimple : ModuleRules
{
    public AzureKinectSimple(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "RHI", "RenderCore"
        });

        // Look up the SDK root
        string SDK = Environment.GetEnvironmentVariable("AZUREKINECT_SDK");
        if (string.IsNullOrEmpty(SDK))
        {
            throw new BuildException("AZUREKINECT_SDK environment variable not set. Point it to your Azure Kinect SDK install folder.");
        }

        // On Windows the headers live under sdk\include
        string IncludePath = Path.Combine(SDK, "sdk", "include");
        if (!Directory.Exists(IncludePath))
        {
            throw new BuildException($"Azure Kinect include path not found: {IncludePath}");
        }
        PublicIncludePaths.Add(IncludePath);

        // And link against k4a.lib in sdk\windows\lib
        string LibPath = Path.Combine(SDK, "sdk", "windows-desktop", "amd64", "release", "lib");
        PublicAdditionalLibraries.Add(Path.Combine(LibPath, "k4a.lib"));

        // At runtime UE needs to load k4a.dll
        PublicDelayLoadDLLs.Add("k4a.dll");
        RuntimeDependencies.Add(Path.Combine(SDK, "sdk", "windows-desktop", "amd64", "release", "bin", "k4a.dll"));
    }
}
