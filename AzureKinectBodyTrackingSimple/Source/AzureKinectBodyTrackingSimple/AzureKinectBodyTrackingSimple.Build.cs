using System;
using System.IO;
using UnrealBuildTool;

public class AzureKinectBodyTrackingSimple : ModuleRules
{
    public AzureKinectBodyTrackingSimple(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });

        // === sensor SDK (k4a) ===
        string SensorSDK = Environment.GetEnvironmentVariable("AZUREKINECT_SDK");
        if (string.IsNullOrEmpty(SensorSDK))
            throw new BuildException("AZUREKINECT_SDK not set.");

        string SensorInclude = Path.Combine(SensorSDK, "sdk", "include");
        PublicIncludePaths.Add(SensorInclude);

        string SensorLib = Path.Combine(SensorSDK, "sdk", "windows-desktop", "amd64", "release", "lib");
        PublicAdditionalLibraries.Add(Path.Combine(SensorLib, "k4a.lib"));

        // delay‑load the k4a DLL
        PublicDelayLoadDLLs.Add("k4a.dll");
        RuntimeDependencies.Add(
            Path.Combine(SensorSDK, "sdk", "windows-desktop", "amd64", "release", "bin", "k4a.dll"),
            StagedFileType.NonUFS // so it gets packaged
        );

        // === body‑tracking SDK (k4abt) ===
        string BodySDK = Environment.GetEnvironmentVariable("AZUREKINECT_BODY_SDK");
        if (string.IsNullOrEmpty(BodySDK))
            throw new BuildException("AZUREKINECT_BODY_SDK not set.");

        string BodyInclude = Path.Combine(BodySDK, "sdk", "include");
        PublicIncludePaths.Add(BodyInclude);

        string BodyLib = Path.Combine(BodySDK, "sdk", "windows-desktop", "amd64", "release", "lib");
        PublicAdditionalLibraries.Add(Path.Combine(BodyLib, "k4abt.lib"));

        // delay‑load the k4abt DLL
        PublicDelayLoadDLLs.Add("k4abt.dll");
        RuntimeDependencies.Add(
            Path.Combine(BodySDK, "sdk", "windows-desktop", "amd64", "release", "bin", "k4abt.dll"),
            StagedFileType.NonUFS
        );

        // depth‑engine lives under Tools/
        PublicDelayLoadDLLs.Add("depthengine_2_0.dll");
        RuntimeDependencies.Add(
            Path.Combine(BodySDK, "tools", "depthengine_2_0.dll"),
            StagedFileType.NonUFS
        );
    }
}
