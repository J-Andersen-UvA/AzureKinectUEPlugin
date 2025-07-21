// AzureKinectSimple.cpp
#include "Modules/ModuleManager.h"

class FAzureKinectSimpleModule : public IModuleInterface
{
public:
    // Called right after the module DLL is loaded and before any UObject
    virtual void StartupModule() override
    {
        UE_LOG(LogTemp, Log, TEXT("AzureKinectSimple loaded"));
    }

    // Called before the module is unloaded, right before the DLL is freed
    virtual void ShutdownModule() override
    {
    }
};

IMPLEMENT_MODULE(FAzureKinectSimpleModule, AzureKinectSimple);
