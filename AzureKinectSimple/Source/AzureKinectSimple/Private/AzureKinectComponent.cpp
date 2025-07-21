#include "AzureKinectComponent.h"
#include "Engine/Texture2D.h"
#include "Rendering/Texture2DResource.h"
#include "Runtime/Engine/Public/EngineGlobals.h"

UAzureKinectComponent::UAzureKinectComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UAzureKinectComponent::BeginPlay()
{
    Super::BeginPlay();

    // only open the device if we are running a Play-in-Editor or Standalone game:
#if WITH_EDITOR
    // Only initialize when we’re actually running gameplay
    if (UWorld* World = GetWorld())
    {
        // UWorld::IsGameWorld() returns true for both Standalone and PIE
        if (!World->IsGameWorld())
        {
            return;
        }
    }
    else
    {
        return;
    }
#endif

    UE_LOG(LogTemp, Log, TEXT("Begin Play"));

    // Open device 0
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(0, &Device))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to open Azure Kinect"));
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("Opened Azure Kinect!"));

    // Configure: color + depth
    k4a_device_configuration_t Config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    Config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    Config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    Config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    k4a_device_start_cameras(Device, &Config);
}

void UAzureKinectComponent::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Device)
    {
        k4a_device_stop_cameras(Device);
        k4a_device_close(Device);
        Device = nullptr;
    }
    Super::EndPlay(Reason);
}

void UAzureKinectComponent::TickComponent(float DeltaTime, ELevelTick Tick, FActorComponentTickFunction* ThisTickFunc)
{
    Super::TickComponent(DeltaTime, Tick, ThisTickFunc);

    // 1) A quick on‑screen ping so you know Tick is happening:
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1, 0.2f, FColor::Cyan, TEXT("Kinect Ticking...!"));
    }

    // 2) Make sure device is open:
    if (!Device)
    {
        UE_LOG(LogTemp, Warning, TEXT("Kinect: device not open!"));
        return;
    }

    // 3) Wait up to 100ms for a new capture:
    k4a_wait_result_t Wait = k4a_device_get_capture(Device, &Capture, 100);
    if (Wait == K4A_WAIT_RESULT_TIMEOUT)
    {
        UE_LOG(LogTemp, Warning, TEXT("Kinect: no frame this tick (timeout)"));
        return;
    }
    else if (Wait != K4A_WAIT_RESULT_SUCCEEDED)
    {
        UE_LOG(LogTemp, Error, TEXT("Kinect: capture error %d"), (int)Wait);
        return;
    }

    UpdateColor();
    UpdateDepth();
    k4a_capture_release(Capture);
}

void UAzureKinectComponent::InitializeTextures(int Width, int Height)
{
    // Don't try to make a 0×0 texture!
    if (Width <= 0 || Height <= 0)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("AzureKinect: InitializeTextures called with invalid size %d×%d"),
            Width, Height);
        return;
    }

    // Create or update ColorTexture
    if (!ColorTexture)
    {
        ColorTexture = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8);
        ColorTexture->AddToRoot();
        ColorTexture->UpdateResource();
    }
}

void UAzureKinectComponent::UpdateColor()
{
    // — Color image —
    k4a_image_t ColorImg = k4a_capture_get_color_image(Capture);
    if (ColorImg)
    {
        const int32 W = k4a_image_get_width_pixels(ColorImg);
        const int32 H = k4a_image_get_height_pixels(ColorImg);
        uint8* ColorPtr = k4a_image_get_buffer(ColorImg);

        // Only proceed if we actually have pixels
        if (W > 0 && H > 0 && ColorPtr)
        {
            // Recreate the transient if size changed (or first time)
            if (!ColorTexture || ColorTexture->GetSizeX() != W || ColorTexture->GetSizeY() != H)
            {
                ColorTexture = UTexture2D::CreateTransient(W, H, PF_B8G8R8A8);
                ColorTexture->AddToRoot();
                ColorTexture->UpdateResource();
            }

            // Lock the texture's bulk data, memcpy, unlock, and push to GPU
            if (FTexturePlatformData* PlatData = ColorTexture->GetPlatformData())
            {
                if (PlatData->Mips.Num() > 0)
                {
                    // Grab the first mip
                    FTexture2DMipMap& Mip = PlatData->Mips[0];

                    // Lock for writing
                    void* Dest = Mip.BulkData.Lock(LOCK_READ_WRITE);
                    // Copy entire image (W*H pixels * 4 bytes/pixel)
                    FMemory::Memcpy(Dest, ColorPtr, W * H * 4);
                    Mip.BulkData.Unlock();

                    // Immediately update the resource so the GPU sees it
                    ColorTexture->UpdateResource();
                }
            }
        }

        k4a_image_release(ColorImg);
    }
}

void UAzureKinectComponent::UpdateDepth()
{
    k4a_image_t DepthImg = k4a_capture_get_depth_image(Capture);
    if (!DepthImg)
    {
        UE_LOG(LogTemp, Warning, TEXT("AzureKinect: no depth image in this capture"));
        return;
    }

    const int32 DepthW = k4a_image_get_width_pixels(DepthImg);
    const int32 DepthH = k4a_image_get_height_pixels(DepthImg);
    const int32 NumPixels = DepthW * DepthH;
    const uint16* DepthPtr = reinterpret_cast<uint16*>(k4a_image_get_buffer(DepthImg));

    if (!DepthPtr || NumPixels <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("AzureKinect: depth buffer invalid"));
        k4a_image_release(DepthImg);
        return;
    }

    // Resize buffers if needed
    if (DepthBuffer.Num() != NumPixels)
    {
        DepthBuffer.SetNumUninitialized(NumPixels);
    }

    // Convert depth to grayscale
    for (int32 i = 0; i < NumPixels; ++i)
    {
        uint16 depth = DepthPtr[i];

        // Example: Normalize and clamp (scale this appropriately to your depth range)
        uint8 grayscale = FMath::Clamp(depth / 10, 0, 255);

        DepthBuffer[i] = FColor(grayscale, grayscale, grayscale, 255);
    }

    if (!DepthTexture || DepthTexture->GetSizeX() != DepthW || DepthTexture->GetSizeY() != DepthH)
    {
        DepthTexture = UTexture2D::CreateTransient(DepthW, DepthH, PF_B8G8R8A8);
        DepthTexture->AddToRoot();
        DepthTexture->Filter = TF_Nearest;
        DepthTexture->SRGB = false;
        DepthTexture->UpdateResource();
    }

    for (int32 i = 0; i < NumPixels; ++i)
    {
        uint16 depth = DepthPtr[i];
        uint8 gray = FMath::Clamp(depth / 10, 0, 255);
        DepthBuffer[i] = FColor(gray, gray, gray, 255);
    }

    // Update raw texture memory
    FTexture2DMipMap& Mip = DepthTexture->GetPlatformData()->Mips[0];  // ✅ Requires full engine access
    void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
    FMemory::Memcpy(Data, DepthBuffer.GetData(), NumPixels * sizeof(FColor));
    Mip.BulkData.Unlock();

    DepthTexture->UpdateResource();


    k4a_image_release(DepthImg);
}
