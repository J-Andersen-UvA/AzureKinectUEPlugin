#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <k4a/k4a.h>
#include "Runtime/Engine/Public/EngineGlobals.h"
#include "AzureKinectComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class AZUREKINECTSIMPLE_API UAzureKinectComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UAzureKinectComponent();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick Tick, FActorComponentTickFunction* ThisTickFunc) override;

    /** Exposed texture you can bind in UMG or Blueprint */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="AzureKinect")
    UTexture2D* ColorTexture = nullptr;

    /** Exposed depth array in millimeters */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="AzureKinect")
    TArray<FColor> DepthBuffer;

    /** Exposed depth Texture */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="AzureKinect")
    UTexture2D* DepthTexture = nullptr;


    /** Getter nodes for Blueprint graphs */
    UFUNCTION(BlueprintCallable, Category="AzureKinect")
    UTexture2D* GetColorTexture() const { return ColorTexture; }

    UFUNCTION(BlueprintCallable, Category="AzureKinect")
    void GetDepthData(TArray<FColor>& OutDepth) const
    {
        OutDepth = DepthBuffer;
    }

    UFUNCTION(BlueprintCallable, Category="AzureKinect")
    UTexture2D* GetDepthTexture() const { return DepthTexture; }


private:
    // Kinect handles
    k4a_device_t Device = nullptr;
    k4a_capture_t Capture = nullptr;

    // Internal raw buffer
    TArray<uint16> RawDepthBuffer;

    void InitializeTextures(int Width, int Height);
    void UpdateColor();
    void UpdateDepth();
};
