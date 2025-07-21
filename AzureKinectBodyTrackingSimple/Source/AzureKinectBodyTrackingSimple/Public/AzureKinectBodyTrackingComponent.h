#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <k4a/k4a.h>
#include <k4abt.h>
#include "Runtime/Engine/Public/EngineGlobals.h"
#include "AzureKinectBodyTrackingComponent.generated.h"

USTRUCT(BlueprintType)
struct FBodyJointData
{
    GENERATED_BODY()

    // index into k4abt_joint_id_t
    UPROPERTY(BlueprintReadOnly, Category="Azure Kinect BT")
    int32 JointId;

    // in meters, your choice of axes
    UPROPERTY(BlueprintReadOnly, Category="Azure Kinect BT")
    FVector Position;

    // sensor-space orientation
    UPROPERTY(BlueprintReadOnly, Category="Azure Kinect BT")
    FQuat Orientation;
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class AZUREKINECTBODYTRACKINGSIMPLE_API UAzureKinectBodyTrackingComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UAzureKinectBodyTrackingComponent();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick Tick, FActorComponentTickFunction* ThisTickFunc) override;

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT")
    void startTracking();

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT")
    void stopTracking();

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT")
    void getBodySkeleton(TArray<FBodyJointData>& OutJoints) const;

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT")
    void getBoneData();

    UFUNCTION(BlueprintCallable, Category="Azure Kinect BT")
	int32 GetTrackedBodyCount()
	{
		return TrackedBodyCount;
	}

    UFUNCTION(BlueprintCallable, Category="Cry")
    void cry();

	// Flag to check if tracking is active
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Azure Kinect BT")
	bool bIsTracking = false;

    /** Number of tracked bodies, updated every frame */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Azure Kinect BT")
    int32 TrackedBodyCount = 0;

private:
    // Device handles for the Azure Kinect
    k4a_device_t Device = nullptr;
    k4a_capture_t Capture = nullptr;
    k4abt_tracker_t Tracker = nullptr;
    k4abt_frame_t FrameData = nullptr;
    k4abt_skeleton_t* BodySkeleton = nullptr;

    void findClosestTrackedBody();
};
