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

    UPROPERTY(BlueprintReadOnly, Category="Azure Kinect BT")
    FString JointName;

    // in meters, your choice of axes
    UPROPERTY(BlueprintReadOnly, Category="Azure Kinect BT")
    FVector Position;

    // sensor-space orientation
    UPROPERTY(BlueprintReadOnly, Category="Azure Kinect BT")
    FQuat Orientation;
};

UENUM(BlueprintType)
enum class EAzureKinectJoint : uint8
{
    Pelvis = 0      UMETA(DisplayName="Pelvis"),
    SpineNaval      UMETA(DisplayName="Spine (Naval)"),
    SpineChest      UMETA(DisplayName="Spine (Chest)"),
    Neck            UMETA(DisplayName="Neck"),
    ClavicleLeft    UMETA(DisplayName="Clavicle Left"),
    ShoulderLeft    UMETA(DisplayName="Shoulder Left"),
    ElbowLeft       UMETA(DisplayName="Elbow Left"),
    WristLeft       UMETA(DisplayName="Wrist Left"),
    HandLeft        UMETA(DisplayName="Hand Left"),
    HandTipLeft     UMETA(DisplayName="Hand Tip Left"),
    ThumbLeft       UMETA(DisplayName="Thumb Left"),
    ClavicleRight   UMETA(DisplayName="Clavicle Right"),
    ShoulderRight   UMETA(DisplayName="Shoulder Right"),
    ElbowRight      UMETA(DisplayName="Elbow Right"),
    WristRight      UMETA(DisplayName="Wrist Right"),
    HandRight       UMETA(DisplayName="Hand Right"),
    HandTipRight    UMETA(DisplayName="Hand Tip Right"),
    ThumbRight      UMETA(DisplayName="Thumb Right"),
    HipLeft         UMETA(DisplayName="Hip Left"),
    KneeLeft        UMETA(DisplayName="Knee Left"),
    AnkleLeft       UMETA(DisplayName="Ankle Left"),
    FootLeft        UMETA(DisplayName="Foot Left"),
    HipRight        UMETA(DisplayName="Hip Right"),
    KneeRight       UMETA(DisplayName="Knee Right"),
    AnkleRight      UMETA(DisplayName="Ankle Right"),
    FootRight       UMETA(DisplayName="Foot Right"),
    Head            UMETA(DisplayName="Head"),
    Nose            UMETA(DisplayName="Nose"),
    EyeLeft         UMETA(DisplayName="Eye Left"),
    EarLeft         UMETA(DisplayName="Ear Left"),
    EyeRight        UMETA(DisplayName="Eye Right"),
    EarRight        UMETA(DisplayName="Ear Right")
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
    bool getBodySkeleton(TArray<FBodyJointData>& OutJoints) const;

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT")
    bool getBoneDataByName(const FString& BoneName, const TArray<FBodyJointData>& Joints, FBodyJointData& OutJointData) const;

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT")
    bool getBoneDataByEnum(EAzureKinectJoint JointEnum, const TArray<FBodyJointData>& Joints, FBodyJointData& OutJointData) const;

    /** The Kinect’s transform in world‐space (set from Blueprint). */
    UPROPERTY(BlueprintReadOnly, Category="Azure Kinect BT")
    FTransform AzureCameraTransform = FTransform::Identity;

    UFUNCTION(BlueprintCallable, Category="Azure Kinect BT")
    void setAzureCameraTransform(const FTransform& NewTransform);

    UFUNCTION(BlueprintCallable, Category="Azure Kinect BT")
	int32 getTrackedBodyCount()
	{
		return TrackedBodyCount;
	}

    /** The SDK’s persistent ID of the body we're currently tracking (or -1 if none) */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Azure Kinect BT")
    int32 TrackedBodyId = -1;

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
