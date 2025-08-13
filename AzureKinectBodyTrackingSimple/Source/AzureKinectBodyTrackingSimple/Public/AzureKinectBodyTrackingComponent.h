#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <k4a/k4a.h>
#include <k4abt.h>

#include "AzureActiveSelector.h"

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

UENUM(BlueprintType)
enum class EActiveSelectionMode : uint8
{
    Closest         UMETA(DisplayName = "Closest To Sensor"),
    WaveLastRaised  UMETA(DisplayName = "Last Hand-Above-Head")
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FAzureActiveBodyChanged, int32, OldBodyId, int32, NewBodyId);

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

    //UFUNCTION(BlueprintCallable, Category="Azure Kinect BT")
    //void setAzureCameraTransform(const FTransform& NewTransform);

    UFUNCTION(BlueprintCallable, Category="Azure Kinect BT")
	int32 getTrackedBodyCount()
	{
		return TrackedBodyCount;
	}

    /** The SDK’s persistent ID of the body we're currently tracking (or -1 if none) */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Azure Kinect BT")
    int32 TrackedBodyId = -1;

	// Flag to check if tracking is active
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Azure Kinect BT")
	bool bIsTracking = false;

    /** Number of tracked bodies, updated every frame */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category="Azure Kinect BT")
    int32 TrackedBodyCount = 0;

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT")
    FVector ComputeLookTargetFromKinectHead(
        const FVector& HeadPosMeters_Kinect,
        const FTransform& KinectToWorld,
        const FTransform& CameraWorld,
        const FVector& AvatarHeadWorld,
        float AimDistance = 1000.f) const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Azure Kinect BT|Active Selection")
    EActiveSelectionMode SelectionMode = EActiveSelectionMode::Closest;

    /** The currently "active" body id (may differ from TrackedBodyId in Wave mode). */
    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Azure Kinect BT|Active Selection")
    int32 ActiveBodyId = -1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Azure Kinect BT|Gesture")
    int32 AboveHeadMarginMM = 120; // ~12 cm

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Azure Kinect BT|Gesture")
    float RaiseHoldSeconds = 0.15f;

    /** After an active is not seen for this long, clear to -1. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Azure Kinect BT|Gesture")
    float ActiveStickySeconds = 2.0f;

    UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Azure Kinect BT|Active")
    bool bHasActive = false;

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT|Active Selection")
    void SetSelectionMode(EActiveSelectionMode NewMode) { SelectionMode = NewMode; }

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT|Active Selection")
    int32 GetActiveBodyId() const { return ActiveBodyId; }

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT|Active Selection")
    bool GetActiveBodySkeleton(TArray<FBodyJointData>& OutJoints) const;

    UFUNCTION(BlueprintCallable, Category = "Azure Kinect BT|Active")
    bool HasActive() const { return bHasActive; }

    UPROPERTY(BlueprintAssignable, Category = "Azure Kinect BT|Active")
    FAzureActiveBodyChanged OnActiveBodyChanged;

private:
    // Device handles for the Azure Kinect
    k4a_device_t Device = nullptr;
    k4a_capture_t Capture = nullptr;
    k4abt_tracker_t Tracker = nullptr;
    k4abt_frame_t FrameData = nullptr;
    k4abt_skeleton_t* BodySkeleton = nullptr;

    FAzureActiveSelector ActiveSelector;

    void findClosestTrackedBody();

    void UpdateActiveBodyFromFrame();         // called each Tick after we set FrameData
    bool GetSkeletonByBodyId(int32 BodyId, k4abt_skeleton_t& OutSkel) const;
    void SetActiveBody(int32 NewId);
};
