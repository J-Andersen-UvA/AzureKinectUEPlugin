#include "AzureKinectBodyTrackingComponent.h"
#include <k4abt.h>      // for k4abt_tracker_create and k4abt_result_t
#include <k4a/k4a.h>   // for k4a_device_get_calibration, etc.
#include "AzureKinectLookSolver.h"
#include "AzureActiveSelector.h"
#include "AzureKinectSkeletonUtils.h"
#include "AzureBodyFrameUtils.h"

UAzureKinectBodyTrackingComponent::UAzureKinectBodyTrackingComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UAzureKinectBodyTrackingComponent::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Log, TEXT("BodyBT: Hello World"));

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
    ActiveSelector.Configure(AboveHeadMarginMM, RaiseHoldSeconds, ActiveStickySeconds);

    UE_LOG(LogTemp, Log, TEXT("BodyBT: BeginPlay"));

    // 1) Open the sensor:
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(0, &Device))
    {
        UE_LOG(LogTemp, Error, TEXT("BodyBT: k4a_device_open failed"));
        return;
    }

    // 2) Start its cameras (depth + color):
    k4a_device_configuration_t Config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    Config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    Config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(Device, &Config))
    {
        UE_LOG(LogTemp, Error, TEXT("BodyBT: k4a_device_start_cameras failed"));
        k4a_device_close(Device);
        Device = nullptr;
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("BodyBT: camera started"));
    startTracking();
}

void UAzureKinectBodyTrackingComponent::EndPlay(const EEndPlayReason::Type Reason)
{
    // 1) Tear down the tracker
    if (Tracker)
    {
        k4abt_tracker_shutdown(Tracker);
        k4abt_tracker_destroy(Tracker);
        Tracker = nullptr;
    }

    // 2) Stop & close the sensor
    if (Device)
    {
        k4a_device_stop_cameras(Device);
        k4a_device_close(Device);
        Device = nullptr;
    }

    Super::EndPlay(Reason);
}

void UAzureKinectBodyTrackingComponent::TickComponent(float DeltaTime, ELevelTick Tick, FActorComponentTickFunction* ThisTickFunc)
{
    Super::TickComponent(DeltaTime, Tick, ThisTickFunc);

    if (!bIsTracking || !Tracker || !Device)
    {
        UE_LOG(LogTemp, Error, TEXT("Not tracking or no tracker or no device!"));
        return;
    }

    k4a_capture_t sensorCapture = nullptr;
    if (k4a_device_get_capture(Device, &sensorCapture, 0) != K4A_WAIT_RESULT_SUCCEEDED)
    {
        return; // no new camera frame
    }

    if (k4abt_tracker_enqueue_capture(Tracker, sensorCapture, 0) != K4A_RESULT_SUCCEEDED)
    {
        UE_LOG(LogTemp, Error, TEXT("BodyBT: enqueue failed"));
        k4a_capture_release(sensorCapture);
        return;
    }

    k4abt_frame_t newBodyFrame = nullptr;
    if (k4abt_tracker_pop_result(Tracker, &newBodyFrame, 0) == K4A_RESULT_SUCCEEDED)
    {
        if (FrameData)
        {
            k4abt_frame_release(FrameData);
        }

        FrameData = newBodyFrame;
        TrackedBodyCount = static_cast<int32>(k4abt_frame_get_num_bodies(FrameData));
    }
    else
    {
        // no body frame ready yet
        TrackedBodyCount = 0;
    }

    // Update selection based on chosen mode
    UpdateActiveBodyFromFrame();

    // Release sensor capture
    k4a_capture_release(sensorCapture);
}

void UAzureKinectBodyTrackingComponent::UpdateActiveBodyFromFrame()
{
    const float Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.f;

    if (!FrameData)
    {
        SetActiveBody(-1);
        return;
    }

    // If we’re in Closest mode, mirror the old behavior for ActiveBodyId too
    if (SelectionMode == EActiveSelectionMode::Closest)
    {
        SetActiveBody(TrackedBodyId);
        return;
    }

    // WaveLastRaised: build a compact list of samples for this frame
    TArray<FAzureBodySample> Samples;
    const uint32 Num = k4abt_frame_get_num_bodies(FrameData);
    Samples.Reserve(Num);

    for (uint32 i = 0; i < Num; ++i)
    {
        const int32 BodyId = (int32)k4abt_frame_get_body_id(FrameData, i);
        if (BodyId < 0) continue;

        k4abt_skeleton_t Skel;
        if (k4abt_frame_get_body_skeleton(FrameData, i, &Skel) != K4A_RESULT_SUCCEEDED) continue;

        FAzureBodySample S;
        S.BodyId = BodyId;
        S.HeadY_mm = Skel.joints[K4ABT_JOINT_HEAD].position.xyz.y;
        S.LHandY_mm = Skel.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.y;
        S.RHandY_mm = Skel.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.y;
        S.SeenAtSeconds = Now;

        Samples.Add(S);
    }

    const int32 SuggestedId = ActiveSelector.UpdateWaveLastRaised(Samples, Now);
    SetActiveBody(SuggestedId); // this fires your Blueprint event and sets bHasActive
}

void UAzureKinectBodyTrackingComponent::startTracking()
{
    if (!Device)
    {
        UE_LOG(LogTemp, Error, TEXT("BodyBT: startTracking called before device open!"));
        return;
    }

    // 3) Grab the calibration from the live camera stream:
    k4a_calibration_t Calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(
        Device,
        K4A_DEPTH_MODE_NFOV_UNBINNED,
        K4A_COLOR_RESOLUTION_720P,
        &Calibration))
    {
        UE_LOG(LogTemp, Error, TEXT("BodyBT: k4a_device_get_calibration failed"));
        return;
    }

    // 4) Create the body tracker
    k4abt_tracker_configuration_t TrackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4a_result_t BodyRes = k4abt_tracker_create(&Calibration, TrackerConfig, &Tracker);
    if (BodyRes != K4A_RESULT_SUCCEEDED)
    {
        UE_LOG(LogTemp, Error, TEXT("BodyBT: k4abt_tracker_create failed (code = %d)"), (int)BodyRes);
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("BodyBT: tracker initialized!"));
    bIsTracking = true;
}


void UAzureKinectBodyTrackingComponent::stopTracking()
{
    // Implementation for stopping the body tracking
    UE_LOG(LogTemp, Log, TEXT("Stopping body tracking..."));

    // Here you would typically stop the body tracking process and clean up resources.
    // For example:
    // k4a_device_stop_body_tracker(Device);
    // k4a_device_close(Device);
}

bool UAzureKinectBodyTrackingComponent::getBodySkeleton(TArray<FBodyJointData>& OutJoints) const
{
    OutJoints.Reset();

    if (!FrameData)
    {
        UE_LOG(LogTemp, Warning, TEXT("BodyBT: no FrameData"));
        return false;
    }

    // how many bodies?
    uint32 NumBodies = k4abt_frame_get_num_bodies(FrameData);
    UE_LOG(LogTemp, Log, TEXT("BodyBT: NumBodies=%u TrackedBodyId=%d"), NumBodies, TrackedBodyId);
    if (NumBodies == 0) return false;

    // grab skeleton for the closest body
    k4abt_skeleton_t Skeleton;
    bool bFound = false;

    for (uint32 i = 0; i < NumBodies; ++i)
    {
        if (TrackedBodyId != static_cast<int32>(k4abt_frame_get_body_id(FrameData, i)))
        {
            continue;
        }

        if (k4abt_frame_get_body_skeleton(FrameData, i, &Skeleton) == K4A_RESULT_SUCCEEDED)
        {
            bFound = true;
        }

        break;
    }

    if (!bFound)
    {
        UE_LOG(LogTemp, Warning, TEXT("BodyBT: requested BodyId %d not in frame"), TrackedBodyId);
        return false;
    }

    AzureSkel::FillJointArrayFromSkeleton(Skeleton, AzureCameraTransform, OutJoints);
    return true;
}

bool UAzureKinectBodyTrackingComponent::getBoneDataByName(const FString& BoneName, const TArray<FBodyJointData>& Joints, FBodyJointData& OutJointData) const
{
    for (const FBodyJointData& Joint : Joints)
    {
        if (Joint.JointName.Equals(BoneName, ESearchCase::IgnoreCase))
        {
            OutJointData = Joint;
            return true;
        }
    }
    UE_LOG(LogTemp, Warning, TEXT("BodyBT: bone '%s' not found in provided skeleton"), *BoneName);
    return false;
}

bool UAzureKinectBodyTrackingComponent::getBoneDataByEnum(EAzureKinectJoint JointEnum, const TArray<FBodyJointData>& Joints, FBodyJointData& OutJointData) const
{
    int32 WantedId = static_cast<int32>(JointEnum);
    for (const FBodyJointData& Joint : Joints)
    {
        if (Joint.JointId == WantedId)
        {
            OutJointData = Joint;
            return true;
        }
    }
    UE_LOG(LogTemp, Warning, TEXT("BodyBT: joint enum '%d' not found in provided skeleton"), WantedId);
    return false;
}

void UAzureKinectBodyTrackingComponent::findClosestTrackedBody()
{
    TrackedBodyId = AzureFrame::FindClosestBodyId(FrameData);
}

static FORCEINLINE FVector AzureToUE_SensorLocal_cm(const FVector& p_m)
{
    return FVector(-p_m.X * 100.f,
        -p_m.Y * 100.f,
        p_m.Z * 100.f);
}

FVector UAzureKinectBodyTrackingComponent::ComputeLookTargetFromKinectHead(
    const FVector& HeadPosMeters_Kinect,
    const FTransform& KinectToWorld,
    const FTransform& CameraWorld,
    const FVector& AvatarHeadWorld,
    float AimDistance) const
{
    return AzureLook::ComputeLookTargetFromKinectHead(
        HeadPosMeters_Kinect, KinectToWorld, CameraWorld, AvatarHeadWorld, AimDistance);
}

int32 UAzureKinectBodyTrackingComponent::AmountOfSkeletonsTracked() const
{
    if (FrameData)
        return k4abt_frame_get_num_bodies(FrameData);
    return -1;
}

bool UAzureKinectBodyTrackingComponent::GetSkeletonByBodyId(int32 BodyId, k4abt_skeleton_t& OutSkel) const
{
    if (!FrameData || BodyId < 0) return false;

    const uint32 NumBodies = k4abt_frame_get_num_bodies(FrameData);
    for (uint32 i = 0; i < NumBodies; ++i)
    {
        uint32 ThisId = k4abt_frame_get_body_id(FrameData, i);
        if (ThisId == K4ABT_INVALID_BODY_ID) continue;
        if (static_cast<int32>(ThisId) != BodyId) continue;

        if (k4abt_frame_get_body_skeleton(FrameData, i, &OutSkel) == K4A_RESULT_SUCCEEDED)
        {
            return true;
        }
        break;
    }
    return false;
}

bool UAzureKinectBodyTrackingComponent::GetActiveBodySkeleton(TArray<FBodyJointData>& OutJoints) const
{
    if (SelectionMode == EActiveSelectionMode::Closest)
    {
        // Preserve current behavior for convenience (use closest)
        return getBodySkeleton(OutJoints);
    }

    k4abt_skeleton_t Skel;
    if (!GetSkeletonByBodyId(ActiveBodyId, Skel)) return false;

    AzureSkel::FillJointArrayFromSkeleton(Skel, AzureCameraTransform, OutJoints);
    return true;
}

void UAzureKinectBodyTrackingComponent::SetActiveBody(int32 NewId)
{
    if (ActiveBodyId == NewId) return;
    const int32 Old = ActiveBodyId;
    ActiveBodyId = NewId;
    bHasActive = (ActiveBodyId >= 0);
    OnActiveBodyChanged.Broadcast(Old, ActiveBodyId);
}

