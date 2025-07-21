#include <k4abt.h>      // for k4abt_tracker_create and k4abt_result_t
#include <k4a/k4a.h>   // for k4a_device_get_calibration, etc.
#include "AzureKinectBodyTrackingComponent.h"

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

    k4a_capture_release(sensorCapture);
}

void UAzureKinectBodyTrackingComponent::cry()
{
    UE_LOG(LogTemp, Error, TEXT("WAAAAAAAA!"));

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

void UAzureKinectBodyTrackingComponent::getBodySkeleton(TArray<FBodyJointData>& OutJoints) const
{
    OutJoints.Reset();

    // no frame? bail out
    if (!FrameData) return;

    // how many bodies?
    uint32 NumBodies = k4abt_frame_get_num_bodies(FrameData);
    if (NumBodies == 0) return;

    // grab skeleton for the first body
    k4abt_skeleton_t Skeleton;
    if (k4abt_frame_get_body_skeleton(FrameData, 0, &Skeleton) != K4A_RESULT_SUCCEEDED)
    {
        return;
    }

    // reserve space
    OutJoints.Reserve(K4ABT_JOINT_COUNT);

    // for each joint in the SDK enum
    for (int JointIndex = 0; JointIndex < K4ABT_JOINT_COUNT; ++JointIndex)
    {
        const auto& Src = Skeleton.joints[JointIndex];

        FBodyJointData Data;
        Data.JointId = JointIndex;
        // convert millimeters → meters, reorder if you like
        Data.Position = FVector(Src.position.xyz.x,
            Src.position.xyz.y,
            Src.position.xyz.z) * 0.001f;
        Data.Orientation = FQuat(Src.orientation.wxyz.x,
            Src.orientation.wxyz.y,
            Src.orientation.wxyz.z,
            Src.orientation.wxyz.w);

        OutJoints.Add(Data);
    }
}

void UAzureKinectBodyTrackingComponent::getBoneData()
{
    // Implementation for retrieving bone data
    UE_LOG(LogTemp, Log, TEXT("Retrieving bone data..."));

    // Here you would typically retrieve the bone data from the body tracking frame.
    // For example:
    // k4abt_frame_get_body_skeleton(BodyFrame, 0, &skeleton);
}

void UAzureKinectBodyTrackingComponent::findClosestTrackedBody()
{
    UE_LOG(LogTemp, Log, TEXT("Finding closest tracked body..."));
}