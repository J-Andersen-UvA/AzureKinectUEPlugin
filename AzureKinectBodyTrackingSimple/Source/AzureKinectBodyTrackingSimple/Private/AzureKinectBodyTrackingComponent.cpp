#include "AzureKinectBodyTrackingComponent.h"
#include <k4abt.h>      // for k4abt_tracker_create and k4abt_result_t
#include <k4a/k4a.h>   // for k4a_device_get_calibration, etc.

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

    // Update selection based on chosen mode
    UpdateActiveBodyFromFrame();

    // Keep your old closest logic for backward compat / debug
    findClosestTrackedBody();

    // Release sensor capture
    k4a_capture_release(sensorCapture);
}

void UAzureKinectBodyTrackingComponent::UpdateActiveBodyFromFrame()
{
    const float Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.f;

    if (!FrameData)
    {
        ActiveBodyId = -1;
        return;
    }

    // If we’re in Closest mode, mirror the old behavior for ActiveBodyId too
    if (SelectionMode == EActiveSelectionMode::Closest)
    {
        ActiveBodyId = TrackedBodyId;
        return;
    }

    // WaveLastRaised mode
    const uint32 NumBodies = k4abt_frame_get_num_bodies(FrameData);
    bool bAnySeen = false;

    for (uint32 i = 0; i < NumBodies; ++i)
    {
        const uint32 BodyIdU = k4abt_frame_get_body_id(FrameData, i);
        if (BodyIdU == K4ABT_INVALID_BODY_ID) continue;
        const int32 BodyId = static_cast<int32>(BodyIdU);
        bAnySeen = true;

        k4abt_skeleton_t Skel;
        if (k4abt_frame_get_body_skeleton(FrameData, i, &Skel) != K4A_RESULT_SUCCEEDED)
        {
            continue;
        }

        // Azure camera frame (mm): +Y is down. "Above head" => handY < headY by a margin.
        const float HeadY = Skel.joints[K4ABT_JOINT_HEAD].position.xyz.y;
        const float LHandY = Skel.joints[K4ABT_JOINT_HAND_LEFT].position.xyz.y;
        const float RHandY = Skel.joints[K4ABT_JOINT_HAND_RIGHT].position.xyz.y;

        const bool LeftAbove = (HeadY - LHandY) > AboveHeadMarginMM;
        const bool RightAbove = (HeadY - RHandY) > AboveHeadMarginMM;

        FBodyRaiseState& S = BodyStates.FindOrAdd(BodyId);
        const bool LeftRising = (!S.bLeftAbove && LeftAbove);
        const bool RightRising = (!S.bRightAbove && RightAbove);

        // Optionally require the above condition to hold briefly
        // We can keep a tiny “debounce” by only accepting when it stays above across frames,
        // but a simple rising-edge + small BlendIn on montages is usually enough.
        if (LeftRising || RightRising)
        {
            S.LastRaiseTime = Now;
        }

        S.bLeftAbove = LeftAbove;
        S.bRightAbove = RightAbove;
        S.LastSeenTime = Now;

        // Commit selection on rising edge (or if still held after RaiseHoldSeconds)
        const bool JustRaised = (LeftRising || RightRising);
        const bool HeldRaise = (LeftAbove || RightAbove) && (Now - S.LastRaiseTime >= RaiseHoldSeconds);

        if (JustRaised || HeldRaise)
        {
            ActiveBodyId = BodyId; // “last raise wins”
        }
    }

    // If our active body disappeared, drop it after a short sticky time, then fallback
    if (ActiveBodyId != -1)
    {
        if (FBodyRaiseState* SActive = BodyStates.Find(ActiveBodyId))
        {
            if ((Now - SActive->LastSeenTime) > ActiveStickySeconds)
            {
                ActiveBodyId = -1;
            }
        }
        else
        {
            ActiveBodyId = -1;
        }
    }

    // Fallback to closest if none active or nobody in view
    if (ActiveBodyId == -1)
    {
        findClosestTrackedBody();
        ActiveBodyId = TrackedBodyId;
    }

    // Optional: prune stale entries
    for (auto It = BodyStates.CreateIterator(); It; ++It)
    {
        if ((Now - It->Value.LastSeenTime) > 5.f)
        {
            It.RemoveCurrent();
        }
    }
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

    // reserve space
    OutJoints.Reserve(K4ABT_JOINT_COUNT);

    // for each joint in the SDK enum
    for (int JointIndex = 0; JointIndex < K4ABT_JOINT_COUNT; ++JointIndex)
    {
        const auto& Src = Skeleton.joints[JointIndex];

        FBodyJointData Data;
        Data.JointId = JointIndex;

        // Get Joint name
        EAzureKinectJoint JointEnum = static_cast<EAzureKinectJoint>(Data.JointId);
        const UEnum* UEEnum = StaticEnum<EAzureKinectJoint>();
        Data.JointName = UEEnum->GetDisplayNameTextByValue((int64)JointEnum).ToString();

        // 1) convert millimeters → unreal cm, reorder if you like
        FVector LocalCm = FVector(
            Src.position.xyz.x * 0.1f,
            Src.position.xyz.y * 0.1f,
            Src.position.xyz.z * 0.1f);

        // 2) remap into Unreal local axes:
        //    Unreal.X = Kinect.Z
        //    Unreal.Y = Kinect.X
        //    Unreal.Z = –Kinect.Y
        FVector LocalUnrealCm = FVector(
            LocalCm.Z,         // forward
            LocalCm.X,         // right
            LocalCm.Y          // up
        );

        FVector WorldCm = AzureCameraTransform.TransformPosition(LocalUnrealCm);
        Data.Position = WorldCm;

        FQuat Qkinect(
            Src.orientation.wxyz.x,
            Src.orientation.wxyz.y,
            Src.orientation.wxyz.z,
            Src.orientation.wxyz.w
        );
        static const FMatrix RemapMatrix = FMatrix(
            FPlane(0, 0, 1, 0),
            FPlane(1, 0, 0, 0),
            FPlane(0, -1, 0, 0),
            FPlane(0, 0, 0, 1)
        );
        const FQuat RemapQuat(RemapMatrix);
        const FQuat RemapQuatInv = RemapQuat.Inverse();
        FQuat Qlocal = RemapQuat * Qkinect * RemapQuatInv;
        FQuat Qworld = AzureCameraTransform.GetRotation() * Qlocal;
        Data.Orientation = Qworld;

        OutJoints.Add(Data);
    }

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
    TrackedBodyId = -1;  // reset

    if (!FrameData)
    {
        UE_LOG(LogTemp, Warning, TEXT("BodyBT: no frame to search"));
        return;
    }

    const uint32 NumBodies = k4abt_frame_get_num_bodies(FrameData);
    if (NumBodies == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("BodyBT: no bodies in frame"));
        return;
    }

    float BestDistSq = FLT_MAX;

    for (uint32 i = 0; i < NumBodies; ++i)
    {
        uint32 bodyID = k4abt_frame_get_body_id(FrameData, i);
        if (bodyID == K4ABT_INVALID_BODY_ID)
        {
            break;
        }

        k4abt_skeleton_t Skeleton;
        if (k4abt_frame_get_body_skeleton(FrameData, i, &Skeleton) != K4A_RESULT_SUCCEEDED)
        {
            continue;
        }

        // Read the pelvis joint as our “root” point
        const auto& Pelvis = Skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz;
        FVector PosMeters(
            Pelvis.x * 0.001f,
            Pelvis.y * 0.001f,
            Pelvis.z * 0.001f
        );

        // Compute squared distance to origin (sensor at 0,0,0)
        const float DistSq = PosMeters.SizeSquared();

        if (DistSq < BestDistSq)
        {
            BestDistSq = DistSq;
            TrackedBodyId = static_cast<int32>(bodyID);
        }
    }
}

static FORCEINLINE FVector AzureToUE_SensorLocal_cm(const FVector& p_m)
{
    //return FVector(p_m.Z * 100.f,  // +X forward
    //    p_m.X * 100.f,  // +Y right
    //    p_m.Y * 100.f); // +Z up
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
    const FVector HeadWorld = KinectToWorld.TransformPosition(AzureToUE_SensorLocal_cm(HeadPosMeters_Kinect));
    const FVector HeadInCam = CameraWorld.InverseTransformPosition(HeadWorld);

    FVector DirCam = HeadInCam.GetSafeNormal();
    // DirCam.Y *= -1; // mirror feel, optional

    const FVector WorldPointOnRay = CameraWorld.GetLocation() +
        CameraWorld.TransformVectorNoScale(DirCam) * 1000.f;
    const FVector AimDirWorld = (WorldPointOnRay - AvatarHeadWorld).GetSafeNormal();
    return AvatarHeadWorld + AimDirWorld * AimDistance;
}

int32 UAzureKinectBodyTrackingComponent::AmountOfSkeletonsTracked() const
{
    if (FrameData)
        return k4abt_frame_get_num_bodies(FrameData);
    else
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

    OutJoints.Reset();
    OutJoints.Reserve(K4ABT_JOINT_COUNT);

    // Convert to your existing FBodyJointData layout & remap (reuse your getBodySkeleton code)
    for (int JointIndex = 0; JointIndex < K4ABT_JOINT_COUNT; ++JointIndex)
    {
        const auto& Src = Skel.joints[JointIndex];

        FBodyJointData Data;
        Data.JointId = JointIndex;

        EAzureKinectJoint JointEnum = static_cast<EAzureKinectJoint>(Data.JointId);
        const UEnum* UEEnum = StaticEnum<EAzureKinectJoint>();
        Data.JointName = UEEnum->GetDisplayNameTextByValue((int64)JointEnum).ToString();

        // mm -> cm (same as your getBodySkeleton)
        FVector LocalCm(
            Src.position.xyz.x * 0.1f,
            Src.position.xyz.y * 0.1f,
            Src.position.xyz.z * 0.1f);

        // your remap (note: you currently have Y not negated — keep consistent with your pipeline)
        FVector LocalUnrealCm(
            LocalCm.Z,
            LocalCm.X,
            LocalCm.Y
        );

        FVector WorldCm = AzureCameraTransform.TransformPosition(LocalUnrealCm);
        Data.Position = WorldCm;

        FQuat Qkinect(
            Src.orientation.wxyz.x,
            Src.orientation.wxyz.y,
            Src.orientation.wxyz.z,
            Src.orientation.wxyz.w
        );
        static const FMatrix RemapMatrix = FMatrix(
            FPlane(0, 0, 1, 0),
            FPlane(1, 0, 0, 0),
            FPlane(0, -1, 0, 0),
            FPlane(0, 0, 0, 1)
        );
        const FQuat RemapQuat(RemapMatrix);
        const FQuat RemapQuatInv = RemapQuat.Inverse();
        FQuat Qlocal = RemapQuat * Qkinect * RemapQuatInv;
        FQuat Qworld = AzureCameraTransform.GetRotation() * Qlocal;
        Data.Orientation = Qworld;

        OutJoints.Add(Data);
    }

    return true;
}
