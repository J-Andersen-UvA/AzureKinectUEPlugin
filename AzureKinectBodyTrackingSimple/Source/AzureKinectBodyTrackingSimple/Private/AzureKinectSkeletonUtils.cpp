#include "AzureKinectSkeletonUtils.h"
#include "AzureKinectBodyTrackingComponent.h" // for FBodyJointData / EAzureKinectJoint

namespace AzureSkel
{
    static FORCEINLINE FVector MmToUEcmAndRemap(const k4a_float3_t& Pmm)
    {
        // Your existing mapping: UE.X = Kinect.Z, UE.Y = Kinect.X, UE.Z = Kinect.Y (in cm)
        const FVector LocalCm(Pmm.xyz.x * 0.1f, Pmm.xyz.y * 0.1f, Pmm.xyz.z * 0.1f);
        return FVector(LocalCm.Z, LocalCm.X, LocalCm.Y);
    }

    static FORCEINLINE FQuat RemapOrientation(const k4a_quaternion_t& Q)
    {
        const FQuat Qk(Q.wxyz.x, Q.wxyz.y, Q.wxyz.z, Q.wxyz.w);
        static const FMatrix RemapMatrix(
            FPlane(0, 0, 1, 0),
            FPlane(1, 0, 0, 0),
            FPlane(0, -1, 0, 0),
            FPlane(0, 0, 0, 1)
        );
        const FQuat R(RemapMatrix);
        return R * Qk * R.Inverse();
    }

    void FillJointArrayFromSkeleton(
        const k4abt_skeleton_t& Skeleton,
        const FTransform&       AzureCameraTransform,
        TArray<FBodyJointData>& OutJoints)
    {
        OutJoints.Reset();
        OutJoints.Reserve(K4ABT_JOINT_COUNT);

        const UEnum* UEEnum = StaticEnum<EAzureKinectJoint>();

        for (int JointIndex = 0; JointIndex < K4ABT_JOINT_COUNT; ++JointIndex)
        {
            const auto& Src = Skeleton.joints[JointIndex];

            FBodyJointData Data;
            Data.JointId = JointIndex;

            const EAzureKinectJoint JointEnum = static_cast<EAzureKinectJoint>(JointIndex);
            Data.JointName = UEEnum->GetDisplayNameTextByValue((int64)JointEnum).ToString();

            const FVector LocalUnrealCm = MmToUEcmAndRemap(Src.position);
            Data.Position = AzureCameraTransform.TransformPosition(LocalUnrealCm);

            const FQuat Qlocal = RemapOrientation(Src.orientation);
            Data.Orientation   = AzureCameraTransform.GetRotation() * Qlocal;

            OutJoints.Add(Data);
        }
    }
}
