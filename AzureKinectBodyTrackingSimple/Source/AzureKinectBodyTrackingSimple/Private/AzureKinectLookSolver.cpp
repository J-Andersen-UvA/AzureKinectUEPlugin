// AzureKinectLookSolver.cpp
#include "AzureKinectLookSolver.h"

namespace AzureLook
{
    FVector AzureToUE_SensorLocal_cm(const FVector& P_m)
    {
        // Matches your existing remap and meters->cm
        return FVector(-P_m.X * 100.f,  // UE +X
            -P_m.Y * 100.f,  // UE +Y
            P_m.Z * 100.f); // UE +Z
    }

    FVector ComputeLookTargetFromKinectHead(
        const FVector& HeadPosMeters_Kinect,
        const FTransform& KinectToWorld,
        const FTransform& CameraWorld,
        const FVector& AvatarHeadWorld,
        float            AimDistance)
    {
        // 1) Kinect -> UE sensor-local (cm)
        const FVector HeadLocalUE_cm = AzureToUE_SensorLocal_cm(HeadPosMeters_Kinect);

        // 2) Sensor-local -> World
        const FVector HeadWorld = KinectToWorld.TransformPosition(HeadLocalUE_cm);

        // 3) World -> Camera space; direction from camera to head
        const FVector HeadInCam = CameraWorld.InverseTransformPosition(HeadWorld);
        const FVector DirCam = HeadInCam.GetSafeNormal();

        // 4) Camera dir -> World dir; point on camera ray
        const FVector DirWorldFromCam = CameraWorld.TransformVectorNoScale(DirCam);
        const FVector WorldPointOnRay = CameraWorld.GetLocation() + DirWorldFromCam * 1000.f;

        // 5) Build final target along that world ray from the avatar head
        const FVector AimDirWorld = (WorldPointOnRay - AvatarHeadWorld).GetSafeNormal();
        return AvatarHeadWorld + AimDirWorld * AimDistance;
    }
}
