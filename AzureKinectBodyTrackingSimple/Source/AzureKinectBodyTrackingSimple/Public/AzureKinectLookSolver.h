// AzureKinectLookSolver.h
#pragma once
#include "CoreMinimal.h"

// If other modules will use these, keep the API macro; if not, you can drop it.
namespace AzureLook
{
    // Convert Azure camera-space (meters) -> UE sensor-local (centimeters)
    AZUREKINECTBODYTRACKINGSIMPLE_API FVector AzureToUE_SensorLocal_cm(const FVector& P_m);

    // Compute a world-space look target that is invariant to where the virtual camera sits.
    AZUREKINECTBODYTRACKINGSIMPLE_API FVector ComputeLookTargetFromKinectHead(
        const FVector& HeadPosMeters_Kinect,  // Azure camera space (meters)
        const FTransform& KinectToWorld,      // real sensor transform in UE world
        const FTransform& CameraWorld,        // virtual camera transform in UE world
        const FVector& AvatarHeadWorld,     // avatar head/eyes world position
        float            AimDistance = 1000.f // cm
    );
}
