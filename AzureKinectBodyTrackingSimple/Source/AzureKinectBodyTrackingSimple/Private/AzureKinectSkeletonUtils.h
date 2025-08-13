// AzureKinectSkeletonUtils.h (Private)
#pragma once
#include "CoreMinimal.h"
#include <k4abt.h>

// Uses FBodyJointData and EAzureKinectJoint from your component header.
// We include it ONLY in .cpp files to avoid circular includes.
class UAzureKinectBodyTrackingComponent;
struct FBodyJointData;
enum class EAzureKinectJoint : uint8;

namespace AzureSkel
{
    /** Fills OutJoints from a k4abt_skeleton_t using your existing mm->cm and axis remap. */
    void FillJointArrayFromSkeleton(
        const k4abt_skeleton_t& Skeleton,
        const FTransform&       AzureCameraTransform,
        TArray<FBodyJointData>& OutJoints);
}
