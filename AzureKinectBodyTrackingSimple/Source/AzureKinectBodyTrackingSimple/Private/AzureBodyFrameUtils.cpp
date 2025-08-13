#include "AzureBodyFrameUtils.h"

namespace AzureFrame
{
    int32 FindClosestBodyId(k4abt_frame_t Frame)
    {
        if (!Frame) return -1;

        const uint32 NumBodies = k4abt_frame_get_num_bodies(Frame);
        float BestDistSq = TNumericLimits<float>::Max();
        int32 BestId = -1;

        for (uint32 i = 0; i < NumBodies; ++i)
        {
            const uint32 BodyId = k4abt_frame_get_body_id(Frame, i);
            if (BodyId == K4ABT_INVALID_BODY_ID) break;

            k4abt_skeleton_t Skel;
            if (k4abt_frame_get_body_skeleton(Frame, i, &Skel) != K4A_RESULT_SUCCEEDED) continue;

            const auto& P = Skel.joints[K4ABT_JOINT_PELVIS].position.xyz;
            const float DistSq = (P.x * 0.001f) * (P.x * 0.001f)
                               + (P.y * 0.001f) * (P.y * 0.001f)
                               + (P.z * 0.001f) * (P.z * 0.001f);

            if (DistSq < BestDistSq) { BestDistSq = DistSq; BestId = (int32)BodyId; }
        }
        return BestId;
    }
}
