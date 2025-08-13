#pragma once
#include "CoreMinimal.h"
#include <k4abt.h>

namespace AzureFrame
{
    /** Returns the body_id of the closest body (by pelvis distance), or -1 if none. */
    int32 FindClosestBodyId(k4abt_frame_t Frame);
}
