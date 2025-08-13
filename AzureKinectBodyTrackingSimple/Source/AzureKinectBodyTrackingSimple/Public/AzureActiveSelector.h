// AzureActiveSelector.h
#pragma once
#include "CoreMinimal.h"

/**
 * Stateless inputs per body (only the Y values needed for "hand above head"),
 * measured in **millimeters** from Azure Kinect camera space (+Y is down).
 */
struct FAzureBodySample
{
    int32 BodyId = -1;
    float HeadY_mm = 0.f;
    float LHandY_mm = 0.f;
    float RHandY_mm = 0.f;
    float SeenAtSeconds = 0.f; // world time when this sample was observed
};

struct FActiveRaiseState
{
    bool  bLeftAbove = false;
    bool  bRightAbove = false;
    float LastSeen = 0.f;
    float LastRaise = 0.f; // time when a rising edge was detected
};

class FAzureActiveSelector
{
public:
    void Configure(int32 InAboveHeadMarginMM, float InRaiseHoldS, float InStickyS)
    {
        AboveHeadMarginMM = InAboveHeadMarginMM;
        RaiseHoldSeconds = InRaiseHoldS;
        StickySeconds = InStickyS;
    }

    void Reset()
    {
        States.Reset();
        ActiveId = -1;
    }

    /** Update using current frame bodies (Wave mode). Returns the suggested ActiveId (or -1). */
    int32 UpdateWaveLastRaised(const TArray<FAzureBodySample>& Bodies, float NowSeconds)
    {
        // Process all visible bodies
        for (const FAzureBodySample& B : Bodies)
        {
            if (B.BodyId < 0) continue;

            // Azure: +Y is down. Hand above head => HeadY - HandY > margin.
            const bool LeftAbove = (B.HeadY_mm - B.LHandY_mm) > AboveHeadMarginMM;
            const bool RightAbove = (B.HeadY_mm - B.RHandY_mm) > AboveHeadMarginMM;

            FActiveRaiseState& S = States.FindOrAdd(B.BodyId);
            const bool LeftRising = (!S.bLeftAbove && LeftAbove);
            const bool RightRising = (!S.bRightAbove && RightAbove);

            if (LeftRising || RightRising)
            {
                S.LastRaise = NowSeconds;
            }

            S.bLeftAbove = LeftAbove;
            S.bRightAbove = RightAbove;
            S.LastSeen = NowSeconds;

            const bool Held = (LeftAbove || RightAbove) && (NowSeconds - S.LastRaise) >= RaiseHoldSeconds;
            if (LeftRising || RightRising || Held)
            {
                ActiveId = B.BodyId; // last raise wins
            }
        }

        // Clear if the current active has gone stale
        if (ActiveId >= 0)
        {
            if (const FActiveRaiseState* SActive = States.Find(ActiveId))
            {
                if ((NowSeconds - SActive->LastSeen) > StickySeconds)
                {
                    ActiveId = -1;
                }
            }
            else
            {
                ActiveId = -1;
            }
        }

        // Optional: prune very stale bodies
        for (auto It = States.CreateIterator(); It; ++It)
        {
            if ((NowSeconds - It->Value.LastSeen) > 5.f)
            {
                It.RemoveCurrent();
            }
        }

        return ActiveId;
    }

    /** Current suggestion (-1 means no active). */
    int32 GetActiveId() const { return ActiveId; }

private:
    TMap<int32, FActiveRaiseState> States;
    int32  AboveHeadMarginMM = 120;
    float  RaiseHoldSeconds = 0.15f;
    float  StickySeconds = 2.0f;

    int32  ActiveId = -1;
};
