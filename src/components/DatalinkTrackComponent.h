#pragma once

#include "ecs/World.h"
#include "util/Numeric.h"
#include "util/Vec3.h"

// What a missile has been told about its target over the launch
// aircraft's datalink, and how wrong it is.
//
// During midcourse the missile is not measuring the target -- it is
// steering to a position someone else radioed to it. That position is the
// launcher's radar track (AirborneRadarTrack::estimate), and carries the
// launcher's measurement and filtering error, not truth: angle-dominated,
// so mostly across the line of sight and growing with range. See
// REFERENCE.md - Datalink target state.
struct DatalinkTrackComponent {
    // --- parameters ---
    // How often the launcher sends an update. Bounded below by its radar
    // sweep period in practice: it cannot report what it has not just
    // measured.
    double updateIntervalS = 1.0;

    // --- state ---
    bool hasTrack = false;         // has ever received an update
    Vec3 estimatedPos;             // ft, extrapolated to now
    Vec3 estimatedVel;             // ft/s, as last reported
    double lastUpdateTime = Numeric::kLongAgoS;
    double stalenessS = 0.0;       // since the last real update

    // Set true once the launcher's track is no longer confirmed, so the
    // updates stop. The missile keeps flying the last estimate forward:
    // this is what a midcourse break actually costs it.
    bool linkLost = false;
};
