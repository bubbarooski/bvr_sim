#pragma once

#include <cstdint>
#include <utility>

#include "ecs/World.h"
#include "sim/RadarTrackFilter.h"
#include "util/Numeric.h"
#include "util/Vec3.h"

// The emitter-mode ladder the RWR reports and the sensor cones are coloured
// by (LatticeStyle::RadarTrackModeColor). The radar's own track state is
// RadarTrackState below.
enum class RadarTrackMode { NONE, SEARCH, TRACK, FIRING_SOLUTION };

// Where the radar's track on one target stands. TENTATIVE: detected, the
// filter is running, but nothing outside the radar may use it yet.
// CONFIRMED: confirmFrames detections in the window -- what the launch, the
// datalink and the missile's guidance need.
enum class RadarTrackState { NONE, TENTATIVE, CONFIRMED };

// One aircraft's airborne-radar track on one target, keyed by the (owner,
// target) pair and held in SimContext's TrackTable, one persistent entry per
// pair for the whole run. The state comes from the frame window below; it
// does not gate whether an entry exists.
struct AirborneRadarTrack {
    EntityID owner = 0;   // the aircraft carrying this radar
    EntityID target = 0;
    // Where the radar's tracker thinks the target is and how it moves,
    // from noisy measurements on each detection -- not truth. Kept through
    // missed frames (it coasts) and dropped with the track.
    TrackEstimate estimate;
    RadarTrackState state = RadarTrackState::NONE;

    // Rolling M-of-N window of frames, one bit per frame (LSB = most
    // recent): did this frame's look detect the target. Window length is
    // the radar's confirmWindowFrames (at most 15, so 16 bits hold it).
    uint16_t frameHits = 0;
    // Frames in a row without a detection, for the confirmed-track drop.
    int missedFrames = 0;

    // Sim time of the last detection. The default means never detected, as
    // opposed to a track that was up and has gone cold.
    double lastSeenTime = Numeric::kLongAgoS;
};

// The (owner, target) pair this track is keyed on, for TrackTable's index.
inline std::pair<EntityID, EntityID> TrackKey(const AirborneRadarTrack& t) { return {t.owner, t.target}; }
