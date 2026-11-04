#include "render/RenderSystem.h"

namespace RenderSystem {

// Green while the radar scans (with or without a tentative track), amber
// once a track is confirmed, red while it holds a single-target-track lock.
// A radar that holds nothing is still scanning, so it is green, and a
// dropped track goes back to green rather than to grey: grey (NONE) is for a
// sensor that is not working at all. One cone can cover several contacts, so
// it reports the most escalated thing inside it. track.state is live
// (AirborneRadarSystem drops tracks by its frame rule), so no staleness
// check is needed here.
RadarTrackMode RadarConeTier(const SimContext& simContext, EntityID owner, const AirborneRadarComponent& radar) {
    if (radar.sttTarget != kNoEntity) return RadarTrackMode::FIRING_SOLUTION;
    RadarTrackState highest = RadarTrackState::NONE;
    for (const AirborneRadarTrack& track : simContext.airborneRadarTracks) {
        if (track.owner != owner) continue;
        if (track.state > highest) highest = track.state;
    }
    return highest == RadarTrackState::CONFIRMED ? RadarTrackMode::TRACK : RadarTrackMode::SEARCH;
}

}  // namespace RenderSystem
