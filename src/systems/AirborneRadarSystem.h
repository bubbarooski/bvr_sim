#pragma once

#include "ecs/World.h"
#include "sim/SimContext.h"

// Nose-mounted airborne radar: FOV-gated detection by SNR, one look per
// target per frame, tracks confirmed and dropped by frame counts (tentative
// -> confirmed). While fire control holds a single-target-track lock
// (AirborneRadarComponent::sttTarget) only the locked target is looked at,
// once per STT frame. Tracks live in simContext.airborneRadarTracks;
// LaunchSystem gates missile launch on a confirmed (or locked) track.
namespace AirborneRadarSystem {

void Update(World& world, SimContext& simContext, double simTime);

}  // namespace AirborneRadarSystem
