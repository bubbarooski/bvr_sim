#pragma once

#include <cstddef>

#include "components/WaypointComponent.h"
#include "ecs/World.h"
#include "sim/Environment.h"

// General-purpose route following, and the simulator's entire external
// control surface. For every entity with both a WaypointComponent and an
// AutopilotComponent, steers toward queue.front by writing the
// autopilot's heading/altitude/speed targets, and advances the queue on
// arrival. In wind the heading is crabbed into it, so the aircraft makes
// good a straight track to the waypoint rather than drifting downwind of
// it. See REFERENCE.md - The Control Interface.
namespace WaypointSystem {

void Update(World& world, const Environment& environment);

// Route edits made from outside (the GUI). They change the queue's length
// at its back, so each also re-bases the route on the queue as shown;
// editing the queue directly leaves a rotated loop to be saved scrambled.
void AppendPoint(WaypointComponent& route, WaypointPoint point);
// Keeps the first `keep` queued points and drops the rest.
void TruncateQueue(WaypointComponent& route, size_t keep);

}  // namespace WaypointSystem
