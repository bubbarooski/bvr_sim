#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

#include "util/Vec3.h"

// One queued route point plus an optional speed override. Unset speedKts
// leaves the autopilot's target speed alone. Position and speed live in
// one struct rather than parallel deques, so there is no second array to
// keep in sync as points are pushed, popped, or reordered.
struct WaypointPoint {
    Vec3 pos;
    std::optional<double> speedKts;
};

// General-purpose route queue, attachable to any entity with an
// AutopilotComponent. The current waypoint is always queue.front; there
// is no separate index to keep in sync. WaypointSystem is the only system
// that reads or mutates this.
struct WaypointComponent {
    std::deque<WaypointPoint> queue;
    double arrivalRadiusM = 150.0;
    bool loop = false;
    // Index in the authored route where a looping route's repeat begins.
    // Points before it are a lead-in flown once; 0 loops the whole route.
    size_t loopFrom = 0;

    // The queue only holds what is still ahead, but a saved scenario must
    // describe the whole route as authored. These two record what the
    // queue has consumed so ScenarioSaver can reconstruct it.
    std::vector<WaypointPoint> passed;  // reached points dropped (a looping route's lead-in only), in order
    size_t loopRotation = 0;            // front-to-back rotations a looping route has made
};
