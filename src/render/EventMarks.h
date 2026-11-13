#pragma once

#include <optional>

#include "components/AirborneRadarTrack.h"
#include "ecs/World.h"
#include "sim/EventLog.h"

// Which engagement events leave a tick on a flight path, and on whose: the
// radar's own (track, lock, launch), the target's RWR, and the missile's
// seeker milestones. Detections, dropped tracks and G warnings are left
// out: too frequent to read as a timeline.
namespace EventMarks {

struct MarkSpec {
    EntityID path = kNoEntity;  // the entity whose trail carries the tick
    // A sensor tier takes that tier's colour (GUI_DESIGN.md "Sensor tier
    // color code"); nullopt is a milestone (launch, loft end, datalink lost).
    std::optional<RadarTrackMode> tier;
    const char* label = "";
};

// nullopt for an event that leaves no tick, or whose path entity is unknown.
std::optional<MarkSpec> Classify(World& world, const SimEvent& event);

}  // namespace EventMarks
