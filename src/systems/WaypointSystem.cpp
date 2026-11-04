#include "WaypointSystem.h"

#include <cmath>
#include <utility>

#include "components/AutopilotComponent.h"
#include "components/WaypointComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "sim/WindTriangle.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace {

using Units::kFeetPerMeter;  // arrival_radius_m is metric
using Units::kRadToDeg;

// atan2(east, north) is the compass bearing directly, normalized to
// [0, 360) since targetHeadingDeg is a compass heading, not a signed angle.
double BearingDeg(const Vec3& from, const Vec3& to) {
    double dx = to.x() - from.x();  // east
    double dz = to.z() - from.z();  // north
    double deg = std::atan2(dx, dz) * kRadToDeg;
    if (deg < 0.0) deg += 360.0;
    return deg;
}

}  // namespace

namespace WaypointSystem {

// Pure geometry: nothing to integrate, so no dt.
void Update(World& world, const Environment& environment) {
    ComponentStore<WaypointComponent>& waypoints = world.Components<WaypointComponent>();
    ComponentStore<AutopilotComponent>& autopilots = world.Components<AutopilotComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();

    for (auto& [entity, waypoint] : waypoints) {
        AutopilotComponent* autopilot = autopilots.TryGet(entity);
        const Transform* transform = transforms.TryGet(entity);
        if (autopilot == nullptr || transform == nullptr) continue;
        if (waypoint.queue.empty()) continue;

        const Vec3& pos = transform->pos;
        const Vec3& target = waypoint.queue.front().pos;

        // Arrival check first, so the command below is computed against
        // the new front rather than the just-reached waypoint.
        double arrivalRadiusFt = waypoint.arrivalRadiusM * kFeetPerMeter;
        if ((pos - target).norm() < arrivalRadiusFt) {
            WaypointPoint reached = waypoint.queue.front();
            waypoint.queue.pop_front();
            // A looping route's lead-in (before loopFrom) is dropped like a
            // non-looping route's points; only the repeating part rotates.
            if (waypoint.loop && waypoint.passed.size() >= waypoint.loopFrom) {
                waypoint.queue.push_back(reached);
                ++waypoint.loopRotation;
            } else {
                waypoint.passed.push_back(reached);
            }
            if (waypoint.queue.empty()) continue;
        }

        const WaypointPoint& current = waypoint.queue.front();
        autopilot->targetAltitudeFt = current.pos.y();
        // No override at this point leaves targetSpeedKts alone.
        if (current.speedKts.has_value()) autopilot->targetSpeedKts = *current.speedKts;
        // The bearing is the track to make good; the heading that makes it
        // good is crabbed into the crosswind, at the airspeed the autopilot
        // is holding (true airspeed, as everywhere here), through the wind.
        // Re-solved every tick from the current position, so any drift is
        // steered out.
        autopilot->targetHeadingDeg =
            WindTriangle::HeadingForTrackDeg(BearingDeg(pos, current.pos), autopilot->targetSpeedKts * Units::kFpsPerKt,
                                             WindFps(environment));
    }
}

// ScenarioSaver reconstructs the authored route by un-rotating the queue by
// loopRotation % queue.size(). That is only right while the queue keeps the
// size it had when it rotated, so an edit that changes it re-bases the
// route on the queue as drawn: the order the user sees is the one saved.
void AppendPoint(WaypointComponent& route, WaypointPoint point) {
    route.queue.push_back(std::move(point));
    route.loopRotation = 0;
}

void TruncateQueue(WaypointComponent& route, size_t keep) {
    route.queue.resize(keep);
    route.loopRotation = 0;
}

}  // namespace WaypointSystem
