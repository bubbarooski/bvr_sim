#include "DatalinkSystem.h"

#include <string>

#include "components/AirborneRadarTrack.h"
#include "components/DatalinkTrackComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "ecs/ComponentStore.h"
#include "sim/RadarTrackFilter.h"
#include "util/Vec3.h"
#include "util/Numeric.h"

namespace DatalinkSystem {

void Update(World& world, SimContext& simContext, double dt, double simTime) {
    if (dt <= 0.0) return;

    ComponentStore<DatalinkTrackComponent>& datalinks = world.Components<DatalinkTrackComponent>();
    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();

    for (auto& [entity, datalink] : datalinks) {
        const GuidanceComponent* guidancePtr = guidances.TryGet(entity);
        if (guidancePtr == nullptr || destroyed.Has(entity)) continue;
        const GuidanceComponent& guidance = *guidancePtr;
        if (!guidance.launched || !guidance.active) continue;

        // Extrapolate the held estimate forward every tick. Between
        // updates this is all the missile has, and the estimate ages at
        // the rate of whatever the reported velocity got wrong.
        datalink.estimatedPos += datalink.estimatedVel * dt;
        datalink.stalenessS = datalink.hasTrack ? simTime - datalink.lastUpdateTime : 0.0;

        // Once the missile's own seeker is up it stops caring what the
        // launcher says, so there is nothing to maintain.
        if (guidance.seekerActive) continue;

        // The launcher can only relay a target it is still tracking.
        const AirborneRadarTrack* track =
            simContext.airborneRadarTracks.Find(guidance.launcher, guidance.target);
        bool linkUp = track != nullptr && track->state == RadarTrackState::CONFIRMED && track->estimate.valid;
        if (!linkUp) {
            // No update. The missile keeps flying the last estimate
            // forward, and its error grows with staleness.
            if (datalink.hasTrack && !datalink.linkLost) {
                datalink.linkLost = true;
                simContext.eventLog.Record(SimEvent{simTime, "DATALINK_LOST", entity,
                                                      "target=" + std::to_string(guidance.target)});
            }
            continue;
        }
        datalink.linkLost = false;

        if (datalink.hasTrack && !Numeric::PeriodElapsed(simTime, datalink.lastUpdateTime, datalink.updateIntervalS)) continue;

        // --- A datalink update ---
        // What the launcher sends is its radar's own track -- measured,
        // filtered and therefore wrong in the radar's way (REFERENCE.md -
        // Track measurement and filtering) -- carried forward from the
        // sweep it was formed on to now.
        Vec3 trackPos = RadarTrackFilter::Position(track->estimate);
        Vec3 trackVel = RadarTrackFilter::Velocity(track->estimate);
        datalink.estimatedPos = trackPos + trackVel * (simTime - track->estimate.time);
        datalink.estimatedVel = trackVel;
        datalink.hasTrack = true;
        datalink.lastUpdateTime = simTime;
        datalink.stalenessS = 0.0;
    }
}

}  // namespace DatalinkSystem
