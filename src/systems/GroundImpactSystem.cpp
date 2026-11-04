#include "GroundImpactSystem.h"

#include <cstdio>
#include <string>

#include "aircraft/AircraftEntity.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/JSBSimComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "sim/AircraftBounds.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace GroundImpactSystem {

void Update(World& world, SimContext& simContext, double simTime) {
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();

    for (auto& [aircraft, jsbsim] : world.Components<JSBSimComponent>()) {
        if (destroyed.Has(aircraft)) continue;
        // The ground is the plane at altitude zero; there is no terrain.
        double altitudeFt = jsbsim.props.hSlFt->getDoubleValue();
        if (altitudeFt > 0.0) continue;

        destroyed.Add(aircraft, DestroyedComponent{});
        char data[96];
        std::snprintf(data, sizeof(data), "speed_kts=%.1f descent_fps=%.1f", jsbsim.props.vtKts->getDoubleValue(),
                      jsbsim.props.vDownFps->getDoubleValue());
        simContext.eventLog.Record(SimEvent{simTime, "GROUND_IMPACT", aircraft, data});

        const Transform* aircraftTransform = transforms.TryGet(aircraft);
        for (auto& [missile, guidance] : world.Components<GuidanceComponent>()) {
            if (!guidance.active || guidance.target != aircraft) continue;
            const Transform* missileTransform = transforms.TryGet(missile);

            // Where the missile was when its target died, for the record.
            double rangeFt = 0.0;
            if (missileTransform != nullptr && aircraftTransform != nullptr) {
                rangeFt = (missileTransform->pos - aircraftTransform->pos).norm();
            }
            char interceptData[96];
            std::snprintf(interceptData, sizeof(interceptData), "cause=ground_impact missile_range_m=%.1f",
                          rangeFt / Units::kFeetPerMeter);
            simContext.eventLog.Record(SimEvent{simTime, "INTERCEPT", missile, interceptData});

            // Pk 1: the kill is certain, not a roll. Miss distance is the
            // missile's range at the moment, which is what an analyst
            // filtering on it would need to see that it never closed.
            EngagementOutcome& outcome = simContext.outcome;
            outcome.resolved = true;
            outcome.hit = true;
            outcome.armed = true;
            outcome.pk = 1.0;
            outcome.missDistanceFt = rangeFt;
            outcome.impactRegion = "ground_impact";
            outcome.impactBodyFt = Vec3{};
            outcome.resolutionSimTime = simTime;
            // The missile keeps coasting ballistically, as after any
            // resolution; it has nothing left to guide on.
            guidance.active = false;
        }
    }

    // A missile that reaches the ground is destroyed (IntegrationSystem
    // already froze it there). If it was still guiding, the engagement
    // resolves as a miss, recorded the way a self-destruct records one.
    // A missile that had already missed and was coasting is just removed.
    for (auto& [missile, guidance] : world.Components<GuidanceComponent>()) {
        if (!guidance.launched) continue;
        const Transform* missileTransform = transforms.TryGet(missile);
        if (missileTransform == nullptr || missileTransform->pos.y() > 0.0) continue;
        if (!destroyed.Has(missile)) destroyed.Add(missile, DestroyedComponent{});
        if (!guidance.active) continue;

        // Range to the target's airframe surface, as InterceptSystem
        // measures it.
        double rangeFt = 0.0;
        Vec3 impactBodyFt{};
        const Transform* targetTransform = transforms.TryGet(guidance.target);
        if (targetTransform != nullptr) {
            impactBodyFt =
                AircraftBounds::ToBodyFrameFt(missileTransform->pos, targetTransform->pos, targetTransform->euler);
            rangeFt =
                AircraftBounds::DistanceToSurfaceFromBodyFt(AircraftEntity::HitBoxOf(world, guidance.target), impactBodyFt);
        }
        double distanceFromLaunchFt = (missileTransform->pos - guidance.launchPos).norm();
        bool armed = distanceFromLaunchFt >= guidance.armingRangeFt;
        char missData[176];
        std::snprintf(missData, sizeof(missData),
                      "cause=missile_ground_impact miss_distance_m=%.2f pk=0.000000 armed=%s "
                      "impact_region=missile_ground_impact",
                      rangeFt / Units::kFeetPerMeter, armed ? "yes" : "no");
        simContext.eventLog.Record(SimEvent{simTime, "MISS", missile, missData});

        EngagementOutcome& outcome = simContext.outcome;
        outcome.resolved = true;
        outcome.hit = false;
        outcome.armed = armed;
        outcome.pk = 0.0;
        outcome.missDistanceFt = rangeFt;
        outcome.impactRegion = "missile_ground_impact";
        outcome.impactBodyFt = impactBodyFt;
        outcome.resolutionSimTime = simTime;
        guidance.active = false;
    }
}

}  // namespace GroundImpactSystem
