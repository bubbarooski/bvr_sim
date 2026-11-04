#include "MissileSelfDestructSystem.h"

#include <cstdio>
#include <string>

#include "aircraft/AircraftEntity.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "sim/AircraftBounds.h"
#include "sim/Environment.h"
#include "sim/FlightEnvelope.h"
#include "systems/GuidanceSystem.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace MissileSelfDestructSystem {

void Update(World& world, SimContext& simContext, const ScenarioData& scenario, double simTime) {
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<KinematicsComponent>& kinematics = world.Components<KinematicsComponent>();
    ComponentStore<MissileGLimitComponent>& missileGLimits = world.Components<MissileGLimitComponent>();

    for (auto& [missile, guidance] : world.Components<GuidanceComponent>()) {
        if (!guidance.launched || destroyed.Has(missile)) continue;
        const Transform* missileTransform = transforms.TryGet(missile);
        const KinematicsComponent* missileKinematics = kinematics.TryGet(missile);
        if (missileTransform == nullptr || missileKinematics == nullptr) continue;

        double timeOfFlightS = simTime - guidance.launchSimTime;
        double burnDurationS = guidance.boostDurationS + guidance.sustainDurationS;
        Vec3 airVel = missileKinematics->vel - WindFps(simContext.environment);
        double airSpeed = airVel.norm();

        // Available G only means something once the motor is out: during
        // the burn the missile is accelerating, and at launch it can have
        // less than the floor. A missile with no turn-ceiling model (a
        // synthetic test missile) has only the time limit.
        double availableG = -1.0;
        const MissileGLimitComponent* gLimit = missileGLimits.TryGet(missile);
        if (gLimit != nullptr) {
            availableG = FlightEnvelope::GetMissileMaxAvailableG(
                airSpeed, missileTransform->pos.y(), GuidanceSystem::CurrentMassSlug(guidance, timeOfFlightS), *gLimit,
                simContext.environment.temperatureOffsetC);
        }

        const char* cause = nullptr;
        if (guidance.selfDestructMaxTimeOfFlightS > 0.0 && timeOfFlightS >= guidance.selfDestructMaxTimeOfFlightS) {
            cause = "max_tof";
        } else if (guidance.selfDestructMinAvailableG > 0.0 && gLimit != nullptr && timeOfFlightS >= burnDurationS &&
                   availableG < guidance.selfDestructMinAvailableG) {
            cause = "min_g";
        }
        if (cause == nullptr) continue;

        // Range to the target's airframe surface, as InterceptSystem
        // measures it, so a self-destruct's miss distance means the same
        // thing as any other.
        double rangeFt = 0.0;
        Vec3 impactBodyFt{};
        AircraftBounds::HitBox targetBox{};
        const Transform* targetTransform = transforms.TryGet(guidance.target);
        if (targetTransform != nullptr) {
            targetBox = AircraftEntity::HitBoxOf(world, guidance.target);
            impactBodyFt =
                AircraftBounds::ToBodyFrameFt(missileTransform->pos, targetTransform->pos, targetTransform->euler);
            rangeFt = AircraftBounds::DistanceToSurfaceFromBodyFt(targetBox, impactBodyFt);
        }

        // Mid-pass, the fuze has already fired; InterceptSystem only waits a
        // few ticks to confirm the closest approach against numerical
        // noise. Holding off lets it score the pass. Bounded: the range
        // opens within a few ticks and the pass resolves.
        bool passInProgress = targetTransform != nullptr &&
                              (rangeFt <= scenario.killRadiusFt ||
                               (guidance.cpaOpeningTicks > 0 && guidance.cpaCandidateRangeFt <= scenario.killRadiusFt));
        if (guidance.active && passInProgress) continue;

        char data[192];
        std::snprintf(data, sizeof(data),
                      "cause=%s time_of_flight_s=%.2f air_speed_fps=%.1f available_g=%.2f range_m=%.1f guided=%s",
                      cause, timeOfFlightS, airSpeed, availableG, rangeFt / Units::kFeetPerMeter,
                      guidance.active ? "yes" : "no");
        simContext.eventLog.Record(SimEvent{simTime, "SELF_DESTRUCT", missile, data});

        // Still guiding: this ends the engagement, as a miss, recorded the
        // way every resolution is (a MISS event with its cause, as ground
        // impact records an INTERCEPT) so anything counting outcomes by
        // event counts it. Coasting after a miss: already resolved.
        if (guidance.active) {
            double distanceFromLaunchFt = (missileTransform->pos - guidance.launchPos).norm();
            bool armed = distanceFromLaunchFt >= guidance.armingRangeFt;
            char missData[160];
            std::snprintf(missData, sizeof(missData),
                          "cause=self_destruct miss_distance_m=%.2f pk=0.000000 armed=%s impact_region=self_destruct",
                          rangeFt / Units::kFeetPerMeter, armed ? "yes" : "no");
            simContext.eventLog.Record(SimEvent{simTime, "MISS", missile, missData});

            EngagementOutcome& outcome = simContext.outcome;
            outcome.resolved = true;
            outcome.hit = false;
            outcome.armed = armed;
            outcome.pk = 0.0;
            outcome.missDistanceFt = rangeFt;
            outcome.impactRegion = "self_destruct";
            outcome.impactBodyFt = impactBodyFt;
            outcome.resolutionSimTime = simTime;
            guidance.active = false;
        }
        destroyed.Add(missile, DestroyedComponent{});
    }
}

}  // namespace MissileSelfDestructSystem
