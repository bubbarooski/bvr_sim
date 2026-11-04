#include "InterceptSystem.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "aircraft/AircraftEntity.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "sim/AircraftBounds.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace {

// Guards a false "closest approach passed" trigger from numerical wobble
// in the first ticks after launch.
constexpr double kMinFlightTimeSec = 0.5;

// Consecutive opening ticks before closest approach is believed. 25ms at
// 120Hz; a genuine pass opens at ~24ft/tick and never reverses.
constexpr int kCpaConfirmTicks = 3;

std::string MissDistanceData(double missDistanceFt) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "miss_distance_m=%.2f", missDistanceFt / Units::kFeetPerMeter);
    return buf;
}

// Impact point in the target's body frame (+X starboard, +Y up, +Z nose)
// plus a readable region name, so a hit reads as "port_wing" rather than
// just "0.00" -- and a near miss says which side it went past.
std::string ImpactData(const AircraftBounds::HitBox& box, const Vec3& impactBodyFt) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), " impact_region=%s impact_body_ft=(%.1f,%.1f,%.1f)",
                  AircraftBounds::ImpactRegionName(box, impactBodyFt).c_str(), impactBodyFt.x(), impactBodyFt.y(),
                  impactBodyFt.z());
    return buf;
}

// Linear blend between the two stored body-frame points that bracket the
// interpolated CPA, so impact_region/impact_body_ft describe the same
// instant as miss_distance_m rather than the nearest whole tick.
Vec3 LerpBodyPoint(const Vec3& atPrevPrev, const Vec3& atPrev, const Vec3& atCurrent, double t) {
    double f = t < 0.0 ? t + 1.0 : t;
    const Vec3& a = t < 0.0 ? atPrevPrev : atPrev;
    const Vec3& b = t < 0.0 ? atPrev : atCurrent;
    return a + (b - a) * f;
}

// The engagement's final outcome, shared by both ways it can end. Leaves
// `armed` and `pk` to the caller: only a closest-approach pass has them.
void RecordResolution(EngagementOutcome& outcome, bool hit, double missDistanceFt,
                      const AircraftBounds::HitBox& box, const Vec3& impactBodyFt, double simTime) {
    outcome.resolved = true;
    outcome.hit = hit;
    outcome.missDistanceFt = missDistanceFt;
    outcome.impactRegion = AircraftBounds::ImpactRegionName(box, impactBodyFt);
    outcome.impactBodyFt = impactBodyFt;
    outcome.resolutionSimTime = simTime;
}

}  // namespace

namespace InterceptSystem {

// Divided by its own value at zero so a pass through the airframe is a
// certain kill: the bare logistic gives only 1/(1 + e^(-r/w)) there (0.917
// at 12 m / 5 m), a property of the curve, not a sourced fuze reliability.
double Pk(double missDistanceFt, double killRadiusFt, double falloffWidthFt) {
    auto logistic = [&](double d) { return 1.0 / (1.0 + std::exp((d - killRadiusFt) / falloffWidthFt)); };
    return logistic(std::max(missDistanceFt, 0.0)) / logistic(0.0);
}

// Sub-tick closest approach: fits a parabola through three consecutive
// samples bracketing the minimum and returns its vertex. Fits range
// SQUARED, not range, because r(t)^2 is exactly a parabola at constant
// relative velocity. See REFERENCE.md - Sub-tick CPA recovery.
//
// tOut is the vertex position in ticks relative to the middle sample
// (negative = between the first two), clamped to the bracketing interval
// so a noisy triple cannot extrapolate outside its own samples.
double SubTickMinRangeFt(double rPrevPrev, double rPrev, double rCurrent, double& tOut) {
    double y0 = rPrevPrev * rPrevPrev;
    double y1 = rPrev * rPrev;
    double y2 = rCurrent * rCurrent;

    // Convex (> 0) means a real minimum between the outer samples.
    // Collinear or concave triples have no meaningful vertex.
    double secondDiff = y0 - 2.0 * y1 + y2;
    if (secondDiff <= 0.0) {
        tOut = 0.0;
        return rPrev;
    }

    tOut = std::clamp(0.5 * (y0 - y2) / secondDiff, -1.0, 1.0);
    double vertex = y1 - 0.125 * (y2 - y0) * (y2 - y0) / secondDiff;
    // Squared range can round slightly negative when the missile passes
    // essentially through the airframe. That is a hit, not a negative
    // distance.
    return vertex > 0.0 ? std::sqrt(vertex) : 0.0;
}

void Update(World& world, SimContext& simContext, const ScenarioData& scenario, double simTime) {
    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();

    // Resolving a missile sets active=false, so each one resolves exactly
    // once: this loop never revisits it.
    for (auto& [entity, guidance] : guidances) {
        if (!guidance.active) continue;
        const Transform* missileTransform = transforms.TryGet(entity);
        const Transform* targetTransformPtr = transforms.TryGet(guidance.target);
        if (missileTransform == nullptr || targetTransformPtr == nullptr) continue;
        const Vec3& missilePos = missileTransform->pos;

        // Distance to the nearest point on the target's airframe (its
        // type's oriented bounding box), not to its center point. The body-frame
        // point is computed once and drives both the impact point recorded
        // below and the surface distance -- they must describe the same
        // tick, and the transform is the same for both.
        const Transform& targetTransform = *targetTransformPtr;
        const AircraftBounds::HitBox targetBox = AircraftEntity::HitBoxOf(world, guidance.target);
        Vec3 impactBodyFt = AircraftBounds::ToBodyFrameFt(missilePos, targetTransform.pos, targetTransform.euler);
        double range = AircraftBounds::DistanceToSurfaceFromBodyFt(targetBox, impactBodyFt);
        double elapsed = simTime - guidance.launchSimTime;
        // The first time the path since last tick crosses the airframe from
        // outside it: where a pass-through hit struck.
        if (!guidance.impactEntryValid && guidance.prevRangeFt > 0.0) {
            guidance.impactEntryValid = AircraftBounds::SegmentEntryFromBodyFt(
                targetBox, guidance.prevImpactBodyFt, impactBodyFt, guidance.impactEntryBodyFt);
        }

        // Closest approach must be CONFIRMED over several ticks -- a bare
        // "range > prevRangeFt" is satisfied by one tick of numerical
        // noise on a momentarily flat profile.
        //
        // Measurement and decision happen at different instants: the
        // interpolated CPA can only be computed on the FIRST opening tick,
        // while the three samples still bracket the minimum, but acting on
        // it is only sound once confirmed. Capture then, decide later; any
        // tick that closes again discards the candidate.
        if (range > guidance.prevRangeFt) {
            if (guidance.cpaOpeningTicks == 0) {
                double cpaTicks = 0.0;
                guidance.cpaCandidateRangeFt =
                    SubTickMinRangeFt(guidance.prevPrevRangeFt, guidance.prevRangeFt, range, cpaTicks);
                guidance.cpaCandidateBodyFt = LerpBodyPoint(guidance.prevPrevImpactBodyFt, guidance.prevImpactBodyFt,
                                                             impactBodyFt, cpaTicks);
            }
            ++guidance.cpaOpeningTicks;
        } else {
            guidance.cpaOpeningTicks = 0;
            if (range < guidance.prevRangeFt) guidance.rangeHasClosed = true;
        }
        // A range that has only ever opened has had no closest approach
        // yet: the missile is still catching up (or never will, and
        // self-destructs). See REFERENCE.md - Closest point of approach.
        bool pastClosestApproach = elapsed > kMinFlightTimeSec && guidance.rangeHasClosed &&
                                   guidance.cpaOpeningTicks >= kCpaConfirmTicks;

        // Resolved once, at closest approach. Both outcomes end active
        // guidance: a hit has nothing left to guide, and a miss -- even a
        // near-miss that only failed the Pk roll -- means the warhead has
        // had its one shot. A real proximity fuze gets no second try.
        if (pastClosestApproach) {
            // Captured several ticks ago; the sample window has since slid
            // past the minimum.
            double missDistanceFt = guidance.cpaCandidateRangeFt;
            // A pass through the airframe struck where it went in, not
            // wherever along the zero-distance stretch the fit landed
            // (typically the exit, which would read a head-on hit as "tail").
            const Vec3& cpaBodyFt = missDistanceFt <= 0.0 && guidance.impactEntryValid
                                        ? guidance.impactEntryBodyFt
                                        : guidance.cpaCandidateBodyFt;

            // Hard gate: an unarmed pass is a miss regardless of Pk.
            // Straight-line from launch, not accumulated path length.
            double distanceFromLaunchFt = (missilePos - guidance.launchPos).norm();
            bool armed = distanceFromLaunchFt >= guidance.armingRangeFt;

            double pk = Pk(missDistanceFt, scenario.killRadiusFt, scenario.falloffWidthFt);
            bool hit = armed && simContext.rng.NextUniform(0.0, 1.0) < pk;

            std::string data = MissDistanceData(missDistanceFt) + " pk=" + std::to_string(pk) +
                                " armed=" + (armed ? "yes" : "no") + ImpactData(targetBox, cpaBodyFt);
            // Close enough that the warhead, not the guidance, decided it.
            if (armed && !hit && missDistanceFt <= scenario.killRadiusFt) data += " warhead=no_kill";

            RecordResolution(simContext.outcome, hit, missDistanceFt, targetBox, cpaBodyFt, simTime);
            simContext.outcome.armed = armed;
            simContext.outcome.pk = pk;
            guidance.active = false;

            if (hit) {
                world.Components<DestroyedComponent>().Add(guidance.target, DestroyedComponent{});
                // The missile is destroyed too. A miss deliberately does
                // not mark it destroyed, so it keeps coasting
                // ballistically instead of freezing.
                world.Components<DestroyedComponent>().Add(entity, DestroyedComponent{});
            }
            simContext.eventLog.Record(SimEvent{simTime, hit ? "INTERCEPT" : "MISS", entity, data});
            continue;
        }

        // Second, independent stop condition: range opens past max range
        // without ever registering a closest-approach pass, e.g. a target
        // already diverging at launch.
        if (range > scenario.maxRangeFt) {
            simContext.eventLog.Record(
                SimEvent{simTime, "MISS", entity, MissDistanceData(range) + ImpactData(targetBox, impactBodyFt)});
            RecordResolution(simContext.outcome, /*hit=*/false, range, targetBox, impactBodyFt, simTime);
            guidance.active = false;
            continue;
        }

        // Shift the three-sample window so a minimum detected next tick
        // still has both neighbours.
        guidance.prevPrevRangeFt = guidance.prevRangeFt;
        guidance.prevPrevImpactBodyFt = guidance.prevImpactBodyFt;
        guidance.prevRangeFt = range;
        guidance.prevImpactBodyFt = impactBodyFt;
    }
}

}  // namespace InterceptSystem
