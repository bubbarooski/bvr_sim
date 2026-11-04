// Straight-and-level flight holds, per airframe: one aircraft entity built
// directly (no scenario), flown under the autopilot, with heading and
// altitude hold checked against fixed tolerances. The F-16 cases also
// cover the G envelope, fuel burn and structural damage. Only a G-command
// FCS can be flown: ControlSystem commands load factor through it.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include "AircraftRig.h"
#include "components/DestroyedComponent.h"
#include "aircraft/AircraftEntity.h"
#include "components/GLimitComponent.h"
#include "components/JSBSimComponent.h"
#include "components/WaypointComponent.h"
#include "sim/Atmosphere.h"
#include "sim/Environment.h"
#include "sim/FlightEnvelope.h"
#include "sim/WindTriangle.h"
#include "components/GuidanceComponent.h"
#include "systems/EnvironmentSystem.h"
#include "systems/GroundImpactSystem.h"
#include "systems/WaypointSystem.h"
#include "util/Units.h"

namespace {

using testrig::AircraftRig;

// The trim-to-controller handoff has its own transient; measuring from t=0
// would conflate "how good is trim" with "how good is the sustained hold".
// The F-16 is slow but genuinely convergent here, hence 15 s rather than 5.
constexpr double kSettleS = 15.0;

double HeadingErrorDeg(double targetDeg, double currentDeg) {
    double error = std::fmod(targetDeg - currentDeg, 360.0);
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

struct StabilityResult {
    double maxAltErrorFt = 0.0;
    double maxHeadingErrorDeg = 0.0;
    double maxBankDeg = 0.0;
    double finalAltitudeFt = 0.0;
    int gLimitExceededCount = 0;
};

StabilityResult RunStraightAndLevel(const char* model, double altitudeFt, double airspeedKts, double headingDeg,
                                     double durationSec, bool attachGLimit = false) {
    AircraftRig rig(model, altitudeFt, airspeedKts, headingDeg);

    // A real GLimitComponent (F-16 check only) confirms straight-and-level
    // flight never trips G_LIMIT_EXCEEDED -- normal cruise is nowhere near
    // either envelope limit, so any trip here would mean the load-factor
    // property lookup or envelope math is wrong.
    if (attachGLimit) rig.world.Components<GLimitComponent>().Add(rig.id, AircraftEntity::MakeGLimit(*testrig::F16()));

    StabilityResult result;
    int ticks = static_cast<int>(durationSec / AircraftRig::kDt);
    int skipTicks = static_cast<int>(kSettleS / AircraftRig::kDt);
    for (int i = 0; i < ticks; ++i) {
        rig.Step();
        if (i < skipTicks) continue;
        const Transform& pose = rig.Pose();
        result.maxAltErrorFt = std::max(result.maxAltErrorFt, std::fabs(pose.pos.y() - altitudeFt));
        result.maxHeadingErrorDeg =
            std::max(result.maxHeadingErrorDeg, std::fabs(HeadingErrorDeg(headingDeg, pose.euler.z())));
        result.maxBankDeg = std::max(result.maxBankDeg, std::fabs(pose.euler.x()));
    }
    result.finalAltitudeFt = rig.Pose().pos.y();
    for (const SimEvent& event : rig.ctx.eventLog.Events()) {
        if (event.type == "G_LIMIT_EXCEEDED" && event.simTime >= kSettleS) ++result.gLimitExceededCount;
    }
    return result;
}

void CheckStraightAndLevel(const StabilityResult& result, double altToleranceFt, double headingToleranceDeg) {
    EXPECT_LT(result.maxAltErrorFt, altToleranceFt);
    EXPECT_LT(result.maxHeadingErrorDeg, headingToleranceDeg);
    EXPECT_EQ(result.gLimitExceededCount, 0);
}

}  // namespace

// F-16 at a realistic BVR cruise condition: +-50ft/+-2deg over 120s.
TEST(FlightStability, F16StraightAndLevel) {
    StabilityResult result = RunStraightAndLevel(testrig::kF16, 25000.0, 400.0, 90.0, 120.0, /*attachGLimit=*/true);
    CheckStraightAndLevel(result, 50.0, 2.0);
}

// Every entity spawns airborne, so the gear must be up from the first tick:
// JSBSim's default is gear-down, and the F-16's CDgear term would otherwise
// add ~60% to clean parasite drag. Sampled every tick from spawn, so a
// gear that merely retracts over its 5 s traverse still fails.
// The envelope's Mach factor on maximum lift is wired to the model: the
// property exists (a missing one would silently read as 1), follows the
// high-alpha factor calibrated to the flight manual's quickest turns
// subsonic (0.984 at Mach 0.7), and supersonic equals the measured F-16C
// lift-slope ratio (NASA TP-3355 over the model's 0.0645).
TEST(FlightStability, F16MaxLiftFollowsMach) {
    struct Point { double altitudeFt, kts, expected; };
    // ~M0.7 at 20 000 ft; ~M1.8 and ~M2.0 at 40 000 ft.
    const Point points[] = {{20000.0, 433.0, 0.984}, {40000.0, 1033.0, 0.899}, {40000.0, 1147.0, 0.806}};
    for (const Point& point : points) {
        AircraftRig rig(testrig::kF16, point.altitudeFt, point.kts, 90.0);
        const JSBSimPropertyNodes& props = rig.world.Components<JSBSimComponent>().Get(rig.id).props;
        ASSERT_NE(props.clMaxMachFactor, nullptr);
        EXPECT_NEAR(props.clMaxMachFactor->getDoubleValue(), point.expected, 0.01)
            << "at Mach " << rig.Prop("velocities/mach");
    }
}

TEST(FlightStability, F16GearRetractedInFlight) {
    AircraftRig rig(testrig::kF16, 25000.0, 400.0, 90.0);
    // A missing property would also read 0, so confirm both exist first.
    ASSERT_TRUE(rig.Fdm().GetPropertyManager()->HasNode("gear/gear-pos-norm"));
    ASSERT_TRUE(rig.Fdm().GetPropertyManager()->HasNode("aero/coefficient/CDgear"));
    double maxGearPos = 0.0;
    double maxGearDrag = 0.0;
    for (int i = 0; i < static_cast<int>(10.0 / AircraftRig::kDt); ++i) {
        rig.Step();
        maxGearPos = std::max(maxGearPos, rig.Prop("gear/gear-pos-norm"));
        maxGearDrag = std::max(maxGearDrag, std::fabs(rig.Prop("aero/coefficient/CDgear")));
    }
    EXPECT_EQ(maxGearPos, 0.0);
    EXPECT_EQ(maxGearDrag, 0.0);
}

// The F-16's trailing edge flaps droop only with the gear handle down (T.O.
// GR1F-16CJ-1-1, Flight Controls); the stock model drooped them 20 deg below
// 250 KCAS in any configuration, which put flap drag on a clean aircraft at
// 35 000 ft and 420 KTAS (235 KCAS) and made it burn 46% too much fuel there.
TEST(FlightStability, F16FlapsAreNeutralGearUpAtLowSpeed) {
    AircraftRig rig(testrig::kF16, 35000.0, 420.0, 90.0);
    rig.Fly(10.0);
    ASSERT_LT(rig.Prop("velocities/vc-kts"), 250.0) << "the stock schedule's flap region";
    EXPECT_EQ(rig.Prop("gear/gear-pos-norm"), 0.0);
    EXPECT_NEAR(rig.Prop("fcs/tef-pos-rad"), 0.0, 1e-9);
    EXPECT_NEAR(rig.Prop("fcs/tef-control"), 0.0, 1e-9);
    // The leading edge flap follows NASA TP-1538's schedule, not a step to 15 deg.
    EXPECT_LT(rig.Prop("fcs/lef-pos-rad"), 0.1);
}

// Fuel flow in steady cruise against the flight manual's Subsonic Cruise
// tables (T.O. GR1F-16CJ-1-1, drag index 0, 24 000 lb, standard day: total
// fuel flow in lb/hr). tools/fit/fuel_fit.py fits the TSFC to the manual's
// fuel-flow chart and the part-throttle factor to the cruise tables at other
// gross weights; these 24 000 lb points were held out of the fit, and
// real_world_validation flies the longer list at 5%. Tolerance 7%: the worst
// row is sea level at 240 KTAS, 6.6% low (VALIDATION.md).
TEST(FlightStability, F16CruiseFuelFlowMatchesTheManual) {
    struct Point { double altitudeFt, ktas, manualPph; };
    const Point points[] = {{200.0, 240.0, 2701.0}, {200.0, 480.0, 6039.0}, {8000.0, 420.0, 3819.0},
                            {25000.0, 330.0, 2205.0}, {35000.0, 420.0, 2082.0}};
    const FlightDynamicsSystem::FuelCapacity capacity = FlightDynamicsSystem::FuelCapacityOf(*testrig::F16());
    const double fuelFraction = (24000.0 - capacity.zeroFuelWeightLbf) / capacity.internalLbf;  // 24 000 lb gross
    for (const Point& p : points) {
        AircraftRig rig(testrig::kF16, p.altitudeFt, p.ktas, 90.0, {}, fuelFraction);
        rig.Fly(40.0);  // handoff, and JSBSim's slew of fuel flow
        double flowPph = 0.0;
        const int samples = static_cast<int>(20.0 / AircraftRig::kDt);
        for (int i = 0; i < samples; ++i) {
            rig.Step();
            flowPph += rig.Prop("propulsion/engine[0]/fuel-flow-rate-pps") * 3600.0 / samples;
        }
        EXPECT_NEAR(flowPph, p.manualPph, 0.07 * p.manualPph) << p.altitudeFt << " ft, " << p.ktas << " KTAS";
    }
}

// A non-standard day reaches JSBSim before trim, so the aircraft flies the
// same air Atmosphere reports for the missile and the envelope -- and a
// mid-run change (a live edit) reaches it through EnvironmentSystem.
TEST(FlightStability, F16FliesTheScenarioTemperature) {
    constexpr double kHotC = 30.0;
    AircraftRig hot(testrig::kF16, 25000.0, 400.0, 90.0, Environment{kHotC});
    AircraftRig standard(testrig::kF16, 25000.0, 400.0, 90.0);
    hot.Fly(2.0);
    standard.Fly(2.0);

    auto jsbsimDensity = [](AircraftRig& rig) { return rig.Prop("atmosphere/rho-slugs_ft3"); };
    auto ourDensity = [](AircraftRig& rig, double offsetC) {
        return Atmosphere::Lookup(rig.Prop("position/h-sl-ft"), offsetC).densitySlugPerFt3;
    };
    EXPECT_NEAR(jsbsimDensity(hot) / ourDensity(hot, kHotC), 1.0, 1e-9);
    EXPECT_NEAR(jsbsimDensity(standard) / ourDensity(standard, 0.0), 1.0, 1e-9);
    // Trim solved for different air, so it is a different trim.
    EXPECT_NE(hot.world.Components<JSBSimComponent>().Get(hot.id).trimThrottle,
              standard.world.Components<JSBSimComponent>().Get(standard.id).trimThrottle);

    // Cool the standard-day aircraft mid-flight.
    standard.ctx.environment.temperatureOffsetC = -20.0;
    EnvironmentSystem::Update(standard.world, standard.ctx);
    standard.Step();
    EXPECT_NEAR(jsbsimDensity(standard) / ourDensity(standard, -20.0), 1.0, 1e-9);
}

// The scenario wind reaches JSBSim at spawn: the F-16 starts at its
// commanded TRUE airspeed through the moving air, its ground velocity is
// that air velocity plus the wind, and it holds both while flying. Heading
// east in a 60 kt northerly, it drifts south at the wind speed. A live
// change of wind reaches it through EnvironmentSystem.
TEST(FlightStability, F16FliesThroughTheScenarioWind) {
    constexpr double kAirspeedKts = 400.0, kAltitudeFt = 25000.0;
    Environment northerly;
    northerly.windSpeedKts = 60.0;
    northerly.windFromDeg = 0.0;
    const Vec3 wind = WindFps(northerly);
    AircraftRig rig(testrig::kF16, kAltitudeFt, kAirspeedKts, 90.0, northerly);

    auto check = [&](const char* when, const Vec3& expectedWind) {
        SCOPED_TRACE(when);
        EXPECT_NEAR(rig.Prop("atmosphere/wind-north-fps"), expectedWind.z(), 1e-9);
        EXPECT_NEAR(rig.Prop("atmosphere/wind-east-fps"), expectedWind.x(), 1e-9);
        EXPECT_NEAR(rig.Prop("velocities/vtrue-kts"), kAirspeedKts, 2.0);
        // Air velocity is along the nose (east) to within the sideslip, so
        // the ground velocity's north component is the wind's.
        EXPECT_NEAR(rig.Prop("velocities/v-north-fps"), expectedWind.z(), 5.0);
        EXPECT_NEAR(rig.Prop("velocities/v-east-fps"), kAirspeedKts * Units::kFpsPerKt + expectedWind.x(), 5.0);
    };
    check("at spawn", wind);
    rig.Fly(30.0);
    check("after 30 s", wind);
    EXPECT_NEAR(rig.Pose().pos.y(), kAltitudeFt, 50.0);
    // 30 s of drift at 101 ft/s, from the transform the rest of the sim reads.
    EXPECT_NEAR(rig.Pose().pos.z(), 30.0 * wind.z(), 0.02 * 30.0 * std::fabs(wind.z()));

    // Swing it to a 60 kt headwind mid-flight.
    rig.ctx.environment.windFromDeg = 90.0;
    rig.Step();
    Vec3 headwind = WindFps(rig.ctx.environment);
    EXPECT_NEAR(rig.Prop("atmosphere/wind-north-fps"), headwind.z(), 1e-9);
    EXPECT_NEAR(rig.Prop("atmosphere/wind-east-fps"), headwind.x(), 1e-9);
    rig.Fly(60.0);
    check("after the change", headwind);
}

namespace {

struct WaypointRun {
    bool arrived = false;
    double maxCrossTrackFt = 0.0;
    double settledHeadingDeg = 0.0;  // halfway along the route
};

// An F-16 at 400 kt for a waypoint 20 nm due north, through an 80 kt
// easterly. `steeringEnvironment` is what WaypointSystem is told about the
// air -- the aircraft itself always flies the real wind. Both runs spawn
// already crabbed onto the track, so what is measured is the steering, not
// the turn-in from a spawn heading that ignores the wind.
WaypointRun FlyToWaypointInCrosswind(const Environment& steeringEnvironment) {
    constexpr double kAltitudeFt = 25000.0, kAirspeedKts = 400.0, kRouteFt = 121522.0;  // 20 nm
    Environment easterly;
    easterly.windSpeedKts = 80.0;
    easterly.windFromDeg = 90.0;
    double crabbedDeg = WindTriangle::HeadingForTrackDeg(0.0, kAirspeedKts * Units::kFpsPerKt,
                                                         WindFps(easterly));
    AircraftRig rig(testrig::kF16, kAltitudeFt, kAirspeedKts, crabbedDeg, easterly);
    WaypointComponent waypoint;
    waypoint.queue.push_back(WaypointPoint{Vec3{0.0, kAltitudeFt, kRouteFt}, std::nullopt});
    rig.world.Components<WaypointComponent>().Add(rig.id, waypoint);

    WaypointRun run;
    bool sampledHeading = false;
    for (int i = 0; i < static_cast<int>(400.0 / AircraftRig::kDt); ++i) {
        WaypointSystem::Update(rig.world, steeringEnvironment);
        if (rig.world.Components<WaypointComponent>().Get(rig.id).queue.empty()) {
            run.arrived = true;
            break;
        }
        rig.Step();
        run.maxCrossTrackFt = std::max(run.maxCrossTrackFt, std::fabs(rig.Pose().pos.x()));
        if (!sampledHeading && rig.Pose().pos.z() > 0.5 * kRouteFt) {
            run.settledHeadingDeg = rig.Pose().euler.z();
            sampledHeading = true;
        }
    }
    return run;
}

}  // namespace

// Waypoint steering crabs into a crosswind and holds the straight track to
// the waypoint. The same flight steered as if the air were calm still gets
// there -- re-aiming at the waypoint every tick -- but only by curving
// downwind of the track and back.
TEST(FlightStability, WaypointHoldsItsTrackInACrosswind) {
    Environment easterly;
    easterly.windSpeedKts = 80.0;
    easterly.windFromDeg = 90.0;
    WaypointRun corrected = FlyToWaypointInCrosswind(easterly);
    WaypointRun naive = FlyToWaypointInCrosswind(Environment{});
    std::printf("cross-track: corrected %.2f ft (heading %.2f deg), uncorrected %.0f ft (heading %.2f deg)\n",
                corrected.maxCrossTrackFt, corrected.settledHeadingDeg, naive.maxCrossTrackFt,
                naive.settledHeadingDeg);

    ASSERT_TRUE(corrected.arrived);
    // Crab angle asin(w / V): 80 kt across 400 kt true, into the wind (east).
    EXPECT_NEAR(corrected.settledHeadingDeg, std::asin(80.0 / 400.0) / Units::kDegToRad, 1.0);
    EXPECT_LT(corrected.maxCrossTrackFt, 100.0);
    EXPECT_GT(naive.maxCrossTrackFt, 4.0 * corrected.maxCrossTrackFt);
}

// An aircraft flown into the ground is destroyed there and stops, and a
// missile in flight against it is credited with the kill rather than left
// chasing a wreck.
TEST(FlightStability, GroundImpactDestroysAndCreditsTheShot) {
    AircraftRig rig(testrig::kF16, 3000.0, 400.0, 90.0);
    EntityID missile = rig.world.CreateEntity();
    GuidanceComponent guidance = testrig::GiveAim120(rig.world, missile);
    guidance.target = rig.id;
    guidance.active = true;
    guidance.launched = true;
    rig.world.Components<GuidanceComponent>().Add(missile, guidance);
    rig.world.Components<Transform>().Add(missile, Transform{Vec3{0.0, 3000.0, -30000.0}, Vec3{}});

    rig.Autopilot().targetAltitudeFt = -5000.0;  // straight into the ground
    double impactTime = -1.0;
    for (int i = 0; i < static_cast<int>(60.0 / AircraftRig::kDt) && impactTime < 0.0; ++i) {
        rig.Step();
        GroundImpactSystem::Update(rig.world, rig.ctx, rig.t);
        if (rig.Destroyed()) impactTime = rig.t;
    }
    ASSERT_GT(impactTime, 0.0) << "never reached the ground";
    EXPECT_LE(rig.Pose().pos.y(), 0.0);

    bool sawImpact = false, sawCredit = false;
    for (const SimEvent& event : rig.ctx.eventLog.Events()) {
        if (event.type == "GROUND_IMPACT" && event.entity == rig.id) sawImpact = true;
        if (event.type == "INTERCEPT" && event.entity == missile &&
            event.data.find("cause=ground_impact") != std::string::npos)
            sawCredit = true;
    }
    EXPECT_TRUE(sawImpact);
    EXPECT_TRUE(sawCredit);
    EXPECT_TRUE(rig.ctx.outcome.resolved);
    EXPECT_TRUE(rig.ctx.outcome.hit);
    EXPECT_EQ(rig.ctx.outcome.impactRegion, "ground_impact");
    EXPECT_FALSE(rig.world.Components<GuidanceComponent>().Get(missile).active);

    // Destroyed means frozen where it hit, and logged once.
    Vec3 wreck = rig.Pose().pos;
    rig.Fly(1.0);
    GroundImpactSystem::Update(rig.world, rig.ctx, rig.t);
    EXPECT_EQ(rig.Pose().pos.y(), wreck.y());
    int impacts = 0;
    for (const SimEvent& event : rig.ctx.eventLog.Events()) impacts += event.type == "GROUND_IMPACT" ? 1 : 0;
    EXPECT_EQ(impacts, 1);
}

// Live JSBSim fuel weight genuinely raises GetMaxAvailableG as fuel burns,
// while the same GLimitComponent's static weightLbf correctly stays flat --
// not just that ControlSystem reads the property, but that it changes the
// envelope math over real flight time.
TEST(FlightStability, FuelWeightGLimit) {
    AircraftRig rig(testrig::kF16, 25000.0, 400.0, 90.0);
    GLimitComponent gLimit = AircraftEntity::MakeGLimit(*testrig::F16());
    gLimit.weightLbf = 25000.0;  // this test's "static weightLbf" baseline
    rig.world.Components<GLimitComponent>().Add(rig.id, gLimit);

    constexpr double kRefSpeedFtPerSec = 675.0;  // fixed reference point for the comparison
    constexpr double kRefAltitudeFt = 25000.0;
    double initialWeightLbf = rig.Prop("inertia/weight-lbs");
    double maxGAtStart = FlightEnvelope::GetMaxAvailableG(kRefSpeedFtPerSec, kRefAltitudeFt, gLimit);

    rig.Fly(30.0);  // tens of pounds of fuel at cruise, well clear of the 1 lb threshold

    double finalWeightLbf = rig.Prop("inertia/weight-lbs");
    GLimitComponent liveGLimit = gLimit;
    liveGLimit.weightLbf = finalWeightLbf;
    double maxGLiveEnd = FlightEnvelope::GetMaxAvailableG(kRefSpeedFtPerSec, kRefAltitudeFt, liveGLimit);
    double maxGStaticEnd = FlightEnvelope::GetMaxAvailableG(kRefSpeedFtPerSec, kRefAltitudeFt, gLimit);

    EXPECT_LT(finalWeightLbf, initialWeightLbf - 1.0);
    EXPECT_GT(maxGLiveEnd, maxGAtStart + 1e-9);
    EXPECT_LT(std::fabs(maxGStaticEnd - maxGAtStart), 1e-9);
}

// G-overstress with a fast damage rate: cumulative damage rises
// monotonically, the structural limits stay where they are, and
// STRUCTURAL_FAILURE fires and destroys the entity at 1.0.
// The structural limit sits just below the 1 G the aircraft spawns at.
// ControlSystem clamps its load-factor command to the envelope, so it
// unloads to the limit within ~1.5 s; the overstress is that unload
// transient (~0.042 G-s), and the rate is set so it alone reaches failure.
TEST(FlightStability, StructuralDamage) {
    AircraftRig rig(testrig::kF16, 25000.0, 400.0, 90.0);

    GLimitComponent gLimit = AircraftEntity::MakeGLimit(*testrig::F16());
    gLimit.structuralGLimitPos = 0.9;     // just under the 1 G the aircraft spawns at
    gLimit.structuralGLimitNeg = -3.0;    // not under test here -- leave generous
    gLimit.damageAccumulationRate = 40.0;  // fast, so the unload transient alone reaches failure (~1.7)
    rig.world.Components<GLimitComponent>().Add(rig.id, gLimit);

    constexpr double kDurationS = 30.0;
    int ticks = static_cast<int>(kDurationS / AircraftRig::kDt);

    double prevDamage = 0.0;
    bool monotonic = true;
    bool limitHeld = true;
    bool destroyedMidLoop = false;

    for (int i = 0; i < ticks; ++i) {
        rig.Step();

        const GLimitComponent& live = rig.world.Components<GLimitComponent>().Get(rig.id);
        if (live.cumulativeDamage < prevDamage - 1e-12) monotonic = false;
        prevDamage = live.cumulativeDamage;
        if (live.structuralGLimitPos != gLimit.structuralGLimitPos) limitHeld = false;
        if (rig.Destroyed()) {
            destroyedMidLoop = true;
            break;
        }
    }

    bool sawFailureEvent = false;
    for (const SimEvent& event : rig.ctx.eventLog.Events()) {
        if (event.type == "STRUCTURAL_FAILURE") sawFailureEvent = true;
    }

    EXPECT_TRUE(monotonic);
    EXPECT_GT(prevDamage, 0.5);  // well into the damage, with the limit unchanged
    EXPECT_TRUE(limitHeld);
    EXPECT_TRUE(sawFailureEvent);
    EXPECT_TRUE(destroyedMidLoop) << "damage reached " << prevDamage;
}
