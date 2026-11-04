// The launch, checked against what the documentation says it must do and
// against geometry that does not pass through the launch code: the missile
// leaves the launcher's position at the launcher's velocity, its closest-
// approach and line-of-sight bookkeeping starts from the real geometry (a
// zero would sit inside the target's airframe), the loft climb angle follows
// the documented straight-line schedule of launch range, a gate that shuts
// restarts the fire-control delay, and one engagement fires one missile.
//
// Everything runs through the real pipeline on scenario_basic. Expected values
// come from the type's JSON numbers, REFERENCE 6.10 / 7.3 / 12.8, and
// rotation-invariant geometry (a rotation preserves length, so a vector in the
// target's body frame must have the length of the range, whatever the
// frame's convention).

#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "aircraft/AircraftCatalog.h"
#include "components/AirborneRadarTrack.h"
#include "components/DatalinkTrackComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileCatalog.h"
#include "scenario/ScenarioLoader.h"
#include "scenario/ScenarioLoaderJson.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "systems/LaunchSystem.h"
#include "util/EntityLookup.h"
#include "util/EntityVelocity.h"
#include "util/Units.h"

namespace {

using EntityLookup::FindByTagName;

constexpr double kDt = SimClock::kFixedDt;

double Dist(const Vec3& a, const Vec3& b) { return (a - b).norm(); }

// scenario_basic stepped to the tick its missile fires. The state is read
// right after that tick, so the launcher and the missile are at most a tick or
// two of motion apart.
struct Launched {
    World world;
    SimContext ctx;
    ScenarioData scenario;
    EntityID pursuer = 0, target = 0, missile = 0;
    double launchAt = -1.0;
    double now = 0.0;

    // `edit` adjusts the scenario JSON before it is loaded.
    template <class Edit>
    explicit Launched(Edit edit) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        edit(json);
        scenario = ScenarioLoader::LoadFromJson(json, world);
        SimLoop::BeginRun(scenario, ctx, scenario.seed);
        LaunchSystem::InitFromTags(world, scenario);
        pursuer = *FindByTagName(world, "Pursuer");
        target = *FindByTagName(world, "Target");
        missile = world.Components<GuidanceComponent>().begin()->first;
        while (now < 300.0 && !ctx.outcome.launched) Step();
        if (ctx.outcome.launched) launchAt = now;
    }
    void Step() { SimLoop::Step(world, ctx, scenario, kDt, now); }

    const GuidanceComponent& Guidance() { return world.Components<GuidanceComponent>().Get(missile); }
    Vec3 Pos(EntityID e) { return world.Components<Transform>().Get(e).pos; }
};

// The type's loft numbers, read from the JSON rather than the C++ struct.
nlohmann::json Aim120Json() {
    std::ifstream in("data/missiles/aim120/missile.json");
    return nlohmann::json::parse(in);
}

}  // namespace

// ---------------------------------------------------------------------------
// The launch instant
// ---------------------------------------------------------------------------

// "At the launcher's current position and velocity": the missile starts where
// the launcher is, moving as it moves. One tick of motion separates the two
// readings at most (the launcher flies on, and the missile may already have
// been advanced once), so the tolerance is two ticks of the larger speed.
TEST(LaunchFirstPrinciples, MissileLeavesAtTheLaunchersPositionAndVelocity) {
    Launched shot([](nlohmann::json&) {});
    ASSERT_GE(shot.launchAt, 0.0) << "scenario_basic never fired";

    const Vec3 launcherVel = EntityVelocity::GetWithVelocityFallback(shot.world, shot.pursuer);
    const Vec3 missileVel = shot.world.Components<KinematicsComponent>().Get(shot.missile).vel;
    const double fastest = std::max(launcherVel.norm(), missileVel.norm());
    EXPECT_LT(Dist(shot.Pos(shot.missile), shot.Pos(shot.pursuer)), 2.0 * kDt * fastest);

    // Boost thrust over launch mass is the most the speed can have changed in
    // two ticks; gravity and drag are smaller. Thrust and mass from the type.
    const nlohmann::json type = Aim120Json();
    const double massSlug = type["mass"]["dry_mass_slug"].get<double>() + type["mass"]["propellant_mass_slug"].get<double>();
    const double boostAccel = type["propulsion"]["boost_thrust_lbf"].get<double>() / massSlug;
    EXPECT_LT((missileVel - launcherVel).norm(), 2.0 * kDt * (boostAccel + 32.174) + 1.0);

    const GuidanceComponent& g = shot.Guidance();
    EXPECT_TRUE(g.launched);
    EXPECT_TRUE(g.active);
    EXPECT_LE(std::fabs(g.launchSimTime - shot.launchAt), 2.0 * kDt);
    EXPECT_LT(Dist(g.launchPos, shot.Pos(shot.pursuer)), 2.0 * kDt * fastest);
}

// The closest-approach samples and the LOS memory are seeded from the launch
// geometry (REFERENCE 7.3: "so the first tick's delta is real and small").
// An unseeded zero is a point at the centre of the target's airframe box.
// Body-frame vectors must have the length of the range: a rotation preserves
// length, so this holds for any attitude convention.
TEST(LaunchFirstPrinciples, ClosestApproachAndLosBookkeepingStartFromTheRealGeometry) {
    Launched shot([](nlohmann::json&) {});
    ASSERT_GE(shot.launchAt, 0.0);
    const GuidanceComponent& g = shot.Guidance();

    const Vec3 missilePos = shot.Pos(shot.missile);
    const Vec3 targetPos = shot.Pos(shot.target);
    const double rangeFt = Dist(missilePos, targetPos);
    ASSERT_GT(rangeFt, 20.0 * Units::kFeetPerNmi);  // a long shot, so zero is far from right

    // Seed vs now differ by up to two ticks of closing motion (both aircraft and
    // the missile under boost); a generous 3 * fastest-speed * 2 ticks.
    const double slackFt = 3.0 * 2.0 * kDt * 2500.0;
    EXPECT_NEAR(g.prevRangeFt, rangeFt, slackFt);
    EXPECT_NEAR(g.prevPrevRangeFt, rangeFt, slackFt);
    EXPECT_NEAR(g.prevImpactBodyFt.norm(), rangeFt, slackFt);
    EXPECT_NEAR(g.prevPrevImpactBodyFt.norm(), rangeFt, slackFt);

    // The LOS memory is the true launch-instant line of sight. A first-tick LOS
    // rate under 0.01 rad/s needs a seed within 0.01 * dt rad of the true line;
    // the track's own error (~1 mrad at this range) would give 0.1 rad/s.
    const Vec3 los = (targetPos - missilePos).normalized();
    const double angleRad = std::acos(std::clamp(g.prevLosUnit.normalized().dot(los), -1.0, 1.0));
    EXPECT_LT(angleRad, 0.01 * kDt + 2.0 * kDt * 2500.0 / rangeFt);
}

// ---------------------------------------------------------------------------
// Loft schedule
// ---------------------------------------------------------------------------

// REFERENCE 6.10: zero at or inside start_range, gamma_max at or beyond
// full_range, a straight line between, in the launcher's range at firing. The
// numbers are the type's JSON; the line is the documented one. The tolerance
// is the climb change over the track's range error: the datalink estimate is
// within ~0.75 nmi of truth at this range, and the slope is
// gamma_max / (full - start) per nmi.
TEST(LaunchFirstPrinciples, LoftClimbAngleFollowsTheDocumentedLineOfLaunchRange) {
    const nlohmann::json type = Aim120Json();
    const double startNmi = type["loft"]["start_range_nmi"].get<double>();
    const double fullNmi = type["loft"]["full_range_nmi"].get<double>();
    const double maxClimbDeg = type["loft"]["max_climb_deg"].get<double>();
    const double tolDeg = maxClimbDeg / (fullNmi - startNmi) * 0.75;

    auto scheduled = [&](double rangeNmi) {
        if (rangeNmi <= startNmi) return 0.0;
        if (rangeNmi >= fullNmi) return maxClimbDeg;
        return maxClimbDeg * (rangeNmi - startNmi) / (fullNmi - startNmi);
    };

    bool sawLine = false;  // one shot must land strictly between the ends
    for (double fraction : {1.0, 0.5, 0.4, 0.15}) {
        SCOPED_TRACE(fraction);
        Launched shot([&](nlohmann::json& json) { json["missile_doctrine"]["launch_range_fraction"] = fraction; });
        ASSERT_GE(shot.launchAt, 0.0);
        const double rangeNmi = Dist(shot.Pos(shot.missile), shot.Pos(shot.target)) / Units::kFeetPerNmi;
        const GuidanceComponent& g = shot.Guidance();
        EXPECT_NEAR(g.loftClimbDeg, scheduled(rangeNmi), tolDeg) << "range " << rangeNmi << " nmi";
        if (rangeNmi > startNmi + 2.0 && rangeNmi < fullNmi - 2.0) sawLine = true;
        EXPECT_NEAR(g.loftClimbTimeConstantS, type["loft"]["climb_time_constant_s"].get<double>(), 1e-12);
        EXPECT_NEAR(g.loftMinAvailableG, type["loft"]["min_available_g"].get<double>(), 1e-12);
    }
    EXPECT_TRUE(sawLine) << "no fraction landed on the sloped part of the schedule";
}

// The knob off, or a seeker that measures from the rail, never lofts.
TEST(LaunchFirstPrinciples, NoLoftWhenTheKnobIsOffOrTheSeekerMeasuresFromTheRail) {
    Launched off([](nlohmann::json& json) { json["missile_doctrine"]["loft"] = false; });
    ASSERT_GE(off.launchAt, 0.0);
    EXPECT_EQ(off.Guidance().loftClimbDeg, 0.0);
    EXPECT_FALSE(off.Guidance().lofting);

    Launched sarh([](nlohmann::json& json) { json["missile_doctrine"]["seeker"]["mode"] = "SARH"; });
    ASSERT_GE(sarh.launchAt, 0.0);
    EXPECT_FALSE(sarh.Guidance().lofting);

    Launched on([](nlohmann::json&) {});
    ASSERT_GE(on.launchAt, 0.0);
    EXPECT_GT(on.Guidance().loftClimbDeg, 0.0);
    EXPECT_TRUE(on.Guidance().lofting);
}

// ---------------------------------------------------------------------------
// Gate, delay, count
// ---------------------------------------------------------------------------

// REFERENCE 12.8: the gate must stay open for the fire-control delay, and one
// that shuts meanwhile starts the delay again. The track is knocked to
// tentative for the radar frame it takes to re-confirm; the shot must then
// leave the delay after the gate REOPENS, not after it first opened.
TEST(LaunchFirstPrinciples, AGateThatShutsRestartsTheFireControlDelay) {
    nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    World world;
    SimContext ctx;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(json, world);
    SimLoop::BeginRun(scenario, ctx, scenario.seed);
    LaunchSystem::InitFromTags(world, scenario);
    const EntityID pursuer = *FindByTagName(world, "Pursuer");
    const EntityID target = *FindByTagName(world, "Target");
    const double delayS = AircraftCatalog::Default().Get("f16c_block50")->radar.fireControlDelayS;
    ASSERT_GT(delayS, 0.5);

    auto gateEvents = [&] {
        int n = 0;
        for (const SimEvent& e : ctx.eventLog.Events()) n += e.type == "LAUNCH_ZONE_ENTERED";
        return n;
    };

    double now = 0.0, firstGateAt = -1.0, reopenAt = -1.0;
    bool knocked = false;
    while (now < 300.0 && !ctx.outcome.launched) {
        SimLoop::Step(world, ctx, scenario, kDt, now);
        if (!knocked && gateEvents() > 0) {
            firstGateAt = now;
            ASSERT_NE(ctx.airborneRadarTracks.Find(pursuer, target), nullptr);
            ctx.airborneRadarTracks.FindOrCreate(pursuer, target).state = RadarTrackState::TENTATIVE;
            knocked = true;
        } else if (knocked && reopenAt < 0.0) {
            const AirborneRadarTrack* track = ctx.airborneRadarTracks.Find(pursuer, target);
            if (track != nullptr && track->state == RadarTrackState::CONFIRMED) reopenAt = now;
        }
    }
    ASSERT_TRUE(knocked);
    ASSERT_TRUE(ctx.outcome.launched);
    ASSERT_GE(reopenAt, 0.0);
    // The rig is only meaningful if the gap is well over the 2 ticks of timing
    // slack below.
    ASSERT_GT(reopenAt - firstGateAt, 6.0 * kDt) << "the track re-confirmed too fast to tell the two apart";

    double launchAt = 0.0;
    for (const SimEvent& e : ctx.eventLog.Events())
        if (e.type == "LAUNCH") launchAt = e.simTime;
    EXPECT_NEAR(launchAt - reopenAt, delayS, 3.0 * kDt);
    EXPECT_GT(launchAt - firstGateAt, delayS + 3.0 * kDt);
}

// One missile per engagement: flying on after the launch fires nothing more.
TEST(LaunchFirstPrinciples, OneEngagementFiresOneMissile) {
    Launched shot([](nlohmann::json&) {});
    ASSERT_GE(shot.launchAt, 0.0);
    const double until = shot.now + 10.0;
    while (shot.now < until && !shot.ctx.outcome.resolved) {
        shot.Step();
    }
    int launches = 0, launched = 0;
    for (const SimEvent& e : shot.ctx.eventLog.Events()) launches += e.type == "LAUNCH";
    for (const auto& entry : shot.world.Components<GuidanceComponent>()) launched += entry.second.launched ? 1 : 0;
    EXPECT_EQ(launches, 1);
    EXPECT_EQ(launched, 1);
}

// ---------------------------------------------------------------------------
// A destroyed launcher
// ---------------------------------------------------------------------------

// A launcher destroyed while its gate is open fires nothing: a wreck's radar
// stops returning looks, its track drops, and the gate shuts for good. Run far
// past the fire-control delay plus any track-drop time (30 s) to be sure.
TEST(LaunchFirstPrinciples, ALauncherDestroyedWithTheGateOpenNeverFires) {
    nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    World world;
    SimContext ctx;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(json, world);
    SimLoop::BeginRun(scenario, ctx, scenario.seed);
    LaunchSystem::InitFromTags(world, scenario);
    const EntityID pursuer = *FindByTagName(world, "Pursuer");
    auto gateOpened = [&] {
        for (const SimEvent& e : ctx.eventLog.Events())
            if (e.type == "LAUNCH_ZONE_ENTERED") return true;
        return false;
    };
    double now = 0.0;
    while (now < 300.0 && !gateOpened() && !ctx.outcome.launched) SimLoop::Step(world, ctx, scenario, kDt, now);
    ASSERT_TRUE(gateOpened());
    ASSERT_FALSE(ctx.outcome.launched) << "the rig must destroy the launcher before it fires";

    world.Components<DestroyedComponent>().Add(pursuer, DestroyedComponent{});
    const double until = now + 30.0;
    while (now < until) SimLoop::Step(world, ctx, scenario, kDt, now);
    EXPECT_FALSE(ctx.outcome.launched);
    for (const SimEvent& e : ctx.eventLog.Events()) EXPECT_NE(e.type, "LAUNCH");
}

// A launcher destroyed after the shot stops updating the datalink: its track
// drops, DATALINK_LOST is logged and the link is recorded lost, and from then
// on the missile's staleness only grows (it flies on the last estimate).
TEST(LaunchFirstPrinciples, ALauncherDestroyedAfterTheShotCutsTheDatalink) {
    Launched shot([](nlohmann::json&) {});
    ASSERT_GE(shot.launchAt, 0.0);
    // Let the link settle into normal operation first.
    const double settleUntil = shot.now + 3.0;
    while (shot.now < settleUntil) shot.Step();
    const DatalinkTrackComponent& link = shot.world.Components<DatalinkTrackComponent>().Get(shot.missile);
    ASSERT_TRUE(link.hasTrack);
    ASSERT_FALSE(link.linkLost);
    ASSERT_LT(link.stalenessS, 2.0 * link.updateIntervalS);

    shot.world.Components<DestroyedComponent>().Add(shot.pursuer, DestroyedComponent{});
    const double lostBy = shot.now + 30.0;
    while (shot.now < lostBy && !shot.ctx.outcome.resolved && !link.linkLost) shot.Step();
    ASSERT_FALSE(shot.ctx.outcome.resolved) << "the missile resolved before the link could drop";
    EXPECT_TRUE(link.linkLost);
    bool logged = false;
    for (const SimEvent& e : shot.ctx.eventLog.Events()) logged |= e.type == "DATALINK_LOST";
    EXPECT_TRUE(logged);

    // From here the estimate only ages: staleness never falls and no update lands.
    double previous = link.stalenessS;
    for (int i = 0; i < 240 && !shot.ctx.outcome.resolved; ++i) {
        shot.Step();
        EXPECT_GE(link.stalenessS, previous);
        previous = link.stalenessS;
    }
    EXPECT_TRUE(link.linkLost);
}
