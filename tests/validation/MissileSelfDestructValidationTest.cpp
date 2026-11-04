// The missile's self-destruct (REFERENCE.md 12.7): the time-of-flight
// backstop fires on the first tick at or past its limit and not before;
// the available-G floor fires only after motor burnout; a missile still
// guiding resolves the engagement as a miss, recorded as SELF_DESTRUCT
// then MISS; a missile already coasting after a miss is removed without
// touching the outcome; and a missile inside both limits, a disabled
// limit, or an unlaunched missile is left alone. Also a missile reaching
// the ground (REFERENCE.md 12.6), which resolves the same way.
//
// Each case drives MissileSelfDestructSystem::Update directly on a missile
// set to the state under test, so the check is the rule itself rather than
// a flight that happens to reach it.

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "TestRigs.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/MissileGLimitComponent.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileEntity.h"
#include "sim/FlightEnvelope.h"
#include "sim/SimContext.h"
#include "components/Transform.h"
#include "systems/GroundImpactSystem.h"
#include "systems/GuidanceSystem.h"
#include "systems/MissileSelfDestructSystem.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace {

// The AIM-120's own limits, read from its type so the test follows the data.
double MinG() { return testrig::Aim120()->selfDestruct.minAvailableG; }
double MaxTofS() { return testrig::Aim120()->selfDestruct.maxTimeOfFlightS; }
double BurnS() {
    const MissileType& t = *testrig::Aim120();
    return t.propulsion.boostDurationS + t.propulsion.sustainDurationS;
}

// Fast enough at 25 000 ft to sit at the structural cap; slow enough at
// 40 000 ft that the airframe can barely lift itself.
constexpr double kFastFps = 3000.0;
constexpr double kSlowFps = 250.0;
constexpr double kFastAltFt = 25000.0;
constexpr double kSlowAltFt = 40000.0;

// A target F-16 at the origin, level, nose along +Z, and an AIM-120 10 000 ft
// off its starboard side flying `speedFps` along +Z at `altitudeFt`
// (target moved to that altitude too), launched `timeOfFlightS` before
// t = 0. Guided unless `active` is false.
struct Rig {
    World world;
    SimContext ctx;
    ScenarioData scenario;
    EntityID target = 0;
    EntityID missile = 0;

    Rig(double speedFps, double altitudeFt, double timeOfFlightS, bool active = true) {
        target = testrig::SpawnMover(world, Vec3{0.0, altitudeFt, 0.0}, Vec3{});
        testrig::GiveF16Airframe(world, target);
        Vec3 pos{10000.0, altitudeFt, 0.0};
        missile = testrig::SpawnMover(world, pos, Vec3{0.0, 0.0, speedFps});
        GuidanceComponent g = testrig::GiveAim120(world, missile);
        g.target = target;
        g.launched = true;
        g.active = active;
        g.launchSimTime = -timeOfFlightS;
        g.launchPos = Vec3{0.0, altitudeFt, -50000.0};  // well past the arming range
        world.Components<GuidanceComponent>().Add(missile, g);
        world.Components<MissileGLimitComponent>().Add(missile, MissileEntity::MakeGLimit(*testrig::Aim120()));
    }

    void Update() { MissileSelfDestructSystem::Update(world, ctx, scenario, 0.0); }
    GuidanceComponent& Guidance() { return world.Components<GuidanceComponent>().Get(missile); }
    bool Destroyed() { return world.Components<DestroyedComponent>().Has(missile); }
    std::vector<SimEvent> Events() { return ctx.eventLog.Events(); }

    // Precondition, not the rule under test: what the envelope says this
    // missile can pull right now.
    double AvailableG(double speedFps, double altitudeFt, double timeOfFlightS) {
        return FlightEnvelope::GetMissileMaxAvailableG(
            speedFps, altitudeFt, GuidanceSystem::CurrentMassSlug(Guidance(), timeOfFlightS),
            world.Components<MissileGLimitComponent>().Get(missile));
    }
};

bool Contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

}  // namespace

TEST(MissileSelfDestruct, TimeOfFlightFiresAtTheLimitNotBefore) {
    {
        Rig rig(kFastFps, kFastAltFt, MaxTofS() - 0.01);
        ASSERT_GT(rig.AvailableG(kFastFps, kFastAltFt, MaxTofS()), MinG());  // the G floor stays out of it
        rig.Update();
        EXPECT_FALSE(rig.Destroyed());
        EXPECT_TRUE(rig.Events().empty());
    }
    {
        Rig rig(kFastFps, kFastAltFt, MaxTofS());
        rig.Update();
        EXPECT_TRUE(rig.Destroyed());
        ASSERT_FALSE(rig.Events().empty());
        EXPECT_EQ(rig.Events().front().type, "SELF_DESTRUCT");
        EXPECT_TRUE(Contains(rig.Events().front().data, "cause=max_tof"));
    }
}

TEST(MissileSelfDestruct, GFloorFiresOnlyAfterBurnout) {
    // Same slow, high missile, below the floor: still burning, it flies on.
    {
        Rig rig(kSlowFps, kSlowAltFt, BurnS() - 0.5);
        ASSERT_LT(rig.AvailableG(kSlowFps, kSlowAltFt, BurnS() - 0.5), MinG());
        rig.Update();
        EXPECT_FALSE(rig.Destroyed());
        EXPECT_TRUE(rig.Events().empty());
    }
    // Burnt out, it self-destructs on the floor.
    {
        Rig rig(kSlowFps, kSlowAltFt, BurnS());
        ASSERT_LT(rig.AvailableG(kSlowFps, kSlowAltFt, BurnS()), MinG());
        rig.Update();
        EXPECT_TRUE(rig.Destroyed());
        ASSERT_FALSE(rig.Events().empty());
        EXPECT_TRUE(Contains(rig.Events().front().data, "cause=min_g"));
    }
}

TEST(MissileSelfDestruct, GuidedMissileResolvesAsAMiss) {
    Rig rig(kSlowFps, kSlowAltFt, 30.0);
    rig.Update();

    EXPECT_TRUE(rig.Destroyed());
    EXPECT_FALSE(rig.Guidance().active);

    // SELF_DESTRUCT then MISS, the way every resolution is recorded.
    std::vector<SimEvent> events = rig.Events();
    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[0].type, "SELF_DESTRUCT");
    EXPECT_TRUE(Contains(events[0].data, "guided=yes"));
    EXPECT_EQ(events[1].type, "MISS");
    EXPECT_TRUE(Contains(events[1].data, "cause=self_destruct"));
    EXPECT_TRUE(Contains(events[1].data, "armed=yes"));

    const EngagementOutcome& outcome = rig.ctx.outcome;
    EXPECT_TRUE(outcome.resolved);
    EXPECT_FALSE(outcome.hit);
    EXPECT_TRUE(outcome.armed);
    EXPECT_DOUBLE_EQ(outcome.pk, 0.0);
    EXPECT_EQ(outcome.impactRegion, "self_destruct");
    EXPECT_DOUBLE_EQ(outcome.resolutionSimTime, 0.0);
    // Range to the airframe surface: 10 000 ft from the centre, less the
    // F-16's half-span (about 16 ft) off its starboard side.
    EXPECT_LT(outcome.missDistanceFt, 10000.0);
    EXPECT_GT(outcome.missDistanceFt, 9970.0);
}

// Past the time limit but mid-pass: the closest approach is InterceptSystem's
// to score, so the timer holds off. Outside the kill radius, it fires.
TEST(MissileSelfDestruct, HoldsOffDuringAPassInsideTheKillRadius) {
    constexpr double kKillRadiusFt = 12.0 * Units::kFeetPerMeter;
    // Inside the airframe.
    {
        Rig rig(kFastFps, kFastAltFt, MaxTofS());
        rig.scenario.killRadiusFt = kKillRadiusFt;
        rig.world.Components<Transform>().Get(rig.missile).pos = Vec3{0.0, kFastAltFt, 0.0};
        rig.Update();
        EXPECT_FALSE(rig.Destroyed());
        EXPECT_TRUE(rig.Events().empty());
    }
    // Already opening past the kill radius, with a closest approach inside
    // it still awaiting confirmation.
    {
        Rig rig(kFastFps, kFastAltFt, MaxTofS());
        rig.scenario.killRadiusFt = kKillRadiusFt;
        rig.Guidance().cpaOpeningTicks = 2;
        rig.Guidance().cpaCandidateRangeFt = 0.0;
        rig.Update();
        EXPECT_FALSE(rig.Destroyed());
    }
    // Candidate outside the kill radius: nothing for the fuze to do.
    {
        Rig rig(kFastFps, kFastAltFt, MaxTofS());
        rig.scenario.killRadiusFt = kKillRadiusFt;
        rig.Guidance().cpaOpeningTicks = 2;
        rig.Guidance().cpaCandidateRangeFt = 2.0 * kKillRadiusFt;
        rig.Update();
        EXPECT_TRUE(rig.Destroyed());
    }
    // Coasting after a miss: already resolved, so nothing to hold off for.
    {
        Rig rig(kFastFps, kFastAltFt, MaxTofS(), /*active=*/false);
        rig.scenario.killRadiusFt = kKillRadiusFt;
        rig.world.Components<Transform>().Get(rig.missile).pos = Vec3{0.0, kFastAltFt, 0.0};
        rig.Update();
        EXPECT_TRUE(rig.Destroyed());
    }
}

TEST(MissileSelfDestruct, CoastingMissileIsRemovedWithoutTouchingTheOutcome) {
    Rig rig(kSlowFps, kSlowAltFt, 30.0, /*active=*/false);
    rig.Update();

    EXPECT_TRUE(rig.Destroyed());
    std::vector<SimEvent> events = rig.Events();
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].type, "SELF_DESTRUCT");
    EXPECT_TRUE(Contains(events[0].data, "guided=no"));
    EXPECT_FALSE(rig.ctx.outcome.resolved);  // the earlier miss owned the outcome
}

TEST(MissileSelfDestruct, LeavesAloneMissilesItShouldNotTouch) {
    // Inside both limits.
    {
        Rig rig(kFastFps, kFastAltFt, 50.0);
        rig.Update();
        EXPECT_FALSE(rig.Destroyed());
        EXPECT_TRUE(rig.Events().empty());
    }
    // Both limits disabled (0): slow and long past the default time limit.
    {
        Rig rig(kSlowFps, kSlowAltFt, 2.0 * MaxTofS());
        rig.Guidance().selfDestructMinAvailableG = 0.0;
        rig.Guidance().selfDestructMaxTimeOfFlightS = 0.0;
        rig.Update();
        EXPECT_FALSE(rig.Destroyed());
    }
    // Not launched yet.
    {
        Rig rig(kSlowFps, kSlowAltFt, 2.0 * MaxTofS());
        rig.Guidance().launched = false;
        rig.Update();
        EXPECT_FALSE(rig.Destroyed());
    }
    // Already destroyed (a hit): no second event.
    {
        Rig rig(kSlowFps, kSlowAltFt, 2.0 * MaxTofS());
        rig.world.Components<DestroyedComponent>().Add(rig.missile, DestroyedComponent{});
        rig.Update();
        EXPECT_TRUE(rig.Events().empty());
    }
}

// Ground impact (REFERENCE.md 12.6), on the same rig: a missile still
// guiding when it reaches the ground resolves as a miss, recorded the way
// a self-destruct records one; a coasting one is only removed.
TEST(MissileGroundImpact, GuidedMissileResolvesAsAMissCoastingOneIsRemoved) {
    {
        Rig rig(kSlowFps, kSlowAltFt, 30.0);
        rig.world.Components<Transform>().Get(rig.missile).pos.y() = 0.0;
        GroundImpactSystem::Update(rig.world, rig.ctx, 0.0);
        EXPECT_TRUE(rig.Destroyed());
        EXPECT_FALSE(rig.Guidance().active);
        std::vector<SimEvent> events = rig.Events();
        ASSERT_EQ(events.size(), 1u);
        EXPECT_EQ(events[0].type, "MISS");
        EXPECT_TRUE(Contains(events[0].data, "cause=missile_ground_impact"));
        const EngagementOutcome& outcome = rig.ctx.outcome;
        EXPECT_TRUE(outcome.resolved);
        EXPECT_FALSE(outcome.hit);
        EXPECT_DOUBLE_EQ(outcome.pk, 0.0);
        EXPECT_EQ(outcome.impactRegion, "missile_ground_impact");
        EXPECT_GT(outcome.missDistanceFt, kSlowAltFt);  // the target is 40 000 ft up
    }
    {
        Rig rig(kSlowFps, kSlowAltFt, 30.0, /*active=*/false);
        rig.world.Components<Transform>().Get(rig.missile).pos.y() = 0.0;
        GroundImpactSystem::Update(rig.world, rig.ctx, 0.0);
        EXPECT_TRUE(rig.Destroyed());
        EXPECT_TRUE(rig.Events().empty());
        EXPECT_FALSE(rig.ctx.outcome.resolved);
    }
    // Above the ground: untouched.
    {
        Rig rig(kSlowFps, kSlowAltFt, 30.0);
        GroundImpactSystem::Update(rig.world, rig.ctx, 0.0);
        EXPECT_FALSE(rig.Destroyed());
        EXPECT_TRUE(rig.Events().empty());
    }
}
