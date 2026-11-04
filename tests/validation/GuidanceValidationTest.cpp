// Proportional navigation, isolated from everything else: a point-mass
// missile, launched with a deliberate boresight error, guiding on a
// stationary or constant-velocity target, checked on closest approach.

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "TestRigs.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "sim/SimContext.h"
#include "systems/GuidanceSystem.h"
#include "systems/IntegrationSystem.h"
#include "util/Vec3.h"

namespace {

// 20Hz -- plenty for a guidance-only check with no JSBSim rate to match.
constexpr double kDt = 0.05;
constexpr double kFeetPerMeter = 3.280839895;

double RangeFt(const Vec3& a, const Vec3& b) {
    double dx = a.x() - b.x(), dy = a.y() - b.y(), dz = a.z() - b.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Closest approach within one tick, taking the relative position as moving
// in a straight line from `from` to `to`. Sampling range only at the ticks
// misses it: at ~3 500 ft/s and 20 Hz the samples are ~50 m apart, so a
// direct hit can read as tens of metres depending on tick phase.
double SegmentMinRangeFt(const Vec3& from, const Vec3& to) {
    Vec3 step{to.x() - from.x(), to.y() - from.y(), to.z() - from.z()};
    double stepSq = step.x() * step.x() + step.y() * step.y() + step.z() * step.z();
    double s = stepSq > 0.0 ? -(from.x() * step.x() + from.y() * step.y() + from.z() * step.z()) / stepSq : 0.0;
    s = std::clamp(s, 0.0, 1.0);
    return RangeFt(Vec3{from.x() + s * step.x(), from.y() + s * step.y(), from.z() + s * step.z()}, Vec3{});
}

// Spawns a missile with a deliberate ~6 degree boresight error relative
// to target's initial position -- a shot aimed exactly at the target
// would "hit" even with zero guidance, which wouldn't actually exercise
// PN's corrective steering.
EntityID SpawnMissile(World& world, EntityID target, Vec3 missilePos, Vec3 missileVel, double navGain) {
    EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);

    const Transform& targetTransform = world.Components<Transform>().Get(target);
    Vec3 los{targetTransform.pos.x() - missilePos.x(), targetTransform.pos.y() - missilePos.y(),
              targetTransform.pos.z() - missilePos.z()};
    double losLen = std::sqrt(los.x() * los.x() + los.y() * los.y() + los.z() * los.z());
    Vec3 losUnit = losLen > 0.0 ? Vec3{los.x() / losLen, los.y() / losLen, los.z() / losLen} : Vec3{};

    GuidanceComponent guidance = testrig::GiveAim120(world, missile);
    guidance.target = target;
    guidance.navGain = navGain;
    guidance.prevLosUnit = losUnit;
    guidance.active = true;
    world.Components<GuidanceComponent>().Add(missile, guidance);

    return missile;
}

// Runs GuidanceSystem + IntegrationSystem for maxTicks, advancing the
// target's Transform directly by targetVel*dt each tick (a straight-line
// target needs nothing heavier than that -- no FlightDynamicsSystem
// involved). Returns the closest range achieved over the whole run,
// between ticks as well as at them.
double RunEngagement(World& world, EntityID missile, EntityID target, Vec3 targetVel, int maxTicks) {
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    SimContext simContext;

    auto relativePos = [&]() {
        const Vec3& m = transforms.Get(missile).pos;
        const Vec3& t = transforms.Get(target).pos;
        return Vec3{m.x() - t.x(), m.y() - t.y(), m.z() - t.z()};
    };

    double simTime = 0.0;  // missile "launches" at t=0 -- matches GuidanceComponent.launchSimTime's default
    double bestRange = RangeFt(transforms.Get(missile).pos, transforms.Get(target).pos);
    Vec3 prevRelative = relativePos();
    for (int i = 0; i < maxTicks; ++i) {
        simTime += kDt;
        GuidanceSystem::Update(world, simContext, kDt, simTime);
        IntegrationSystem::Update(world, kDt);

        Transform& targetTransform = transforms.Get(target);
        targetTransform.pos.x() += targetVel.x() * kDt;
        targetTransform.pos.y() += targetVel.y() * kDt;
        targetTransform.pos.z() += targetVel.z() * kDt;

        Vec3 relative = relativePos();
        bestRange = std::min(bestRange, SegmentMinRangeFt(prevRelative, relative));
        prevRelative = relative;
    }
    return bestRange;
}

const Vec3 kTargetPos{5000.0 * kFeetPerMeter, 0.0, 0.0};
const Vec3 kMissileVel{900.0, 0.0, 100.0};  // ~6.3 deg boresight error off the initial LOS

}  // namespace

TEST(GuidanceValidation, StationaryTarget5km) {
    World world;
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{kTargetPos, Vec3{}});
    EntityID missile = SpawnMissile(world, target, Vec3{}, kMissileVel, 8.0);

    double bestRangeM = RunEngagement(world, missile, target, Vec3{}, 500) / kFeetPerMeter;
    EXPECT_LT(bestRangeM, 15.0);
}

// Looser bound than the stationary check's 15m -- this is a qualitative
// lead-vs-trail check (if it trails, check the sign convention on
// los_rate), not a re-run of the scenario_basic.json deliverable.
TEST(GuidanceValidation, MovingTarget150ktsClosing) {
    constexpr double kFtPerSecPerKt = 1.68781;
    World world;
    Vec3 targetVel{-150.0 * kFtPerSecPerKt, 0.0, 0.0};
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{kTargetPos, Vec3{}});
    EntityID missile = SpawnMissile(world, target, Vec3{}, kMissileVel, 4.0);

    double bestRangeM = RunEngagement(world, missile, target, targetVel, 500) / kFeetPerMeter;
    EXPECT_LT(bestRangeM, 100.0) << "trailing the target -- check los_rate sign";
}
