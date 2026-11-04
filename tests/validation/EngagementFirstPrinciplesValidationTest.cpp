// Engagement resolution checked against closed forms and geometry that do not
// pass through the model's own formulas: the logistic lethality curve, the
// analytic closest approach of straight-line motion, point-to-box distance,
// aerospace attitude conventions (nose up, right wing down, compass heading),
// a brute-force walk along a segment, flybys designed so the miss distance is
// known by construction, and the speed at which available G crosses the
// self-destruct floor.
//
// Local Vec3 helpers are plain doubles, not Eigen, so expected values do not
// share arithmetic with the code under test.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "TestRigs.h"
#include "ValidationAssertions.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileEntity.h"
#include "scenario/ScenarioLoader.h"
#include "sim/AircraftBounds.h"
#include "sim/SimContext.h"
#include "systems/GuidanceSystem.h"
#include "systems/IntegrationSystem.h"
#include "systems/InterceptSystem.h"
#include "systems/MissileSelfDestructSystem.h"
#include "util/Units.h"

namespace {

using validation::NearPct;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1.0 / 120.0;
constexpr double kStdGravity = 32.174;

Vec3 Add(const Vec3& a, const Vec3& b) { return Vec3{a.x() + b.x(), a.y() + b.y(), a.z() + b.z()}; }
Vec3 Sub(const Vec3& a, const Vec3& b) { return Vec3{a.x() - b.x(), a.y() - b.y(), a.z() - b.z()}; }
Vec3 Scale(const Vec3& a, double s) { return Vec3{a.x() * s, a.y() * s, a.z() * s}; }
double Dot(const Vec3& a, const Vec3& b) { return a.x() * b.x() + a.y() * b.y() + a.z() * b.z(); }
double Len(const Vec3& a) { return std::sqrt(Dot(a, a)); }
Vec3 Unit(const Vec3& a) { return Scale(a, 1.0 / Len(a)); }

// Deterministic uniform [0, 1) stream, independent of the sim's RNG.
struct Lcg {
    uint64_t state;
    double Next() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(state >> 11) * (1.0 / 9007199254740992.0);
    }
    double Range(double lo, double hi) { return lo + (hi - lo) * Next(); }
};

// The logistic lethality curve of REFERENCE 12.4, rescaled to 1 at zero miss.
double LogisticPk(double d, double r, double w) {
    return (1.0 + std::exp(-r / w)) / (1.0 + std::exp((d - r) / w));
}

// Body axes in world coordinates (X east, Y up, Z north) for roll/pitch/yaw in
// degrees, from what the angles mean: yaw is compass heading, positive pitch
// raises the nose, positive roll lowers the starboard wing.
struct BodyAxes {
    Vec3 right, up, nose;
};
BodyAxes AxesInWorld(const Vec3& eulerDeg) {
    const double phi = eulerDeg.x() * kPi / 180.0, theta = eulerDeg.y() * kPi / 180.0, psi = eulerDeg.z() * kPi / 180.0;
    const double cp = std::cos(phi), sp = std::sin(phi), ct = std::cos(theta), st = std::sin(theta);
    const double cy = std::cos(psi), sy = std::sin(psi);
    const Vec3 nose{ct * sy, st, ct * cy};
    const Vec3 right0{cy, 0.0, -sy};
    const Vec3 up0{-st * sy, ct, -st * cy};
    return {Sub(Scale(right0, cp), Scale(up0, sp)), Add(Scale(up0, cp), Scale(right0, sp)), nose};
}
Vec3 BodyToWorld(const BodyAxes& a, const Vec3& b) {
    return Add(Add(Scale(a.right, b.x()), Scale(a.up, b.y())), Scale(a.nose, b.z()));
}
Vec3 WorldToBody(const BodyAxes& a, const Vec3& d) { return Vec3{Dot(d, a.right), Dot(d, a.up), Dot(d, a.nose)}; }

// Distance from a body-frame point to a box centred on the origin.
double BoxDistance(const AircraftBounds::HitBox& h, const Vec3& b) {
    const double dx = std::max(std::fabs(b.x()) - h.halfSpanFt, 0.0);
    const double dy = std::max(std::fabs(b.y()) - h.halfHeightFt, 0.0);
    const double dz = std::max(std::fabs(b.z()) - h.halfLengthFt, 0.0);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool InsideBox(const AircraftBounds::HitBox& h, const Vec3& b, double slack = 0.0) {
    return std::fabs(b.x()) <= h.halfSpanFt + slack && std::fabs(b.y()) <= h.halfHeightFt + slack &&
           std::fabs(b.z()) <= h.halfLengthFt + slack;
}

std::shared_ptr<const MissileType> Type() { return testrig::Aim120(); }

}  // namespace

// ---------------------------------------------------------------------------
// Lethality curve
// ---------------------------------------------------------------------------

TEST(EngagementFirstPrinciples, PkIsTheRescaledLogisticAtManyPoints) {
    for (double r : {5.0, 15.0, 39.4}) {
        for (double w : {1.0, 5.0, 16.4}) {
            for (double d : {0.0, 0.3 * r, r, 1.7 * r, 5.0 * r}) {
                EXPECT_NEAR(InterceptSystem::Pk(d, r, w), LogisticPk(d, r, w), 1e-12) << "d=" << d << " r=" << r << " w=" << w;
            }
        }
    }
}

// Metres or feet give the same probability, as the header promises.
TEST(EngagementFirstPrinciples, PkIsUnitInvariant) {
    for (double k : {0.3048, 3.280839895, 1000.0}) {
        for (double d : {0.0, 4.0, 15.0, 22.0, 50.0}) {
            EXPECT_NEAR(InterceptSystem::Pk(k * d, k * 15.0, k * 5.0), InterceptSystem::Pk(d, 15.0, 5.0), 1e-12) << "k=" << k;
        }
    }
}

// The logistic is odd about the kill radius: L(r + x) + L(r - x) = 1, so with
// Pk = L / L(0) the two Pk values sum to 1 / L(0) = 1 + exp(-r / w).
TEST(EngagementFirstPrinciples, PkIsSymmetricAboutTheKillRadius) {
    const double r = 15.0, w = 5.0;
    for (double x : {0.5, 3.0, 9.0, 14.9}) {
        EXPECT_NEAR(InterceptSystem::Pk(r + x, r, w) + InterceptSystem::Pk(r - x, r, w), 1.0 + std::exp(-r / w), 1e-12);
    }
}

// A vanishing falloff width is a hard range cutoff at the kill radius.
TEST(EngagementFirstPrinciples, PkBecomesAHardCutoffAsTheFalloffWidthShrinks) {
    EXPECT_GT(InterceptSystem::Pk(0.99 * 15.0, 15.0, 1e-3), 1.0 - 1e-9);
    EXPECT_LT(InterceptSystem::Pk(1.01 * 15.0, 15.0, 1e-3), 1e-9);
    for (double d = 0.0; d < 80.0; d += 0.7) {
        double pk = InterceptSystem::Pk(d, 15.0, 5.0);
        EXPECT_GE(pk, 0.0);
        EXPECT_LE(pk, 1.0);
    }
}

// ---------------------------------------------------------------------------
// Sub-tick closest approach
// ---------------------------------------------------------------------------

// Two points in straight-line relative motion have r(t) = |r0 + u t|, closest
// approach at t* = -r0.u / u.u with miss |r0 + u t*|. Three 120 Hz range
// samples around the nearest tick must give that miss and that instant.
TEST(EngagementFirstPrinciples, SubTickRecoversTheAnalyticClosestApproachOfStraightLineMotion) {
    Lcg rng{12345};
    int checked = 0;
    for (int i = 0; i < 2000; ++i) {
        const double speed = rng.Range(100.0, 3500.0);
        Vec3 dir{rng.Range(-1, 1), rng.Range(-1, 1), rng.Range(-1, 1)};
        if (Len(dir) < 0.2) continue;
        const Vec3 u = Scale(Unit(dir), speed);
        const Vec3 r0{rng.Range(-4000, 4000), rng.Range(-4000, 4000), rng.Range(-4000, 4000)};
        const double tStar = -Dot(r0, u) / Dot(u, u);
        if (tStar < 5.0 * kDt) continue;
        const double missFt = Len(Add(r0, Scale(u, tStar)));

        const double kStar = tStar / kDt;
        const long k = std::lround(kStar);
        auto range = [&](long tick) { return Len(Add(r0, Scale(u, tick * kDt))); };
        double tOut = 0.0;
        const double got = InterceptSystem::SubTickMinRangeFt(range(k - 1), range(k), range(k + 1), tOut);
        EXPECT_NEAR(got, missFt, 1e-5) << "case " << i;
        EXPECT_NEAR(tOut, kStar - static_cast<double>(k), 1e-6) << "case " << i;

        // Time reversal mirrors the triple: same miss, instant on the other side.
        double tMirror = 0.0;
        const double mirrored = InterceptSystem::SubTickMinRangeFt(range(k + 1), range(k), range(k - 1), tMirror);
        EXPECT_NEAR(mirrored, got, 1e-9) << "case " << i;
        EXPECT_NEAR(tMirror, -tOut, 1e-9) << "case " << i;
        ++checked;
    }
    EXPECT_GT(checked, 500);
}

// A pass exactly through the target is the corner of the range curve, where the
// range itself has no derivative. Range squared is still an exact parabola.
TEST(EngagementFirstPrinciples, SubTickFindsAZeroMissExactly) {
    const Vec3 u{0.0, 0.0, -2800.0};
    for (double offsetTicks : {0.0, 0.17, 0.5, -0.31}) {
        const double tStar = 40.0 * kDt + offsetTicks * kDt;
        const Vec3 r0 = Scale(u, -tStar);
        auto range = [&](long tick) { return Len(Add(r0, Scale(u, tick * kDt))); };
        double tOut = 0.0;
        const double got = InterceptSystem::SubTickMinRangeFt(range(39), range(40), range(41), tOut);
        EXPECT_NEAR(got, 0.0, 1e-4) << "offset " << offsetTicks;
        EXPECT_NEAR(tOut, offsetTicks, 1e-6) << "offset " << offsetTicks;
    }
}

// ---------------------------------------------------------------------------
// Airframe geometry
// ---------------------------------------------------------------------------

// Beyond a face the distance is the gap; beyond an edge it is the Euclidean
// distance to that edge; beyond a corner, to that corner.
TEST(EngagementFirstPrinciples, BoxDistanceIsEuclideanToTheNearestFeature) {
    const AircraftBounds::HitBox box{10.0, 5.0, 20.0};
    const Vec3 centre{300.0, 20000.0, -50.0}, level{0.0, 0.0, 0.0};
    struct Case {
        Vec3 body;
        double expected;
    };
    const Case cases[] = {
        {{10.0 + 3.0, 0.0, 0.0}, 3.0},                                      // face
        {{10.0 + 3.0, 5.0 + 4.0, 0.0}, 5.0},                                // edge: 3-4-5
        {{10.0 + 3.0, 5.0 + 4.0, 20.0 + 12.0}, std::sqrt(9.0 + 16.0 + 144.0)},  // corner: 13
        {{-(10.0 + 6.0), 0.0, -(20.0 + 8.0)}, 10.0},                        // edge, negative side
        {{-(10.0 + 2.0), -(5.0 + 3.0), -(20.0 + 6.0)}, 7.0},                // corner, negative side (2-3-6)
        {{9.0, -4.9, 19.9}, 0.0},                                           // inside
        {{10.0, 5.0, 20.0}, 0.0},                                           // on the corner
    };
    for (const Case& c : cases) {
        EXPECT_NEAR(AircraftBounds::DistanceToSurfaceFt(box, Add(centre, c.body), centre, level), c.expected, 1e-9)
            << "body " << c.body.x() << "," << c.body.y() << "," << c.body.z();
        EXPECT_NEAR(AircraftBounds::DistanceToSurfaceFromBodyFt(box, c.body), c.expected, 1e-9);
    }
}

// The attitude angles mean what they mean in flight: yaw is compass heading,
// positive pitch raises the nose, positive roll lowers the starboard wing.
// Checked over a grid of attitudes against axes built from that meaning.
TEST(EngagementFirstPrinciples, AttitudeFollowsAerospaceConventions) {
    const AircraftBounds::HitBox box{15.0, 8.0, 25.0};
    const Vec3 centre{-700.0, 8000.0, 1200.0};

    // Plain readings of single angles.
    EXPECT_NEAR(AircraftBounds::WorldToBodyDirection(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 0.0, 90.0}).z(), 1.0, 1e-12);  // heading east: east is ahead
    EXPECT_NEAR(AircraftBounds::WorldToBodyDirection(Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 0.0}).z(), 1.0, 1e-12);   // heading north
    EXPECT_NEAR(AircraftBounds::WorldToBodyDirection(Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 30.0, 0.0}).z(), 0.5, 1e-12);  // nose 30 deg up: up is 0.5 ahead
    const Vec3 rolled = AircraftBounds::WorldToBodyDirection(Vec3{0.0, -1.0, 0.0}, Vec3{30.0, 0.0, 0.0});
    EXPECT_NEAR(rolled.x(), 0.5, 1e-12);  // right wing down: down is toward the starboard side

    Lcg rng{777};
    for (double roll : {-60.0, 0.0, 35.0}) {
        for (double pitch : {-40.0, 0.0, 25.0, 70.0}) {
            for (double yaw : {0.0, 90.0, 200.0, -130.0}) {
                const Vec3 euler{roll, pitch, yaw};
                const BodyAxes axes = AxesInWorld(euler);
                for (int i = 0; i < 6; ++i) {
                    const Vec3 world{rng.Range(-80, 80), rng.Range(-80, 80), rng.Range(-80, 80)};
                    const Vec3 expectedBody = WorldToBody(axes, world);
                    const Vec3 got = AircraftBounds::ToBodyFrameFt(Add(centre, world), centre, euler);
                    const Vec3 dir = AircraftBounds::WorldToBodyDirection(world, euler);
                    SCOPED_TRACE("attitude " + std::to_string(roll) + "," + std::to_string(pitch) + "," + std::to_string(yaw));
                    EXPECT_NEAR(got.x(), expectedBody.x(), 1e-9);
                    EXPECT_NEAR(got.y(), expectedBody.y(), 1e-9);
                    EXPECT_NEAR(got.z(), expectedBody.z(), 1e-9);
                    EXPECT_NEAR(dir.x(), expectedBody.x(), 1e-9);
                    EXPECT_NEAR(dir.y(), expectedBody.y(), 1e-9);
                    EXPECT_NEAR(dir.z(), expectedBody.z(), 1e-9);
                    EXPECT_NEAR(AircraftBounds::DistanceToSurfaceFt(box, Add(centre, world), centre, euler),
                                BoxDistance(box, expectedBody), 1e-9);
                    EXPECT_NEAR(Len(got), Len(world), 1e-9);  // a rotation keeps lengths
                }
            }
        }
    }
}

// Where the segment between two body-frame points first meets the box, in
// cases whose answer is a line-plane intersection done by hand.
TEST(EngagementFirstPrinciples, SegmentEntryMatchesHandIntersections) {
    const AircraftBounds::HitBox box{10.0, 5.0, 20.0};
    Vec3 entry{};

    // Oblique through the port face: x = -10 at fraction t = 10 / 40.
    ASSERT_TRUE(AircraftBounds::SegmentEntryFromBodyFt(box, Vec3{-20.0, 0.0, 0.0}, Vec3{20.0, 4.0, 6.0}, entry));
    EXPECT_NEAR(entry.x(), -10.0, 1e-9);
    EXPECT_NEAR(entry.y(), 1.0, 1e-9);
    EXPECT_NEAR(entry.z(), 1.5, 1e-9);

    // The reverse segment enters at the far face of the same chord.
    ASSERT_TRUE(AircraftBounds::SegmentEntryFromBodyFt(box, Vec3{20.0, 4.0, 6.0}, Vec3{-20.0, 0.0, 0.0}, entry));
    EXPECT_NEAR(entry.x(), 10.0, 1e-9);
    EXPECT_NEAR(entry.y(), 3.0, 1e-9);
    EXPECT_NEAR(entry.z(), 4.5, 1e-9);

    // Skimming past the x+ z+ vertical edge by 0.1 ft in the x+z direction misses; 0.1 ft inside hits.
    for (double c : {0.1, -0.1}) {
        const Vec3 from{box.halfSpanFt - 10.0, 0.0, box.halfLengthFt + 10.0 + c};
        const Vec3 to{box.halfSpanFt + 10.0, 0.0, box.halfLengthFt - 10.0 + c};
        EXPECT_EQ(AircraftBounds::SegmentEntryFromBodyFt(box, from, to, entry), c < 0.0) << "c=" << c;
    }

    // Stopping short; parallel to a face and outside it; parallel and inside.
    EXPECT_FALSE(AircraftBounds::SegmentEntryFromBodyFt(box, Vec3{0.0, 0.0, 70.0}, Vec3{0.0, 0.0, 30.0}, entry));
    EXPECT_FALSE(AircraftBounds::SegmentEntryFromBodyFt(box, Vec3{12.0, 0.0, -50.0}, Vec3{12.0, 0.0, 50.0}, entry));
    ASSERT_TRUE(AircraftBounds::SegmentEntryFromBodyFt(box, Vec3{9.0, 0.0, -50.0}, Vec3{9.0, 0.0, 50.0}, entry));
    EXPECT_NEAR(entry.z(), -20.0, 1e-9);
}

// Random segments against a walk along each, a hundredth of a foot at a time.
TEST(EngagementFirstPrinciples, SegmentEntryAgreesWithABruteForceWalk) {
    const AircraftBounds::HitBox box{10.0, 5.0, 20.0};
    Lcg rng{2024};
    constexpr int kSteps = 20000;
    int hits = 0, misses = 0;
    for (int i = 0; i < 3000; ++i) {
        const Vec3 from{rng.Range(-60, 60), rng.Range(-40, 40), rng.Range(-70, 70)};
        const Vec3 to{rng.Range(-60, 60), rng.Range(-40, 40), rng.Range(-70, 70)};
        const Vec3 span = Sub(to, from);
        const double stepFt = Len(span) / kSteps;

        int first = -1;
        for (int s = 0; s <= kSteps; ++s) {
            if (InsideBox(box, Add(from, Scale(span, static_cast<double>(s) / kSteps)))) {
                first = s;
                break;
            }
        }

        Vec3 entry{};
        const bool got = AircraftBounds::SegmentEntryFromBodyFt(box, from, to, entry);
        if (first >= 0) {
            ++hits;
            ASSERT_TRUE(got) << "segment " << i;
            const Vec3 walked = Add(from, Scale(span, static_cast<double>(first) / kSteps));
            EXPECT_LE(Len(Sub(entry, walked)), stepFt + 1e-9) << "segment " << i;
        } else {
            ++misses;
        }
        if (got) {
            // Whatever it returns lies on the box surface and on the segment.
            EXPECT_TRUE(InsideBox(box, entry, 1e-9)) << "segment " << i;
            const double s = Dot(Sub(entry, from), span) / Dot(span, span);
            EXPECT_GE(s, -1e-12);
            EXPECT_LE(s, 1.0 + 1e-12);
            EXPECT_LT(Len(Sub(entry, Add(from, Scale(span, s)))), 1e-9) << "segment " << i;
            // From outside, it enters on a face; from inside it "enters" where it starts.
            if (InsideBox(box, from)) {
                EXPECT_LT(Len(Sub(entry, from)), 1e-12) << "segment " << i;
            } else {
                const bool onSurface = std::fabs(std::fabs(entry.x()) - box.halfSpanFt) < 1e-9 ||
                                       std::fabs(std::fabs(entry.y()) - box.halfHeightFt) < 1e-9 ||
                                       std::fabs(std::fabs(entry.z()) - box.halfLengthFt) < 1e-9;
                EXPECT_TRUE(onSurface) << "segment " << i;
            }
        }
    }
    EXPECT_GT(hits, 150);
    EXPECT_GT(misses, 150);
}

// ---------------------------------------------------------------------------
// Whole flybys through InterceptSystem
// ---------------------------------------------------------------------------

namespace {

// A missile and a target moving in straight lines, stepped at 120 Hz and fed to
// InterceptSystem tick by tick. Built so that the relative path r(t) is known
// by construction: the missile is at `missBody` from the target (in the target's
// body frame, for a boxed target) at time tStar after the start, moving at
// `relVel` relative to it.
struct Flyby {
    World world;
    SimContext ctx;
    ScenarioData scenario;
    EntityID missile = 0, target = 0;
    Vec3 missileVel, targetVel;
    double t = 1.0;  // sim time, past the 0.5 s minimum flight time

    void Build(const Vec3& targetPos, const Vec3& targetEuler, bool airframe, const Vec3& missilePos,
               const Vec3& vMissile, const Vec3& vTarget, const Vec3& launchPos) {
        ctx.rng.Seed(7);
        scenario.killRadiusFt = 15.0;
        scenario.falloffWidthFt = 5.0;
        scenario.maxRangeFt = 1.0e7;
        missileVel = vMissile;
        targetVel = vTarget;
        target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, targetEuler});
        if (airframe) testrig::GiveF16Airframe(world, target);
        missile = world.CreateEntity();
        world.Components<Transform>().Add(missile, Transform{missilePos, Vec3{}});
        GuidanceComponent g = testrig::GiveAim120(world, missile);
        g.target = target;
        g.active = true;
        g.launched = true;
        g.launchSimTime = 0.0;
        g.prevRangeFt = g.prevPrevRangeFt = 1.0e9;  // the first tick closes
        g.launchPos = launchPos;
        // Launch seeds the previous samples; left at zero the "previous" point
        // would be the box centre, inside the airframe.
        g.prevImpactBodyFt = g.prevPrevImpactBodyFt = AircraftBounds::ToBodyFrameFt(missilePos, targetPos, targetEuler);
        world.Components<GuidanceComponent>().Add(missile, g);
    }

    GuidanceComponent& Guidance() { return world.Components<GuidanceComponent>().Get(missile); }
    int Count(const std::string& type) {
        int n = 0;
        for (const SimEvent& e : ctx.eventLog.Events()) n += e.type == type ? 1 : 0;
        return n;
    }
    bool Run(double maxSeconds) {
        const int ticks = static_cast<int>(maxSeconds / kDt);
        for (int i = 0; i < ticks && Guidance().active; ++i) {
            world.Components<Transform>().Get(missile).pos = Add(world.Components<Transform>().Get(missile).pos, Scale(missileVel, kDt));
            world.Components<Transform>().Get(target).pos = Add(world.Components<Transform>().Get(target).pos, Scale(targetVel, kDt));
            InterceptSystem::Update(world, ctx, scenario, t);
            t += kDt;
        }
        return !Guidance().active;
    }
};

// The part of `hint` perpendicular to `u`, scaled to length `d`.
Vec3 PerpendicularOfLength(const Vec3& u, const Vec3& hint, double d) {
    const Vec3 uHat = Unit(u);
    const Vec3 perp = Sub(hint, Scale(uHat, Dot(hint, uHat)));
    return Scale(Unit(perp), d);
}

constexpr double kTStar = 1.5037;  // seconds after the start; not a whole number of ticks

}  // namespace

// A straight pass at a point target: the reported miss is the miss the pass was
// built with, whatever the speeds, the geometry or where between two ticks it falls,
// and the lethality figure is the logistic at that miss.
TEST(EngagementFirstPrinciples, FlybyOfAPointTargetReportsTheDesignedMissDistance) {
    struct Case {
        const char* name;
        Vec3 vMissile, vTarget, hint;
        double missFt;
    };
    const Case cases[] = {
        {"head-on, stationary target", {0.0, 0.0, -1500.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 30.0},
        {"crossing, moving target", {-1900.0, -110.0, 1250.0}, {300.0, 0.0, -400.0}, {0.0, 1.0, 0.0}, 12.5},
        {"slow tail chase", {0.0, 0.0, 1400.0}, {0.0, 0.0, 1000.0}, {1.0, 1.0, 0.0}, 20.0},
        {"fast oblique", {2400.0, 300.0, 2100.0}, {-200.0, 0.0, 100.0}, {0.3, 1.0, -0.7}, 3.0},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.name);
        const Vec3 u = Sub(c.vMissile, c.vTarget);
        const Vec3 m = PerpendicularOfLength(u, c.hint, c.missFt);
        const Vec3 targetPos{500.0, 20000.0, -300.0};
        // At tStar the missile is m from the target; walk both back to the start.
        const Vec3 missilePos = Sub(Add(targetPos, m), Scale(u, kTStar));
        Flyby f;
        f.Build(targetPos, Vec3{}, /*airframe=*/false, missilePos, c.vMissile, c.vTarget, Sub(missilePos, Scale(Unit(c.vMissile), 5000.0)));
        ASSERT_TRUE(f.Run(8.0));
        ASSERT_TRUE(f.ctx.outcome.resolved);
        EXPECT_NEAR(f.ctx.outcome.missDistanceFt, c.missFt, 1e-3);
        EXPECT_NEAR(f.ctx.outcome.pk, LogisticPk(f.ctx.outcome.missDistanceFt, 15.0, 5.0), 1e-9);
        EXPECT_EQ(f.Count("INTERCEPT") + f.Count("MISS"), 1);  // resolved once
        // The resolution is never earlier than the closest approach itself.
        EXPECT_GE(f.ctx.outcome.resolutionSimTime, 1.0 + kTStar - 2.0 * kDt);
    }
}

// A pass exactly through the point is a certain kill: Pk 1, both destroyed, one event.
TEST(EngagementFirstPrinciples, ADirectPassIsACertainKillAndDestroysBoth) {
    const Vec3 vMissile{0.0, 0.0, -1500.0}, targetPos{0.0, 20000.0, 0.0};
    const Vec3 missilePos = Sub(targetPos, Scale(vMissile, kTStar));
    Flyby f;
    f.Build(targetPos, Vec3{}, false, missilePos, vMissile, Vec3{}, Sub(missilePos, Vec3{0.0, 0.0, -5000.0}));
    ASSERT_TRUE(f.Run(8.0));
    EXPECT_TRUE(f.ctx.outcome.hit);
    EXPECT_NEAR(f.ctx.outcome.missDistanceFt, 0.0, 1e-3);
    EXPECT_DOUBLE_EQ(f.ctx.outcome.pk, 1.0);
    EXPECT_TRUE(f.world.Components<DestroyedComponent>().Has(f.missile));
    EXPECT_TRUE(f.world.Components<DestroyedComponent>().Has(f.target));
    EXPECT_EQ(f.Count("INTERCEPT"), 1);
    EXPECT_EQ(f.Count("MISS"), 0);
}

// Once the warhead has had its pass, a missile that flies on and the later ticks
// produce no second resolution.
TEST(EngagementFirstPrinciples, AMissIsResolvedOnceAndNeverRescored) {
    const Vec3 vMissile{0.0, 0.0, -1500.0}, targetPos{0.0, 20000.0, 0.0};
    const Vec3 missilePos = Sub(Add(targetPos, Vec3{60.0, 0.0, 0.0}), Scale(vMissile, kTStar));
    Flyby f;
    f.Build(targetPos, Vec3{}, false, missilePos, vMissile, Vec3{}, Sub(missilePos, Vec3{0.0, 0.0, -5000.0}));
    ASSERT_TRUE(f.Run(8.0));
    ASSERT_EQ(f.Count("MISS"), 1);
    EXPECT_FALSE(f.ctx.outcome.hit);
    EXPECT_FALSE(f.world.Components<DestroyedComponent>().Has(f.target));
    // Put the missile through the target afterwards, as if it had turned round
    // and come back: the one shot is spent, so there is still just one event.
    EXPECT_FALSE(f.Guidance().active);
    f.world.Components<Transform>().Get(f.missile).pos = f.world.Components<Transform>().Get(f.target).pos;
    for (int i = 0; i < 600; ++i) InterceptSystem::Update(f.world, f.ctx, f.scenario, f.t += kDt);
    EXPECT_EQ(f.Count("MISS") + f.Count("INTERCEPT"), 1);
    EXPECT_FALSE(f.ctx.outcome.hit);
    EXPECT_FALSE(f.world.Components<DestroyedComponent>().Has(f.target));
}

// A pass at a boxed target past one corner, so the nearest feature is a single
// vertex and the range is an exact hyperbola there. In the target's body frame the
// missile is at corner + (A - s tau, b, A + s tau) at tau seconds from closest
// approach, so the miss is sqrt(2 A^2 + b^2) and the impact point
// is corner + (A, b, A). Flown at two attitudes.
TEST(EngagementFirstPrinciples, FlybyPastABoxCornerReportsTheDistanceToTheCornerAndWhereItWas) {
    const AircraftBounds::HitBox& box = testrig::F16()->hitBox;
    const double A = 30.0, b = 12.0, s = 1000.0;
    const double expectedMiss = std::sqrt(2.0 * A * A + b * b);
    for (const Vec3& euler : {Vec3{0.0, 0.0, 0.0}, Vec3{20.0, -10.0, 130.0}}) {
        SCOPED_TRACE("attitude " + std::to_string(euler.x()) + "," + std::to_string(euler.y()) + "," + std::to_string(euler.z()));
        const BodyAxes axes = AxesInWorld(euler);
        const Vec3 targetPos{1000.0, 15000.0, -2000.0};
        auto bodyPoint = [&](double tau) {
            return Vec3{box.halfSpanFt + A - s * tau, box.halfHeightFt + b, box.halfLengthFt + A + s * tau};
        };
        const Vec3 vMissile = BodyToWorld(axes, Vec3{-s, 0.0, s});
        const Vec3 missilePos = Add(targetPos, BodyToWorld(axes, bodyPoint(-kTStar)));
        Flyby f;
        f.Build(targetPos, euler, /*airframe=*/true, missilePos, vMissile, Vec3{}, Sub(missilePos, Scale(Unit(vMissile), 5000.0)));
        ASSERT_TRUE(f.Run(8.0));
        EXPECT_NEAR(f.ctx.outcome.missDistanceFt, expectedMiss, 1e-3);
        const Vec3 expectedBody = bodyPoint(0.0);
        EXPECT_NEAR(f.ctx.outcome.impactBodyFt.x(), expectedBody.x(), 0.05);
        EXPECT_NEAR(f.ctx.outcome.impactBodyFt.y(), expectedBody.y(), 0.05);
        EXPECT_NEAR(f.ctx.outcome.impactBodyFt.z(), expectedBody.z(), 0.05);
    }
}

// Flying straight through the airframe along its nose-to-tail line is a zero
// miss, a hit, and the strike point is where the path first entered: the nose face.
TEST(EngagementFirstPrinciples, PassThroughTheAirframeStrikesTheNoseFace) {
    const AircraftBounds::HitBox& box = testrig::F16()->hitBox;
    for (const Vec3& euler : {Vec3{0.0, 0.0, 0.0}, Vec3{-25.0, 15.0, 250.0}}) {
        SCOPED_TRACE("attitude " + std::to_string(euler.x()) + "," + std::to_string(euler.y()) + "," + std::to_string(euler.z()));
        const BodyAxes axes = AxesInWorld(euler);
        const Vec3 targetPos{-400.0, 12000.0, 900.0};
        const double v = 1500.0;
        auto bodyPoint = [&](double tau) { return Vec3{0.5, 1.0, -v * tau}; };
        const Vec3 vMissile = BodyToWorld(axes, Vec3{0.0, 0.0, -v});
        const Vec3 missilePos = Add(targetPos, BodyToWorld(axes, bodyPoint(-kTStar)));
        Flyby f;
        f.Build(targetPos, euler, true, missilePos, vMissile, Vec3{}, Sub(missilePos, Scale(Unit(vMissile), 5000.0)));
        ASSERT_TRUE(f.Run(8.0));
        EXPECT_TRUE(f.ctx.outcome.hit);
        EXPECT_NEAR(f.ctx.outcome.missDistanceFt, 0.0, 1e-3);
        for (const SimEvent& e : f.ctx.eventLog.Events()) SCOPED_TRACE(e.type + ": " + e.data);
        SCOPED_TRACE("region " + f.ctx.outcome.impactRegion);
        EXPECT_NEAR(f.ctx.outcome.impactBodyFt.z(), box.halfLengthFt, 0.01);
        EXPECT_NEAR(f.ctx.outcome.impactBodyFt.x(), 0.5, 0.01);
        EXPECT_NEAR(f.ctx.outcome.impactBodyFt.y(), 1.0, 0.01);
    }
}

// The fuze does not arm until the missile has flown its arming range from where
// it was launched: 50 m, in feet inside the sim. A perfect pass at 0.6 of it is inert;
// at 1.5 of it, a kill. A unit slip (metres read as feet) would swap these.
TEST(EngagementFirstPrinciples, FuzeArmsAtItsArmingRangeInFeet) {
    const double armingFt = Type()->fuze.armingRangeFt;
    EXPECT_NEAR(armingFt, 50.0 * Units::kFeetPerMeter, 0.1);

    const Vec3 vMissile{0.0, 0.0, -300.0}, targetPos{0.0, 20000.0, 0.0};
    const Vec3 m{1.0, 0.0, 0.0};
    const Vec3 missilePos = Sub(Add(targetPos, m), Scale(vMissile, kTStar));
    const Vec3 cpaPoint = Add(targetPos, m);
    for (double travelOverArming : {0.6, 1.5}) {
        const bool expectArmed = travelOverArming > 1.0;
        Flyby f;
        f.Build(targetPos, Vec3{}, false, missilePos, vMissile, Vec3{}, Sub(cpaPoint, Scale(Unit(vMissile), travelOverArming * armingFt)));
        f.scenario.killRadiusFt = 2000.0;  // Pk is 1 to machine precision at a 1 ft miss
        ASSERT_TRUE(f.Run(8.0));
        EXPECT_EQ(f.ctx.outcome.armed, expectArmed) << travelOverArming;
        EXPECT_EQ(f.ctx.outcome.hit, expectArmed) << travelOverArming;
        EXPECT_EQ(f.world.Components<DestroyedComponent>().Has(f.target), expectArmed) << travelOverArming;
        EXPECT_EQ(f.Count("INTERCEPT"), expectArmed ? 1 : 0) << travelOverArming;
        EXPECT_EQ(f.Count("MISS"), expectArmed ? 0 : 1) << travelOverArming;
    }
}

// ---------------------------------------------------------------------------
// Self-destruct
// ---------------------------------------------------------------------------

// Flown for real: a burnt-out missile coasting level at 25 000 ft slows under drag
// until the G its airframe can still pull, q CN_max S / (m g) with the standard
// atmosphere's density and speed of sound, falls to the floor; it must destroy
// itself at that speed, not before.
TEST(EngagementFirstPrinciples, GFloorFiresAtTheSpeedWhereAvailableGEqualsTheFloor) {
    const MissileType& type = *Type();
    const double floorG = type.selfDestruct.minAvailableG;
    EXPECT_DOUBLE_EQ(floorG, 2.0);  // REFERENCE 12.7

    const double altFt = 25000.0, rho = 0.0010663;
    const double sound = std::sqrt(1.4 * 1716.49 * (518.67 - 0.00356616 * altFt));
    const double mass = type.mass.dryMassSlug;
    auto availableG = [&](double v) {
        const double aero = 0.5 * rho * v * v * type.aero.normalForce->CnMax(v / sound) * type.aero.referenceAreaFt2 / (mass * kStdGravity);
        return std::min(aero, type.aero.structuralGLimit);
    };
    double lo = 100.0, hi = 1000.0;
    ASSERT_LT(availableG(lo), floorG);
    ASSERT_GT(availableG(hi), floorG);
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        (availableG(mid) < floorG ? lo : hi) = mid;
    }
    const double vStar = 0.5 * (lo + hi);

    World world;
    SimContext ctx;
    ScenarioData scenario;
    ctx.rng.Seed(1);
    const Vec3 pos{0.0, altFt, 0.0};
    EntityID target = testrig::SpawnMover(world, Vec3{2.0e6, altFt, 0.0}, Vec3{});
    testrig::GiveF16Airframe(world, target);
    EntityID missile = testrig::SpawnMover(world, pos, Vec3{800.0, 0.0, 0.0});
    GuidanceComponent g = testrig::GiveAim120(world, missile);
    g.target = target;
    g.active = true;
    g.launched = true;
    g.launchSimTime = -20.0;  // 8 s motor, long burnt out
    g.launchPos = pos;
    g.prevLosUnit = Vec3{1.0, 0.0, 0.0};
    g.selfDestructMaxTimeOfFlightS = 0.0;  // the time backstop is not what is under test
    world.Components<GuidanceComponent>().Add(missile, g);
    world.Components<MissileGLimitComponent>().Add(missile, MissileEntity::MakeGLimit(type));

    double t = 0.0, speedAtDestruct = 0.0;
    for (int i = 0; i < 200 * 120; ++i) {
        GuidanceSystem::Update(world, ctx, kDt, t);
        IntegrationSystem::Update(world, kDt);
        MissileSelfDestructSystem::Update(world, ctx, scenario, t);
        t += kDt;
        if (world.Components<DestroyedComponent>().Has(missile)) {
            speedAtDestruct = world.Components<KinematicsComponent>().Get(missile).speed;
            break;
        }
    }
    ASSERT_GT(speedAtDestruct, 0.0) << "never self-destructed in 200 s";
    EXPECT_TRUE(NearPct(speedAtDestruct, vStar, 1.0, "speed at self-destruct"));
    ASSERT_FALSE(ctx.eventLog.Events().empty());
    EXPECT_NE(ctx.eventLog.Events().front().data.find("cause=min_g"), std::string::npos);
}
