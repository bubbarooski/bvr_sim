// Missile flight checked against independent references: the boost/sustain
// /coast speed profile against a fine-step integration of the axial
// equation of motion, and PN engagements against closest-approach bounds.
// The local Norm/Sub/Dot/Unit/Scale Vec3 helpers below are kept as their
// own independent re-derivation rather than Eigen's operations, per
// REFERENCE.md 15.1's "must re-derive, not re-call".

#include <algorithm>
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "TestRigs.h"
#include "ValidationAssertions.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileEntity.h"
#include "sim/Atmosphere.h"
#include "sim/Environment.h"
#include "sim/FlightEnvelope.h"
#include "sim/SimContext.h"
#include "util/Units.h"
#include "systems/GuidanceSystem.h"
#include "systems/IntegrationSystem.h"

namespace {

using validation::NearPct;

constexpr double kDt = 1.0 / 120.0;
constexpr double kG = 32.174;

double Norm(const Vec3& v) { return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z()); }
Vec3 Sub(const Vec3& a, const Vec3& b) { return Vec3{a.x() - b.x(), a.y() - b.y(), a.z() - b.z()}; }
double Dot(const Vec3& a, const Vec3& b) { return a.x() * b.x() + a.y() * b.y() + a.z() * b.z(); }
Vec3 Unit(const Vec3& v) {
    double n = Norm(v);
    return n > 0.0 ? Vec3{v.x() / n, v.y() / n, v.z() / n} : Vec3{};
}
Vec3 Scale(const Vec3& v, double s) { return Vec3{v.x() * s, v.y() * s, v.z() * s}; }

struct MissileRig {
    World world;
    SimContext ctx;
    EntityID missile = 0;
    EntityID target = 0;
    double t = 0.0;

    void Step() {
        GuidanceSystem::Update(world, ctx, kDt, t);
        IntegrationSystem::Update(world, kDt);
        t += kDt;
    }
    KinematicsComponent& Kin() { return world.Components<KinematicsComponent>().Get(missile); }
    Transform& Tf() { return world.Components<Transform>().Get(missile); }
    Transform& TargetTf() { return world.Components<Transform>().Get(target); }
};

// Missile guiding on a target, both point masses. Target may carry a
// velocity (constant, via its own KinematicsComponent) so GuidanceSystem
// sees it move.
void SpawnMissile(MissileRig& r, const Vec3& pos, const Vec3& vel, const Vec3& targetPos, const Vec3& targetVel,
                  bool attachGLimit) {
    r.ctx.rng.Seed(1);

    r.target = testrig::SpawnMover(r.world, targetPos, targetVel);
    r.missile = testrig::SpawnMover(r.world, pos, vel);

    GuidanceComponent g = testrig::GiveAim120(r.world, r.missile);
    g.active = true;
    g.launched = true;
    g.launchSimTime = 0.0;
    g.target = r.target;
    g.launchPos = pos;
    g.prevLosUnit = Unit(Sub(targetPos, pos));  // so LOS rate reads ~0 on tick 1
    r.world.Components<GuidanceComponent>().Add(r.missile, g);

    if (attachGLimit) r.world.Components<MissileGLimitComponent>().Add(r.missile, MissileEntity::MakeGLimit(*testrig::Aim120()));
}

// Independent fine-step integration of the axial EOM dV/dt = (thrust(t) -
// drag(V))/m(t), matching GuidanceSystem's model exactly (same thrust
// schedule, mass depletion, drag law), used as ground truth
// for the sim's coarse-step trajectory. Returns speed and distance
// travelled at targetTime.
struct RefState {
    double speed;
    double distance;
};
RefState ReferenceProfile(double targetTime, double v0, double altFt, const GuidanceComponent& g) {
    const double fine = kDt / 100.0;
    const double burn = g.boostDurationS + g.sustainDurationS;
    Atmosphere::AtmosphereState atm = Atmosphere::Lookup(altFt);
    double rho = atm.densitySlugPerFt3;
    double sos = atm.speedOfSoundFtPerSec;

    auto accel = [&](double t, double v) {
        double mass = g.dryMassSlug + g.propellantMassSlug * (1.0 - std::clamp(t / burn, 0.0, 1.0));
        double thrust = 0.0;
        if (t < g.boostDurationS) thrust = g.boostThrustLbf;
        else if (t < burn) thrust = g.sustainThrustLbf;
        if (!(v > 0.001)) thrust = 0.0;  // matches the sim's thrust gate
        double drag = 0.5 * rho * v * v * testrig::Aim120()->aero.ZeroLiftCd(v / sos) * g.referenceAreaFt2;
        return (thrust - drag) / mass;
    };

    double v = v0, x = 0.0;
    for (double t = 0.0; t < targetTime - 1e-9; t += fine) {
        double k1 = accel(t, v);
        double k2 = accel(t + fine / 2, v + fine / 2 * k1);
        double k3 = accel(t + fine / 2, v + fine / 2 * k2);
        double k4 = accel(t + fine, v + fine * k3);
        double dv = fine / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        x += (v + dv / 2) * fine;
        v += dv;
    }
    return {v, x};
}

}  // namespace

// Boost/sustain/coast speed and range vs an independent integration of
// the same EOM.
TEST(MissileManeuvers, PoweredFlight) {
    const double altFt = 20000.0;
    const double v0 = 900.0;  // launch speed (carrier aircraft)
    MissileRig r;
    // Target far ahead, collinear and stationary, so PN commands ~zero
    // lateral accel and the flight is pure axial thrust vs drag.
    SpawnMissile(r, Vec3{0.0, altFt, 0.0}, Vec3{v0, 0.0, 0.0}, Vec3{400000.0, altFt, 0.0}, Vec3{}, /*glimit=*/false);

    const GuidanceComponent params = testrig::SyntheticAim120Guidance();  // the reference's physical constants
    struct Sample {
        double t;
        const char* phase;
    };
    const Sample samples[] = {{2.99, "boost end"}, {7.99, "sustain end"}, {12.0, "coast"}};
    size_t next = 0;
    int ticks = static_cast<int>(12.5 / kDt);
    for (int i = 0; i < ticks && next < 3; ++i) {
        r.Step();
        if (r.t >= samples[next].t) {
            RefState ref = ReferenceProfile(r.t, v0, altFt, params);
            std::string label = std::string("speed @ ") + samples[next].phase;
            EXPECT_TRUE(NearPct(r.Kin().speed, ref.speed, 1.5, label.c_str()));
            if (next == 1) {  // range check once, at sustain end
                EXPECT_TRUE(NearPct(r.Tf().pos.x(), ref.distance, 1.5, "range @ sustain end"));
            }
            ++next;
        }
    }
}

// A uniform wind is a Galilean shift: through the air the missile flies
// exactly the still-air profile from its launch AIRSPEED, and over the
// ground it moves at that plus the wind. 100 kt head- and tailwind on an
// eastbound shot; same reference integration as PoweredFlight.
TEST(MissileManeuvers, PoweredFlightThroughWind) {
    const double altFt = 20000.0;
    const double v0 = 900.0;  // ground speed at launch
    const GuidanceComponent params = testrig::SyntheticAim120Guidance();
    for (int windCase = 0; windCase < 2; ++windCase) {
        const char* names[] = {"headwind", "tailwind"};
        SCOPED_TRACE(names[windCase]);
        MissileRig r;
        r.ctx.environment.windSpeedKts = 100.0;
        r.ctx.environment.windFromDeg = windCase == 0 ? 90.0 : 270.0;  // from ahead, from behind
        const double windEast = WindFps(r.ctx.environment).x();
        SpawnMissile(r, Vec3{0.0, altFt, 0.0}, Vec3{v0, 0.0, 0.0}, Vec3{400000.0, altFt, 0.0}, Vec3{},
                     /*glimit=*/false);

        while (r.t < 7.99) r.Step();  // to sustain end
        RefState ref = ReferenceProfile(r.t, v0 - windEast, altFt, params);
        EXPECT_TRUE(NearPct(r.Kin().vel.x() - windEast, ref.speed, 1.5, "airspeed @ sustain end"));
        EXPECT_TRUE(NearPct(r.Tf().pos.x(), ref.distance + windEast * r.t, 1.5, "ground range @ sustain end"));
        EXPECT_NEAR(r.Kin().vel.z(), 0.0, 1e-9);  // no crosswind, no sideways push
    }
}

// Coast deceleration vs 0.5*rho*V^2*Cd*A/m -- in coast (no thrust, no
// gravity in the active branch, collinear so no PN) the sim's whole
// acceleration is drag.
TEST(MissileManeuvers, CoastDrag) {
    const double altFt = 20000.0;
    MissileRig r;
    SpawnMissile(r, Vec3{0.0, altFt, 0.0}, Vec3{900.0, 0.0, 0.0}, Vec3{400000.0, altFt, 0.0}, Vec3{}, /*glimit=*/false);

    // Fly past burnout (8 s) into coast.
    for (int i = 0; i < static_cast<int>(10.0 / kDt); ++i) r.Step();

    double v = r.Kin().speed;
    double accelMag = Norm(r.Kin().accel);
    const GuidanceComponent params = testrig::SyntheticAim120Guidance();
    double mass = params.dryMassSlug;  // propellant gone by now
    Atmosphere::AtmosphereState atm = Atmosphere::Lookup(altFt);
    double expected = 0.5 * atm.densitySlugPerFt3 * v * v * testrig::Aim120()->aero.ZeroLiftCd(v / atm.speedOfSoundFtPerSec) *
                      params.referenceAreaFt2 / mass;
    EXPECT_TRUE(NearPct(accelMag, expected, 1.0, "drag deceleration"))
        << "coast speed " << v << " ft/s (Mach " << v / atm.speedOfSoundFtPerSec << ")";
}

// Missile V-n envelope vs closed form: min(CN(alpha_max) q S / W, structural).
TEST(MissileManeuvers, EnvelopeModel) {
    const MissileGLimitComponent g = MissileEntity::MakeGLimit(*testrig::Aim120());

    struct Point {
        double vFps;
        double altFt;
        double massSlug;
        const char* regime;
    };
    const Point points[] = {{1500.0, 20000.0, 8.0, "aero-limited"},
                            {2500.0, 20000.0, 6.3, "aero-limited"},
                            {4000.0, 20000.0, 6.3, "structural cap"}};
    for (const Point& p : points) {
        Atmosphere::AtmosphereState atm = Atmosphere::Lookup(p.altFt);
        // CN at alpha_max from the type's normal-force model, on the type's
        // reference area.
        double cn = g.normalForce->Cn(g.normalForce->AlphaMaxRad(), p.vFps / atm.speedOfSoundFtPerSec);
        double aero = 0.5 * atm.densitySlugPerFt3 * p.vFps * p.vFps * cn * testrig::Aim120()->aero.referenceAreaFt2 /
                      (p.massSlug * kG);
        double expected = std::min(aero, g.structuralGLimit);
        std::string label = std::string("max-G @ ") + std::to_string(static_cast<int>(p.vFps)) + " ft/s (" +
                             p.regime + ")";
        EXPECT_TRUE(
            NearPct(FlightEnvelope::GetMissileMaxAvailableG(p.vFps, p.altFt, p.massSlug, g), expected, 0.5, label.c_str()));
    }
}

// Crossing intercept: lateral command equals N*Vc*lambda_dot.
TEST(MissileManeuvers, ProportionalNavigation) {
    const double altFt = 20000.0;
    MissileRig r;
    // Missile heading +X; target crossing left-to-right ahead of it, so
    // the LOS rotates and PN must pull lateral G to intercept.
    SpawnMissile(r, Vec3{0.0, altFt, 0.0}, Vec3{2500.0, 0.0, 0.0}, Vec3{25000.0, altFt, -8000.0},
                 Vec3{0.0, 0.0, 900.0}, /*glimit=*/true);
    GuidanceComponent& g = r.world.Components<GuidanceComponent>().Get(r.missile);

    const GuidanceComponent params = testrig::SyntheticAim120Guidance();  // physical constants for isolating thrust/drag
    const double burn = params.boostDurationS + params.sustainDurationS;
    Atmosphere::AtmosphereState atm = Atmosphere::Lookup(altFt);  // stays level (no gravity, planar turn)

    double minRange = 1e18;
    bool identityChecked = false;
    // Seeded to the initial LOS, matching GuidanceComponent::prevLosUnit,
    // so the measured LOS-rate interval lines up tick-for-tick with the
    // one the sim's acceleration was computed from.
    Vec3 losPrev = Unit(Sub(r.TargetTf().pos, r.Tf().pos));

    for (int i = 0; i < static_cast<int>(15.0 / kDt); ++i) {
        Vec3 losCur = Unit(Sub(r.TargetTf().pos, r.Tf().pos));
        Vec3 velCur = r.Kin().vel;
        Vec3 tgtVel = r.world.Components<KinematicsComponent>().Get(r.target).vel;
        double tBefore = r.t;
        r.Step();

        double range = Norm(Sub(r.TargetTf().pos, r.Tf().pos));
        minRange = std::min(minRange, range);

        // Check the PN law once, well into the engagement while guiding hard.
        if (!identityChecked && tBefore > 1.5) {
            // LOS rate over exactly the interval the sim's accel used.
            Vec3 losRate = Scale(Sub(losCur, losPrev), 1.0 / kDt);
            double lambdaDot = Norm(losRate);
            double vClosing = Dot(losCur, Sub(velCur, tgtVel));
            double expected = g.navGain * vClosing * lambdaDot;

            // Isolate the PN command from the total accel by subtracting
            // the (independently computed) axial thrust and drag.
            double v = Norm(velCur);
            Vec3 vHat = Unit(velCur);
            double mass = params.dryMassSlug + params.propellantMassSlug * (1.0 - std::clamp(tBefore / burn, 0.0, 1.0));
            double thrust = 0.0;
            if (tBefore < params.boostDurationS) thrust = params.boostThrustLbf;
            else if (tBefore < burn) thrust = params.sustainThrustLbf;
            if (!(v > 0.001)) thrust = 0.0;
            double dragMag = 0.5 * atm.densitySlugPerFt3 * v * v * testrig::Aim120()->aero.ZeroLiftCd(v / atm.speedOfSoundFtPerSec) *
                             params.referenceAreaFt2 / mass;
            Vec3 pn = Sub(Sub(r.Kin().accel, Scale(vHat, thrust / mass)), Scale(vHat, -dragMag));
            EXPECT_TRUE(NearPct(Norm(pn), expected, 5.0, "lateral accel = N*Vc*lambda_dot"));
            identityChecked = true;
        }
        losPrev = losCur;
    }

    EXPECT_LT(minRange, 60.0) << "closest approach " << minRange << " ft";
}
