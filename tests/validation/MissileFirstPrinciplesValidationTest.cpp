// Missile propulsion, drag, normal force, turn ceiling and response lag
// checked against closed forms and published figures that do not pass
// through the model's own formulas: the rocket equation, the analytic
// solution of quadratic drag, textbook induced drag, the frequency response
// of a first-order lag, and the ISA density table. Where a check needs the
// type's CN(alpha) it inverts it by bisection on Cn() rather than calling
// the closed-form AlphaForCn(), so the two paths can disagree.
//
// Air density and speed of sound come from the published 1976 standard
// atmosphere (table values and the 49.02 sqrt(T) form), not Atmosphere::Lookup.

#include <algorithm>
#include <cmath>
#include <memory>
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
#include "sim/FlightEnvelope.h"
#include "sim/ResponseLag.h"
#include "sim/SimContext.h"
#include "systems/GuidanceSystem.h"
#include "systems/IntegrationSystem.h"
#include "util/Units.h"

namespace {

using validation::NearPct;

constexpr double kStdGravity = 32.174;  // ft/s^2
constexpr double kPi = 3.14159265358979323846;

// 1976 standard atmosphere, density in slug/ft^3 (table values).
constexpr double kRho1000Ft = 0.0023081;
constexpr double kRho5000Ft = 0.0020482;
constexpr double kRho10000Ft = 0.0017556;
constexpr double kRho20000Ft = 0.0012673;
constexpr double kRho25000Ft = 0.0010663;

// Speed of sound below the tropopause: sqrt(gamma R T), T = 518.67 - 0.00356616 h (degR).
double SoundSpeedFtPerSec(double altFt) {
    return std::sqrt(1.4 * 1716.49 * (518.67 - 0.00356616 * altFt));
}

std::shared_ptr<const MissileType> Type() { return testrig::Aim120(); }

// A missile flying along +x at `altFt` with a stationary target far ahead on
// the same line, so proportional navigation commands nothing and the
// acceleration is thrust, drag, gravity and the gravity bias only.
struct Rig {
    World world;
    SimContext ctx;
    EntityID missile = 0;
    double t = 0.0;

    Rig(double altFt, double speed, double timeSinceLaunchS, bool withGLimit) {
        ctx.rng.Seed(1);
        const Vec3 pos{0.0, altFt, 0.0};
        const Vec3 targetPos{2.0e6, altFt, 0.0};
        EntityID target = testrig::SpawnMover(world, targetPos, Vec3{});
        missile = testrig::SpawnMover(world, pos, Vec3{speed, 0.0, 0.0});
        GuidanceComponent g = testrig::GiveAim120(world, missile);
        g.active = true;
        g.launched = true;
        g.launchSimTime = -timeSinceLaunchS;
        g.target = target;
        g.launchPos = pos;
        g.prevLosUnit = Vec3{1.0, 0.0, 0.0};
        world.Components<GuidanceComponent>().Add(missile, g);
        if (withGLimit) world.Components<MissileGLimitComponent>().Add(missile, MissileEntity::MakeGLimit(*Type()));
    }

    void Step(double dt) {
        GuidanceSystem::Update(world, ctx, dt, t);
        IntegrationSystem::Update(world, dt);
        t += dt;
    }
    void Run(double seconds, double dt) {
        const int ticks = static_cast<int>(std::lround(seconds / dt));
        for (int i = 0; i < ticks; ++i) Step(dt);
    }
    KinematicsComponent& Kin() { return world.Components<KinematicsComponent>().Get(missile); }
    Transform& Tf() { return world.Components<Transform>().Get(missile); }
};

}  // namespace

// ---------------------------------------------------------------------------
// The type's numbers against what is published for the AIM-120
// ---------------------------------------------------------------------------

// Published launch mass 152 kg (SOURCES: Wikipedia AIM-120).
TEST(MissileFirstPrinciples, LaunchMassIsThePublished152Kg) {
    const MissileMass& m = Type()->mass;
    EXPECT_TRUE(NearPct((m.dryMassSlug + m.propellantMassSlug) * Units::kKgPerSlug, 152.0, 0.5, "launch mass, kg"));
}

// Drag reference area is the body cross-section of the published 178 mm diameter.
TEST(MissileFirstPrinciples, ReferenceAreaIsTheBodyCrossSectionOfThePublishedDiameter) {
    const double diameterFt = 0.178 * 3.280839895;
    EXPECT_TRUE(NearPct(Type()->aero.referenceAreaFt2, kPi / 4.0 * diameterFt * diameterFt, 1.0, "reference area, ft2"));
}

// Specific impulse = total impulse / propellant weight. A solid composite motor
// delivers 230-260 s (SOURCES: Solid propellant Isp); outside that band the
// thrust, burn time and propellant mass cannot all be right.
TEST(MissileFirstPrinciples, MotorSpecificImpulseIsWhatASolidMotorCanDeliver) {
    const MissilePropulsion& p = Type()->propulsion;
    const double impulseLbfS = p.boostThrustLbf * p.boostDurationS + p.sustainThrustLbf * p.sustainDurationS;
    const double isp = impulseLbfS / (Type()->mass.propellantMassSlug * kStdGravity);
    EXPECT_GE(isp, 230.0);
    EXPECT_LE(isp, 260.0);
}

// A boost thrust-to-weight of 15-20 at launch is the range of beyond-visual-range
// air-to-air missiles; a unit slip (slug vs kg, lbf vs N) lands far outside it.
TEST(MissileFirstPrinciples, LaunchThrustToWeightIsInTheRangeOfAnAirToAirMissile) {
    const MissileType& t = *Type();
    const double weightLbf = (t.mass.dryMassSlug + t.mass.propellantMassSlug) * kStdGravity;
    const double ratio = t.propulsion.boostThrustLbf / weightLbf;
    EXPECT_GT(ratio, 10.0);
    EXPECT_LT(ratio, 25.0);
}

// ---------------------------------------------------------------------------
// Zero-lift drag table
// ---------------------------------------------------------------------------

// The documented table (REFERENCE 6.4), at its knots and halfway between them.
TEST(MissileFirstPrinciples, DragTableMatchesTheDocumentedValues) {
    const DragTable& drag = Type()->aero.drag;
    const double machs[] = {0.0, 0.5, 0.8, 0.9, 1.0, 1.1, 1.2, 1.5, 2.0, 3.0, 4.0};
    const double cds[] = {0.30, 0.30, 0.35, 0.45, 0.60, 0.58, 0.50, 0.40, 0.35, 0.32, 0.30};
    for (int i = 0; i < 11; ++i) {
        EXPECT_NEAR(drag.Lookup(machs[i]), cds[i], 1e-12) << "knot M" << machs[i];
        if (i > 0) {
            EXPECT_NEAR(drag.Lookup(0.5 * (machs[i] + machs[i - 1])), 0.5 * (cds[i] + cds[i - 1]), 1e-12)
                << "midpoint below M" << machs[i];
        }
    }
}

// Wave drag peaks at Mach 1 and the curve has no jumps: a step in Cd would be a
// step in deceleration as the missile passes through a Mach number.
TEST(MissileFirstPrinciples, DragPeaksAtMachOneAndIsContinuous) {
    const DragTable& drag = Type()->aero.drag;
    double peakMach = 0.0, peak = 0.0;
    for (double mach = 0.0; mach <= 6.0; mach += 0.001) {
        double cd = drag.Lookup(mach);
        EXPECT_GT(cd, 0.0);
        if (cd > peak) {
            peak = cd;
            peakMach = mach;
        }
        EXPECT_LT(std::fabs(drag.Lookup(mach + 0.001) - cd), 0.005) << "jump at M" << mach;
    }
    EXPECT_NEAR(peakMach, 1.0, 0.05);
}

// ---------------------------------------------------------------------------
// Propulsion and drag in flight
// ---------------------------------------------------------------------------

// Tsiolkovsky: with drag negligible (250 000 ft, drag under 0.1 lbf against a
// thrust of thousands) and the mass flow constant, a stage of thrust T and
// mass flow mdot changes speed by (T / mdot) ln(m_start / m_end).
TEST(MissileFirstPrinciples, MotorFollowsTheRocketEquationInNearVacuum) {
    const MissileType& t = *Type();
    const double tb = t.propulsion.boostDurationS, ts = t.propulsion.sustainDurationS;
    const double mDry = t.mass.dryMassSlug, mProp = t.mass.propellantMassSlug;
    const double mdot = mProp / (tb + ts);
    const double m0 = mDry + mProp, m3 = m0 - mdot * tb;
    const double v0 = 900.0;
    const double dvBoost = t.propulsion.boostThrustLbf / mdot * std::log(m0 / m3);
    const double dvSustain = t.propulsion.sustainThrustLbf / mdot * std::log(m3 / mDry);

    const double dt = 0.002;
    Rig r(250000.0, v0, 0.0, /*withGLimit=*/false);
    r.Run(tb, dt);
    EXPECT_TRUE(NearPct(r.Kin().vel.x(), v0 + dvBoost, 0.1, "speed at boost end"));
    r.Run(ts, dt);
    EXPECT_TRUE(NearPct(r.Kin().vel.x(), v0 + dvBoost + dvSustain, 0.1, "speed at burnout"));
    r.Run(4.0, dt);
    EXPECT_TRUE(NearPct(r.Kin().vel.x(), v0 + dvBoost + dvSustain, 0.1, "speed after 4 s of coast"));
    EXPECT_LT(std::fabs(r.Kin().vel.y()), 1e-3);  // gravity bias holds it level
    EXPECT_LT(std::fabs(r.Kin().vel.z()), 1e-3);
}

// With Cd constant (subsonic, Mach < 0.5), constant density and constant mass,
// dV/dt = -k V^2 has the closed form V = V0 / (1 + k V0 t), x = ln(1 + k V0 t) / k
// with k = rho Cd S / (2 m). Level flight keeps the density constant: the
// gravity bias holds the missile at its altitude.
TEST(MissileFirstPrinciples, SubsonicCoastFollowsTheQuadraticDragSolution) {
    const double altFt = 1000.0, v0 = 500.0;
    ASSERT_LT(v0, 0.5 * SoundSpeedFtPerSec(altFt));
    const double cd = Type()->aero.drag.Lookup(0.3);
    const double k = kRho1000Ft * cd * Type()->aero.referenceAreaFt2 / (2.0 * Type()->mass.dryMassSlug);

    const double dt = 1.0 / 120.0;
    Rig r(altFt, v0, 100.0, /*withGLimit=*/false);  // 100 s since launch: burnt out, dry mass
    double elapsed = 0.0;
    for (double checkpoint : {10.0, 20.0, 40.0}) {
        r.Run(checkpoint - elapsed, dt);
        elapsed = checkpoint;
        std::string label = "t = " + std::to_string(static_cast<int>(checkpoint)) + " s";
        EXPECT_TRUE(NearPct(r.Kin().vel.x(), v0 / (1.0 + k * v0 * elapsed), 0.1, (label + " speed").c_str()));
        EXPECT_TRUE(NearPct(r.Tf().pos.x(), std::log(1.0 + k * v0 * elapsed) / k, 0.1, (label + " distance").c_str()));
        EXPECT_NEAR(r.Tf().pos.y(), altFt, 5.0) << label;
    }
}

// In level flight at 1 g the airframe makes lift N = m g and, tilted by alpha, N sin(alpha)
// of it points back along the flight path. So the coast deceleration is
// q S Cd0 / m + g sin(alpha), with alpha the angle at which the type's CN(alpha, M)
// equals m g / (q S). alpha comes from bisection on Cn().
TEST(MissileFirstPrinciples, LevelCoastPaysInducedDragAtOneG) {
    struct Case {
        double altFt, speed, rho;
    };
    const Case cases[] = {{20000.0, 1500.0, kRho20000Ft}, {20000.0, 2500.0, kRho20000Ft}, {10000.0, 1000.0, kRho10000Ft}};
    const MissileType& type = *Type();
    const double mass = type.mass.dryMassSlug, area = type.aero.referenceAreaFt2;
    const MissileNormalForce& nf = *type.aero.normalForce;

    for (const Case& c : cases) {
        const double mach = c.speed / SoundSpeedFtPerSec(c.altFt);
        const double q = 0.5 * c.rho * c.speed * c.speed;
        const double cnNeeded = mass * kStdGravity / (q * area);
        ASSERT_LT(cnNeeded, nf.CnMax(mach));

        double lo = 0.0, hi = nf.AlphaMaxRad();
        for (int i = 0; i < 80; ++i) {
            double mid = 0.5 * (lo + hi);
            (nf.Cn(mid, mach) < cnNeeded ? lo : hi) = mid;
        }
        const double alpha = 0.5 * (lo + hi);
        const double expectedDecel = q * area * type.aero.ZeroLiftCd(mach) / mass + kStdGravity * std::sin(alpha);

        Rig r(c.altFt, c.speed, 100.0, /*withGLimit=*/true);
        r.Step(1.0 / 120.0);
        const Vec3 a = r.Kin().accel;
        std::string label = "M" + std::to_string(mach) + " at " + std::to_string(static_cast<int>(c.altFt)) + " ft";
        EXPECT_TRUE(NearPct(-a.x(), expectedDecel, 0.5, label.c_str())) << "alpha " << alpha * Units::kRadToDeg << " deg";
        EXPECT_LT(std::fabs(a.y()), 1e-3) << label;  // lift cancels gravity
        EXPECT_LT(std::fabs(a.z()), 1e-3) << label;
    }
}

// ---------------------------------------------------------------------------
// Turn ceiling
// ---------------------------------------------------------------------------

// n = q CN_max S / (m g) at the REFERENCE 6.5 anchor points (30.0 G early, 19.1 G late
// at 25 000 ft), with q from the table density and CN_max from the type.
TEST(MissileFirstPrinciples, TurnCeilingMatchesTheDocumentedAnchorsAndTheHandCalculation) {
    const MissileType& type = *Type();
    const MissileGLimitComponent g = MissileEntity::MakeGLimit(type);
    const double fullMass = type.mass.dryMassSlug + type.mass.propellantMassSlug;
    const double altFt = 25000.0;

    EXPECT_TRUE(NearPct(FlightEnvelope::GetMissileMaxAvailableG(2500.0, altFt, fullMass, g), 30.0, 0.5, "early"));
    EXPECT_TRUE(NearPct(FlightEnvelope::GetMissileMaxAvailableG(1200.0, altFt, type.mass.dryMassSlug, g), 19.1, 1.0, "late"));

    const double v = 1200.0;
    const double byHand = 0.5 * kRho25000Ft * v * v * g.normalForce->CnMax(v / SoundSpeedFtPerSec(altFt)) *
                          type.aero.referenceAreaFt2 / (type.mass.dryMassSlug * kStdGravity);
    EXPECT_TRUE(NearPct(FlightEnvelope::GetMissileMaxAvailableG(v, altFt, type.mass.dryMassSlug, g), byHand, 0.5, "by hand"));
}

// Below the structural cap, the load factor an airframe can generate is the
// normal force over weight, so n times mass is independent of mass.
TEST(MissileFirstPrinciples, TurnCeilingFallsInverselyWithMass) {
    const MissileGLimitComponent g = MissileEntity::MakeGLimit(*Type());
    const double speed = 900.0, altFt = 25000.0;
    const double light = Type()->mass.dryMassSlug, heavy = light + Type()->mass.propellantMassSlug;
    const double nLight = FlightEnvelope::GetMissileMaxAvailableG(speed, altFt, light, g);
    const double nHeavy = FlightEnvelope::GetMissileMaxAvailableG(speed, altFt, heavy, g);
    ASSERT_LT(nLight, g.structuralGLimit);
    EXPECT_NEAR(nLight * light, nHeavy * heavy, 1e-9 * nLight * light);
}

// More speed means more dynamic pressure and so no less turning ability, up to
// the structural cap, which it then holds.
TEST(MissileFirstPrinciples, TurnCeilingNeverFallsWithSpeed) {
    const MissileGLimitComponent g = MissileEntity::MakeGLimit(*Type());
    for (double altFt : {5000.0, 25000.0, 50000.0}) {
        double previous = 0.0;
        for (double speed = 200.0; speed <= 4500.0; speed += 10.0) {
            double n = FlightEnvelope::GetMissileMaxAvailableG(speed, altFt, Type()->mass.dryMassSlug, g);
            EXPECT_GE(n, previous - 1e-9) << speed << " ft/s at " << altFt << " ft";
            EXPECT_LE(n, g.structuralGLimit + 1e-9);
            previous = n;
        }
    }
}

// ---------------------------------------------------------------------------
// Normal force
// ---------------------------------------------------------------------------

// Fin and body lift do not care which way the nose is off the flight path.
TEST(MissileFirstPrinciples, NormalForceIsEvenInAngleOfAttack) {
    const MissileNormalForce& nf = *Type()->aero.normalForce;
    for (double mach : {0.4, 0.95, 1.6, 3.2}) {
        for (double deg : {0.5, 5.0, 17.0, 29.0}) {
            EXPECT_DOUBLE_EQ(nf.Cn(-deg * Units::kDegToRad, mach), nf.Cn(deg * Units::kDegToRad, mach));
        }
    }
}

// The slope at zero angle is the tabulated lift-curve slope, per degree.
TEST(MissileFirstPrinciples, NormalForceSlopeAtZeroAlphaIsTheTabulatedSlope) {
    const MissileNormalForce& nf = *Type()->aero.normalForce;
    const double machs[] = {0.5, 1.0, 2.0, 3.0, 4.0};
    const double slopes[] = {0.3412, 0.3009, 0.2068, 0.183, 0.1922};
    const double alphaDeg = 0.001;
    for (int i = 0; i < 5; ++i) {
        EXPECT_TRUE(NearPct(nf.Cn(alphaDeg * Units::kDegToRad, machs[i]) / alphaDeg, slopes[i], 0.2, "CN_alpha per deg"))
            << "M" << machs[i];
    }
}

// Neither CN_alpha nor CN_max may jump with Mach: a step would be a step in
// available G as a missile passes the Mach number. Also guards the units: for a
// wing-and-tail missile on body-area reference, CN_alpha is of order 10 per
// radian, so a per-degree/per-radian slip lands far outside 4-40.
TEST(MissileFirstPrinciples, NormalForceIsContinuousInMachAndOfTheRightOrder) {
    const MissileNormalForce& nf = *Type()->aero.normalForce;
    const double alpha = 5.0 * Units::kDegToRad;
    for (double mach = 0.2; mach <= 4.5; mach += 0.001) {
        EXPECT_LT(std::fabs(nf.CnMax(mach + 0.001) - nf.CnMax(mach)), 0.1) << "CN_max jump at M" << mach;
        EXPECT_LT(std::fabs(nf.Cn(alpha, mach + 0.001) - nf.Cn(alpha, mach)), 0.01) << "CN jump at M" << mach;
        double perRad = nf.Cn(0.001 * Units::kDegToRad, mach) / (0.001 * Units::kDegToRad);
        EXPECT_GT(perRad, 4.0) << "M" << mach;
        EXPECT_LT(perRad, 40.0) << "M" << mach;
    }
}

// ---------------------------------------------------------------------------
// Guidance response lag: a first-order lag, H(s) = 1 / (1 + tau s)
// ---------------------------------------------------------------------------

namespace {

// Demand along +y, air velocity along +z (so the lift stays across the body axis).
const Vec3 kAxis{0.0, 0.0, 2500.0};

}  // namespace

// Following a ramp of slope r, a first-order lag settles a constant r*tau behind it.
TEST(MissileFirstPrinciples, LagTrailsARampByRateTimesTau) {
    const double tau = 0.35, dt = tau / 100.0, rate = 50.0;
    Vec3 achieved{};
    double demandY = 0.0;
    for (int n = 1; n <= 2000; ++n) {  // 20 tau
        demandY = rate * n * dt;
        ResponseLag::Step(achieved, Vec3{0.0, demandY, 0.0}, kAxis, tau, dt);
    }
    EXPECT_TRUE(NearPct(demandY - achieved.y(), rate * tau, 0.5, "steady ramp error"));
}

// Steady-state sinusoid response: gain 1/sqrt(1 + (w tau)^2), phase lag atan(w tau).
TEST(MissileFirstPrinciples, LagFrequencyResponseIsTheFirstOrderLag) {
    const double tau = 0.35;
    const int stepsPerPeriod = 2400;
    for (double wTau : {0.2, 1.0, 5.0}) {
        const double w = wTau / tau;
        const double dt = 2.0 * kPi / (w * stepsPerPeriod);
        const int settlePeriods = static_cast<int>(std::ceil(20.0 * tau * w / (2.0 * kPi)));
        Vec3 achieved{};
        double sinProj = 0.0, cosProj = 0.0;
        for (int n = 1; n <= (settlePeriods + 1) * stepsPerPeriod; ++n) {
            double phase = w * n * dt;
            ResponseLag::Step(achieved, Vec3{0.0, std::sin(phase), 0.0}, kAxis, tau, dt);
            if (n > settlePeriods * stepsPerPeriod) {
                sinProj += achieved.y() * std::sin(phase);
                cosProj += achieved.y() * std::cos(phase);
            }
        }
        const double a = 2.0 * sinProj / stepsPerPeriod, b = 2.0 * cosProj / stepsPerPeriod;
        const double gain = std::sqrt(a * a + b * b);
        const double lagDeg = std::atan2(-b, a) * Units::kRadToDeg;
        EXPECT_TRUE(NearPct(gain, 1.0 / std::sqrt(1.0 + wTau * wTau), 1.0, "gain")) << "w tau = " << wTau;
        EXPECT_NEAR(lagDeg, std::atan(wTau) * Units::kRadToDeg, 1.0) << "w tau = " << wTau;
    }
}

// The step response approaches 1 - exp(-t/tau) as the step shrinks, with error
// proportional to the step (first order).
TEST(MissileFirstPrinciples, LagStepResponseConvergesToTheContinuousLag) {
    const double tau = 0.35;
    auto errorAtOneTau = [&](int stepsPerTau) {
        const double dt = tau / stepsPerTau;
        Vec3 achieved{};
        for (int n = 0; n < stepsPerTau; ++n) ResponseLag::Step(achieved, Vec3{0.0, 1.0, 0.0}, kAxis, tau, dt);
        return std::fabs(achieved.y() - (1.0 - std::exp(-1.0)));
    };
    const double e50 = errorAtOneTau(50), e100 = errorAtOneTau(100), e200 = errorAtOneTau(200);
    EXPECT_LT(e100 / (1.0 - std::exp(-1.0)), 0.01);
    EXPECT_NEAR(e50 / e100, 2.0, 0.2);
    EXPECT_NEAR(e100 / e200, 2.0, 0.2);
}

// A real lag never overshoots a step and never reverses, whatever the step
// size, including steps much longer than tau (the output then reaches the demand).
TEST(MissileFirstPrinciples, LagStepIsMonotoneAndNeverOvershoots) {
    const double tau = 0.35;
    for (double dtOverTau : {0.001, 0.05, 1.0, 20.0, 1000.0}) {
        const double dt = dtOverTau * tau;
        Vec3 achieved{};
        double previous = 0.0;
        for (int n = 0; n < 200; ++n) {
            ResponseLag::Step(achieved, Vec3{0.0, 100.0, 0.0}, kAxis, tau, dt);
            EXPECT_GE(achieved.y(), previous) << "dt/tau " << dtOverTau;
            EXPECT_LE(achieved.y(), 100.0 + 1e-9) << "dt/tau " << dtOverTau;
            previous = achieved.y();
        }
    }
    Vec3 achieved{};
    ResponseLag::Step(achieved, Vec3{0.0, 100.0, 0.0}, kAxis, tau, 1000.0 * tau);
    EXPECT_NEAR(achieved.y(), 100.0, 0.2);
}
