// Known-answer physics checks. Every check re-derives its expected result
// independently -- plain doubles, not the production code's Eigen/inline
// paths -- from the documented equation itself, rather than checking
// end-to-end behavior.
//
// The local Add/Sub/Scale/Dot/Length/Normalize Vec3 helpers below are
// kept as their own independent re-derivation rather than pointed at
// Eigen's operations, per REFERENCE.md 15.1's "must re-derive, not re-call".

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "FGFDMExec.h"
#include "models/atmosphere/FGStandardAtmosphere.h"

#include "TestRigs.h"
#include "aircraft/AircraftCatalog.h"
#include "aircraft/AircraftEntity.h"
#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/DatalinkTrackComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GLimitComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Transform.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileEntity.h"
#include "scenario/ScenarioLoader.h"
#include "sim/AircraftBounds.h"
#include "sim/Atmosphere.h"
#include "sim/FlightEnvelope.h"
#include "sim/SimContext.h"
#include "systems/GuidanceSystem.h"
#include "systems/IntegrationSystem.h"
#include "systems/InterceptSystem.h"
#include "sim/RadarEquation.h"
#include "sim/RadarTrackFilter.h"
#include "sim/HitWindow.h"
#include "sim/ResponseLag.h"
#include "systems/AirborneRadarSystem.h"
#include "systems/DatalinkSystem.h"
#include "util/Units.h"

namespace {

// The F-16's airframe, which the miss-distance checks measure against. A
// function, not a namespace-scope constant: reading the type file must wait
// until main has moved to the repo root (tests/TestMain.cpp).
const AircraftBounds::HitBox& F16Box() { return testrig::F16()->hitBox; }

Vec3 Add(Vec3 a, Vec3 b) { return Vec3{a.x() + b.x(), a.y() + b.y(), a.z() + b.z()}; }
Vec3 Sub(Vec3 a, Vec3 b) { return Vec3{a.x() - b.x(), a.y() - b.y(), a.z() - b.z()}; }
Vec3 Scale(Vec3 a, double s) { return Vec3{a.x() * s, a.y() * s, a.z() * s}; }
double Dot(Vec3 a, Vec3 b) { return a.x() * b.x() + a.y() * b.y() + a.z() * b.z(); }
double Length(Vec3 a) { return std::sqrt(Dot(a, a)); }
Vec3 Normalize(Vec3 a) {
    double len = Length(a);
    return len > 0.0 ? Scale(a, 1.0 / len) : Vec3{};
}

// A TRUE_PN command as the fins fly it: the part along the body axis (the
// velocity, no wind in these checks) removed, the rest scaled back up to
// the command's own magnitude. A command with nothing across the axis is
// dropped. See REFERENCE.md - Steering across the body axis.
Vec3 AcrossAxisKeepingMagnitude(Vec3 command, Vec3 vel) {
    Vec3 axis = Normalize(vel);
    Vec3 across = Sub(command, Scale(axis, Dot(command, axis)));
    double acrossLen = Length(across);
    if (acrossLen <= 1e-9 * Length(command)) return Vec3{};
    return Scale(across, Length(command) / acrossLen);
}

// Closest approach must be confirmed over kCpaConfirmTicks consecutive
// ticks, not taken on one sample -- walks the missile outward along +X by
// 1ft/tick, calling Update once per tick. Shared by the arming-range and
// probabilistic-lethality checks below. The missile is set up at its
// closest approach, so it has been closing to get there.
void RunCpaConfirmationTicks(World& world, SimContext& simContext, const ScenarioData& scenario, EntityID missile) {
    world.Components<GuidanceComponent>().Get(missile).rangeHasClosed = true;
    constexpr int kTicks = 4;  // one more than kCpaConfirmTicks, so confirmation is reached, not just approached
    for (int tick = 0; tick < kTicks; ++tick) {
        InterceptSystem::Update(world, simContext, scenario, 1.0);
        world.Components<Transform>().Get(missile).pos.x() += 1.0;
    }
}

}  // namespace

// [V1] Missile integration vs. closed-form kinematics. IntegrationSystem
// never touches KinematicsComponent.accel itself, so holding it constant
// across every step is exactly the precondition under which
// x(t) = x0 + v0*t + 1/2*a*t^2 is exact, not an approximation.
TEST(PhysicsValidation, IntegrationVsClosedFormKinematics) {
    World world;
    EntityID entity = world.CreateEntity();

    Vec3 pos0{0.0, 5000.0, 0.0};
    Vec3 vel0{100.0, 0.0, 50.0};
    Vec3 accel{10.0, 0.0, -5.0};

    world.Components<Transform>().Add(entity, Transform{pos0, Vec3{}});
    KinematicsComponent kin;
    kin.vel = vel0;
    kin.accel = accel;
    world.Components<KinematicsComponent>().Add(entity, kin);

    constexpr double kDt = 0.01;
    constexpr int kTicks = 500;  // 5.0s of flight
    for (int i = 0; i < kTicks; ++i) {
        IntegrationSystem::Update(world, kDt);
    }

    double t = kDt * kTicks;
    Vec3 expectedPos{pos0.x() + vel0.x() * t + 0.5 * accel.x() * t * t, pos0.y(),
                      pos0.z() + vel0.z() * t + 0.5 * accel.z() * t * t};
    Vec3 expectedVel{vel0.x() + accel.x() * t, 0.0, vel0.z() + accel.z() * t};

    const Transform& transform = world.Components<Transform>().Get(entity);
    const KinematicsComponent& kinAfter = world.Components<KinematicsComponent>().Get(entity);

    double posErr = Length(Sub(transform.pos, expectedPos));
    double velErr = Length(Sub(kinAfter.vel, expectedVel));
    EXPECT_LT(posErr, 1e-6);
    EXPECT_LT(velErr, 1e-9);
}

// [V2] Monostatic radar equation, SNR form, and the square-law detector
// it feeds. V2a-c check the mean SNR exactly (no RNG); V2d-g check the
// detector's statistics by Monte Carlo against closed forms and published
// values; V2h that one seed reproduces one look.
TEST(PhysicsValidation, RadarEquation) {
    // An arbitrary but complete budget: every term must reach the result.
    const ReceiverBudget budget{200.0, 2000.0, 0.03, 4.0, 0.047, 6.0};
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kBoltzmann = 1.380649e-23, kT0 = 290.0;

    // V2a: exact magnitude of the mean SNR.
    constexpr double kRangeM = 8000.0;
    double expectedSnr = (budget.averagePowerW * budget.antennaGain * budget.antennaGain * budget.wavelengthM *
                          budget.wavelengthM * 5.0 * budget.integrationTimeS) /
                         (std::pow(4.0 * kPi, 3) * std::pow(kRangeM, 4) * kBoltzmann * kT0 *
                          std::pow(10.0, budget.noiseFigureDb / 10.0) * std::pow(10.0, budget.systemLossDb / 10.0));
    EXPECT_LT(std::fabs(RadarEquation::SnrDb(budget, 5.0, kRangeM) - 10.0 * std::log10(expectedSnr)), 1e-9);

    // V2b: R^4 falloff exponent.
    constexpr double kR1 = 4000.0, kR2 = 16000.0;
    double expectedDeltaDb = 40.0 * std::log10(kR2 / kR1);  // SNR ~ R^-4
    EXPECT_LT(std::fabs(RadarEquation::SnrDb(budget, 5.0, kR1) - RadarEquation::SnrDb(budget, 5.0, kR2) -
                        expectedDeltaDb),
              1e-9);

    // V2c: ReferenceRangeM inverts SnrDb.
    double referenceM = RadarEquation::ReferenceRangeM(budget, 5.0, 11.23);
    EXPECT_LT(std::fabs(RadarEquation::SnrDb(budget, 5.0, referenceM) - 11.23), 1e-9);

    // The fraction of `looks` detector outputs at mean SNR meanSnrDb that
    // cross thresholdDb.
    auto detectedFraction = [&](FluctuationModel model, double meanSnrDb, double thresholdDb, int looks) {
        RadarCrossSection rcs{5.0, model};
        double rangeM = RadarEquation::ReferenceRangeM(budget, 5.0, meanSnrDb);
        SimContext ctx;
        ctx.rng.Seed(918273645ULL);
        int hits = 0;
        for (int i = 0; i < looks; ++i) {
            if (RadarEquation::DetectorOutputDb(ctx, budget, rcs, rangeM) >= thresholdDb) ++hits;
        }
        return static_cast<double>(hits) / looks;
    };

    // V2d: noise alone crosses the threshold with probability pfa. A
    // target 200 dB down is noise to six figures; pfa 0.01 so the rate is
    // measurable (sigma ~2e-4 over 200 000 looks).
    EXPECT_NEAR(detectedFraction(FluctuationModel::STEADY, -200.0, RadarEquation::DetectionThresholdDb(0.01), 200000),
                0.01, 1e-3);

    // V2e: Swerling 1 against its closed form exp(-T / (1 + S)) (sigma
    // <= 1.6e-3 over 100 000 looks), at pfa 1e-6.
    double threshold = RadarEquation::DetectionThresholdDb(1e-6);
    EXPECT_NEAR(threshold, 10.0 * std::log10(std::log(1e6)), 1e-12);
    for (double meanSnrDb : {10.0, 12.77, 16.0, 21.14}) {
        EXPECT_NEAR(detectedFraction(FluctuationModel::SWERLING_1, meanSnrDb, threshold, 100000),
                    RadarEquation::SwerlingOneDetectionProbability(meanSnrDb, threshold), 0.006)
            << meanSnrDb << " dB";
    }

    // V2f: Swerling 1 needs 21.14 dB for Pd 0.9 at pfa 1e-6 in one look
    // (MathWorks, Modeling Radar Detectability Factors: D1 = 21.1436 dB);
    // the inverse round-trips.
    EXPECT_NEAR(RadarEquation::SwerlingOneRequiredSnrDb(threshold, 0.9), 21.1436, 1e-3);
    EXPECT_NEAR(RadarEquation::SwerlingOneDetectionProbability(RadarEquation::SwerlingOneRequiredSnrDb(threshold, 0.5),
                                                               threshold),
                0.5, 1e-12);

    // V2g: a steady target at 13.18 dB is detected 90 % of the time at
    // pfa 1e-6 -- Marcum's Q1(sqrt(2S), sqrt(2T)), computed exactly
    // (MathWorks' approximation quotes 13.12 dB for the same point).
    EXPECT_NEAR(detectedFraction(FluctuationModel::STEADY, 13.18, threshold, 100000), 0.900, 0.006);

    // V2h: one seed, one look.
    RadarCrossSection rcs{5.0, FluctuationModel::SWERLING_1};
    SimContext a, b;
    a.rng.Seed(42);
    b.rng.Seed(42);
    EXPECT_EQ(RadarEquation::DetectorOutputDb(a, budget, rcs, kRangeM),
              RadarEquation::DetectorOutputDb(b, budget, rcs, kRangeM));
}

// [V3] Friis one-way link (RadarEquation::FriisReceivedPowerDb). This
// function draws no RNG jitter itself, so the magnitude/exponent checks
// need no seeded-RNG trick. V3c: at the same range/tx power/tx antenna
// gain, one-way Friis should come out far stronger than the round-trip
// radar equation -- the physical reason an RWR detects an illuminating
// radar before that radar detects its own reflection.
TEST(PhysicsValidation, FriisLink) {
    constexpr double kTxPowerW = 10000.0;
    constexpr double kTxAntennaGain = 2000.0;
    constexpr double kRxAntennaGain = 500.0;
    constexpr double kWavelengthM = 0.03;
    constexpr double kPi = 3.14159265358979323846;

    // V3a: exact magnitude.
    constexpr double kRangeM = 20000.0;
    double actualDb = RadarEquation::FriisReceivedPowerDb(kTxPowerW, kTxAntennaGain, kRxAntennaGain, kWavelengthM,
                                                           kRangeM);
    double expectedDb = 10.0 * std::log10(kTxPowerW) + 10.0 * std::log10(kTxAntennaGain) +
                         10.0 * std::log10(kRxAntennaGain) + 20.0 * std::log10(kWavelengthM) -
                         20.0 * std::log10(4.0 * kPi * kRangeM);
    EXPECT_LT(std::fabs(actualDb - expectedDb), 1e-9);

    // V3b: R^2 falloff exponent.
    constexpr double kR1 = 5000.0, kR2 = 20000.0;
    double db1 = RadarEquation::FriisReceivedPowerDb(kTxPowerW, kTxAntennaGain, kRxAntennaGain, kWavelengthM, kR1);
    double db2 = RadarEquation::FriisReceivedPowerDb(kTxPowerW, kTxAntennaGain, kRxAntennaGain, kWavelengthM, kR2);
    double observedDeltaDb = db1 - db2;
    double expectedDeltaDb = 20.0 * std::log10(kR2 / kR1);  // Pr ~ R^-2
    EXPECT_LT(std::fabs(observedDeltaDb - expectedDeltaDb), 1e-9);

    // V3c: physical asymmetry. The radar's own mean echo power, recovered
    // from its SNR (with no noise figure, loss, or integration, SNR x kT0
    // is the echo power for the same transmitter), against the one-way
    // link: ~8 orders of magnitude apart at this range.
    const ReceiverBudget budget{kTxPowerW, kTxAntennaGain, kWavelengthM, 0.0, 1.0, 0.0};
    double echoPowerW = std::pow(10.0, RadarEquation::SnrDb(budget, 5.0, kRangeM) / 10.0) * 1.380649e-23 * 290.0;
    double friisPrW = std::pow(10.0, actualDb / 10.0);
    EXPECT_GT(friisPrW, echoPowerW);
}

// [V4] Guidance law (TRUE_PN rotated across the body axis + gravity bias +
// thrust + drag + gravity) transcription check. Single GuidanceSystem::Update call against a
// hand-picked state, re-deriving
// accelCmd + thrustAccel + dragAccel + gravity independently in plain doubles from
// GuidanceSystem.cpp's own documented equations.
TEST(PhysicsValidation, GuidanceLaw) {
    World world;
    SimContext simContext;

    Vec3 missilePos{0.0, 5000.0, 0.0};
    Vec3 missileVel{1000.0, 0.0, 200.0};
    Vec3 targetPos{20000.0, 5000.0, 3000.0};
    Vec3 targetVel{-300.0, 0.0, 100.0};
    double navGain = 4.0;
    double dt = 0.01;
    double simTime = 0.0;

    EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);

    EntityID target = testrig::SpawnMover(world, targetPos, targetVel);

    Vec3 losUnitNow = Normalize(Sub(targetPos, missilePos));
    Vec3 prevLosUnit = Normalize(Vec3{losUnitNow.x() + 0.01, losUnitNow.y() - 0.02, losUnitNow.z() + 0.015});

    GuidanceComponent guidance = testrig::GiveAim120(world, missile);
    guidance.target = target;
    guidance.navGain = navGain;
    guidance.prevLosUnit = prevLosUnit;
    guidance.active = true;
    guidance.launched = true;
    guidance.launchSimTime = 0.0;  // simTime - launchSimTime = 0s -- inside the 3s boost window
    world.Components<GuidanceComponent>().Add(missile, guidance);

    GuidanceSystem::Update(world, simContext, dt, simTime);
    Vec3 observedAccel = world.Components<KinematicsComponent>().Get(missile).accel;

    double vClosing = Dot(losUnitNow, Sub(missileVel, targetVel));
    Vec3 losRate = Scale(Sub(losUnitNow, prevLosUnit), 1.0 / dt);
    // The LOS rate here has a part along the velocity, so the rotation is
    // exercised, not a no-op.
    Vec3 accelCmd = AcrossAxisKeepingMagnitude(Scale(losRate, navGain * vClosing), missileVel);

    // Gravity bias (always on): lift cancelling gravity across the body
    // axis, added to the command before the clamp. No wind here, so the
    // body axis is the missile's velocity.
    constexpr double kGravityFtPerSec2 = 32.174;
    const Vec3 gravity{0.0, -kGravityFtPerSec2, 0.0};
    Vec3 axis = Normalize(missileVel);
    Vec3 gravityAcross = Sub(gravity, Scale(axis, Dot(gravity, axis)));
    accelCmd = Sub(accelCmd, gravityAcross);

    constexpr double kMaxAccelFtPerSec2 = 30.0 * kGravityFtPerSec2;
    double accelCmdMag = Length(accelCmd);
    if (accelCmdMag > kMaxAccelFtPerSec2) accelCmd = Scale(accelCmd, kMaxAccelFtPerSec2 / accelCmdMag);

    double missileSpeed = Length(missileVel);
    Vec3 thrustAccel{0.0, 0.0, 0.0};
    double timeSinceLaunch = simTime - guidance.launchSimTime;
    double burnDurationS = guidance.boostDurationS + guidance.sustainDurationS;
    double burnFraction = burnDurationS > 0.0 ? std::clamp(timeSinceLaunch / burnDurationS, 0.0, 1.0) : 1.0;
    double currentMassSlug = guidance.dryMassSlug + guidance.propellantMassSlug * (1.0 - burnFraction);
    double thrustLbf = 0.0;
    if (timeSinceLaunch < guidance.boostDurationS) {
        thrustLbf = guidance.boostThrustLbf;
    } else if (timeSinceLaunch < guidance.boostDurationS + guidance.sustainDurationS) {
        thrustLbf = guidance.sustainThrustLbf;
    }
    if (thrustLbf > 0.0 && missileSpeed > 0.001) {
        thrustAccel = Scale(missileVel, (thrustLbf / currentMassSlug) / missileSpeed);
    }

    Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(missilePos.y());
    double mach = missileSpeed / atmosphere.speedOfSoundFtPerSec;
    double cd = testrig::Aim120()->aero.ZeroLiftCd(mach);
    double dynamicPressure = 0.5 * atmosphere.densitySlugPerFt3 * missileSpeed * missileSpeed;
    double dragForceLbf = dynamicPressure * cd * guidance.referenceAreaFt2;
    double dragAccelMag = dragForceLbf / currentMassSlug;
    Vec3 dragAccel = Scale(missileVel, -(dragAccelMag / missileSpeed));

    Vec3 expectedAccel = Add(Add(Add(accelCmd, thrustAccel), dragAccel), gravity);

    double errMag = Length(Sub(observedAccel, expectedAccel));
    double scale = std::max(Length(expectedAccel), 1.0);
    EXPECT_LT(errMag / scale, 1e-9);
}

// [V4b] Gravity while guided, and the autopilot's gravity bias. The
// missile has no type (drag-free) and is long burnt out (no thrust), with
// the target straight down its velocity axis (no LOS rate, so no PN), so
// its whole acceleration is gravity plus the bias (always on) and each case
// has a closed-form answer:
//   level:      0                  (lift exactly holds it up)
//   climbing:   -g sin(gamma) * axis   (only the along-axis part is left)
//   lock lost:  0                  (the bias outlives the PN command)
//   slow:       (n_max - 1) g up   (lift caps at the available G),
//               less n_max g sin(alpha_max) along the axis (drag due to lift)
TEST(PhysicsValidation, GravityBias) {
    constexpr double kGravityFtPerSec2 = 32.174;
    constexpr double kAltFt = 25000.0;
    constexpr double kTol = 1e-9;

    struct Shot {
        Vec3 vel;
        bool loseLock = false;  // target far off boresight, no memory coast
        bool withGLimit = false;
    };
    auto accelOf = [&](const Shot& shot, SimContext& simContext) -> Vec3 {
        World world;
        Vec3 missilePos{0.0, kAltFt, 0.0};
        EntityID missile = testrig::SpawnMover(world, missilePos, shot.vel);
        // Straight down the velocity axis, or square off the beam.
        Vec3 targetDir = shot.loseLock ? Vec3{0.0, 0.0, 1.0} : Normalize(shot.vel);
        Vec3 targetPos = Add(missilePos, Scale(targetDir, 1.0e6));
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::SyntheticAim120Guidance();  // no MissileComponent: drag-free
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;  // motor long burnt out
        if (shot.loseLock) guidance.memoryCoastTimeS = 0.0;
        world.Components<GuidanceComponent>().Add(missile, guidance);
        if (shot.withGLimit) {
            world.Components<MissileGLimitComponent>().Add(missile, MissileEntity::MakeGLimit(*testrig::Aim120()));
        }

        GuidanceSystem::Update(world, simContext, 0.01, 0.0);
        return world.Components<KinematicsComponent>().Get(missile).accel;
    };

    SimContext ctx;
    Vec3 levelVel{1500.0, 0.0, 0.0};

    // Bias on, level: lift cancels gravity exactly.
    EXPECT_LT(Length(accelOf(Shot{levelVel}, ctx)), kTol);

    // The same with the type's normal-force model: holding 1 G costs drag
    // due to lift, g sin(alpha) along -x, alpha the angle at which CN
    // makes 1 G. Nothing else changes.
    {
        Shot lifting{levelVel};
        lifting.withGLimit = true;
        const MissileNormalForce& nf = *testrig::Aim120()->aero.normalForce;
        Atmosphere::AtmosphereState atm = Atmosphere::Lookup(kAltFt);
        double q = 0.5 * atm.densitySlugPerFt3 * 1500.0 * 1500.0;
        double cn = testrig::SyntheticAim120Guidance().dryMassSlug * kGravityFtPerSec2 / (q * nf.ReferenceAreaFt2());
        double alpha = nf.AlphaForCn(cn, 1500.0 / atm.speedOfSoundFtPerSec);
        EXPECT_GT(alpha, 0.0);
        EXPECT_LT(alpha, 5.0 * Units::kDegToRad);  // ~1 G at Mach 1.5 is a few degrees
        Vec3 expected{-kGravityFtPerSec2 * std::sin(alpha), 0.0, 0.0};
        EXPECT_LT(Length(Sub(accelOf(lifting, ctx), expected)), 1e-9);
    }

    // Bias on, climbing at 30 deg: the along-axis part of gravity is not
    // cancellable, so the missile decelerates by g sin(gamma).
    constexpr double kGammaRad = 30.0 * 3.14159265358979323846 / 180.0;
    Vec3 climbVel{1500.0 * std::cos(kGammaRad), 1500.0 * std::sin(kGammaRad), 0.0};
    Vec3 expectedClimb = Scale(Normalize(climbVel), -kGravityFtPerSec2 * std::sin(kGammaRad));
    EXPECT_LT(Length(Sub(accelOf(Shot{climbVel}, ctx), expectedClimb)), kTol);

    // Bias on, seeker lost (target 90 deg off a 45 deg cone, no coast):
    // PN goes to zero but the autopilot still holds the airframe up.
    Shot lost{levelVel};
    lost.loseLock = true;
    SimContext lostCtx;
    EXPECT_LT(Length(accelOf(lost, lostCtx)), kTol);
    bool sawSeekerLost = false;
    for (const SimEvent& event : lostCtx.eventLog.Events()) sawSeekerLost |= event.type == "SEEKER_LOST";
    EXPECT_TRUE(sawSeekerLost);

    // Bias on, too slow for the airframe to make 1 G: the bias counts
    // against available G, so lift caps at n_max and the missile sags by
    // the rest. Flying at alpha_max, that lift also costs drag due to lift,
    // n_max g sin(alpha_max) along -x.
    Shot slow{Vec3{250.0, 0.0, 0.0}};
    slow.withGLimit = true;
    const GuidanceComponent burntOut = testrig::SyntheticAim120Guidance();
    const MissileGLimitComponent gLimit = MissileEntity::MakeGLimit(*testrig::Aim120());
    double nMax = FlightEnvelope::GetMissileMaxAvailableG(250.0, kAltFt, burntOut.dryMassSlug, gLimit);
    ASSERT_LT(nMax, 1.0);
    double inducedDrag = nMax * kGravityFtPerSec2 * std::sin(gLimit.normalForce->AlphaMaxRad());
    Vec3 expectedSlow{-inducedDrag, (nMax - 1.0) * kGravityFtPerSec2, 0.0};
    EXPECT_LT(Length(Sub(accelOf(slow, ctx), expectedSlow)), kTol);
}

// [V4c] Steering across the body axis. A drag-free, burnt-out missile
// flying level, target stationary 20 deg off the nose, with a chosen LOS
// rate that has a part along the velocity. Level, the gravity bias cancels
// gravity exactly, so its whole acceleration is the steering command:
//   true PN:             N Vc dLOS/dt with the along-axis part removed,
//                        rescaled to N Vc |dLOS/dt|  (no free thrust/braking)
//   along the axis:      a command with nothing across the axis is dropped
TEST(PhysicsValidation, SteeringAcrossBodyAxis) {
    constexpr double kDt = 0.01;
    constexpr double kNavGain = 4.0;
    constexpr double kTol = 1e-9;
    constexpr double kOffNoseRad = 20.0 * 3.14159265358979323846 / 180.0;
    const Vec3 missilePos{0.0, 25000.0, 0.0};
    const Vec3 missileVel{1500.0, 0.0, 0.0};
    const Vec3 targetPos = Add(missilePos, Vec3{1.0e5 * std::cos(kOffNoseRad), 0.0, 1.0e5 * std::sin(kOffNoseRad)});
    const Vec3 losUnit = Normalize(Sub(targetPos, missilePos));
    const Vec3 axis = Normalize(missileVel);
    const double vClosing = Dot(losUnit, missileVel);  // stationary target

    // The steering acceleration for one chosen LOS rate, plus the LOS rate
    // the system actually differenced.
    auto steeringOf = [&](Vec3 chosenLosRate, Vec3& losRateOut) -> Vec3 {
        World world;
        SimContext simContext;
        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::SyntheticAim120Guidance();  // no MissileComponent: drag-free
        guidance.target = target;
        guidance.navGain = kNavGain;
        guidance.prevLosUnit = Sub(losUnit, Scale(chosenLosRate, kDt));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;  // motor long burnt out
        world.Components<GuidanceComponent>().Add(missile, guidance);

        GuidanceSystem::Update(world, simContext, kDt, 0.0);
        losRateOut = Scale(Sub(losUnit, guidance.prevLosUnit), 1.0 / kDt);
        return world.Components<KinematicsComponent>().Get(missile).accel;
    };

    // A LOS rate with parts along, up and across the velocity.
    const Vec3 tiltedLosRate{0.003, 0.004, -0.002};

    // Rotated, magnitude kept, nothing along the axis.
    Vec3 losRate;
    Vec3 truePn = steeringOf(tiltedLosRate, losRate);
    Vec3 truePnRaw = Scale(losRate, kNavGain * vClosing);
    ASSERT_GT(std::fabs(Dot(Normalize(truePnRaw), axis)), 0.1);  // the rotation is not a no-op
    Vec3 expectedTrue = AcrossAxisKeepingMagnitude(truePnRaw, missileVel);
    EXPECT_LT(Length(Sub(truePn, expectedTrue)) / Length(expectedTrue), kTol);
    EXPECT_LT(std::fabs(Dot(truePn, axis)) / Length(truePn), kTol);
    EXPECT_LT(std::fabs(Length(truePn) - Length(truePnRaw)) / Length(truePnRaw), kTol);

    // Commanding straight along the axis: lift cannot supply it and there
    // is no across direction to rotate it into, so nothing is flown.
    Vec3 alongAxis = steeringOf(Vec3{0.005, 0.0, 0.0}, losRate);
    ASSERT_GT(kNavGain * vClosing * Length(losRate), 10.0);  // a real command was asked for
    EXPECT_LT(Length(alongAxis), kTol);
}

// [V9] Seeker gimbal FOV cone with memory/coast. Three GuidanceSystem::
// Update calls: in-cone (locked, fresh LOS rate), out-of-cone-but-
// coasting (frozen last-known LOS rate), and out-of-cone-with-coast-
// expired (zeroed command, SEEKER_LOST logged).
TEST(PhysicsValidation, SeekerGimbalCone) {
    constexpr double kDt = 0.01;

    // V9a: in-cone (~3deg boresight, well inside the AIM-120's 45deg cone).
    // Confirms hasLock stays true and lastLosRate is refreshed.
    {
        World world;
        SimContext simContext;
        Vec3 missilePos{0.0, 5000.0, 0.0};
        Vec3 missileVel{1000.0, 0.0, 200.0};
        Vec3 targetPos{20000.0, 5000.0, 3000.0};
        Vec3 targetVel{-300.0, 0.0, 100.0};

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = testrig::SpawnMover(world, targetPos, targetVel);

        Vec3 losUnitNow = Normalize(Sub(targetPos, missilePos));
        Vec3 prevLosUnit = Normalize(Vec3{losUnitNow.x() + 0.01, losUnitNow.y() - 0.02, losUnitNow.z() + 0.015});
        Vec3 expectedLosRate = Scale(Sub(losUnitNow, prevLosUnit), 1.0 / kDt);

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = prevLosUnit;
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;  // well past boost+sustain -- isolates the guidance command, no thrust term to track
        world.Components<GuidanceComponent>().Add(missile, guidance);

        GuidanceSystem::Update(world, simContext, kDt, 0.0);
        const GuidanceComponent& after = world.Components<GuidanceComponent>().Get(missile);
        double lastLosRateErr = Length(Sub(after.lastLosRate, expectedLosRate));
        EXPECT_TRUE(after.hasLock);
        EXPECT_LT(lastLosRateErr, 1e-6);
    }

    // V9b: out-of-cone (60deg, not 90deg -- keeps vClosing nonzero) but
    // within the memory/coast window. lastLosRate is preset to a known
    // nonzero value; confirms the commanded accel uses that FROZEN value
    // (not a freshly-recomputed one), lastLosRate is left unchanged,
    // hasLock stays true, and timeSinceLockLostS advances by exactly dt.
    {
        World coastWorld;
        SimContext simContext;
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{0.0, 0.0, 1000.0};
        Vec3 targetPos{17320.5, 25000.0, 10000.0};  // 60deg off the missile's own velocity direction

        EntityID missile = testrig::SpawnMover(coastWorld, missilePos, missileVel);
        EntityID target = coastWorld.CreateEntity();
        coastWorld.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        Vec3 presetLastLosRate{0.01, -0.02, 0.015};
        GuidanceComponent guidance = testrig::GiveAim120(coastWorld, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        guidance.hasLock = true;
        guidance.timeSinceLockLostS = 0.0;
        guidance.lastLosRate = presetLastLosRate;
        coastWorld.Components<GuidanceComponent>().Add(missile, guidance);

        Vec3 losUnitNow = Normalize(Sub(targetPos, missilePos));
        double vClosing = Dot(losUnitNow, Sub(missileVel, Vec3{}));
        Vec3 expectedAccelCmd = AcrossAxisKeepingMagnitude(Scale(presetLastLosRate, 4.0 * vClosing), missileVel);
        double missileSpeed = Length(missileVel);
        Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(missilePos.y());
        double mach = missileSpeed / atmosphere.speedOfSoundFtPerSec;
        double cd = testrig::Aim120()->aero.ZeroLiftCd(mach);
        double dynamicPressure = 0.5 * atmosphere.densitySlugPerFt3 * missileSpeed * missileSpeed;
        // launchSimTime=-100 -> burn long since exhausted -> current mass
        // is exactly dryMassSlug.
        double dragAccelMag = (dynamicPressure * cd * guidance.referenceAreaFt2) / guidance.dryMassSlug;
        Vec3 dragAccel = Scale(missileVel, -(dragAccelMag / missileSpeed));
        Vec3 expectedAccel = Add(expectedAccelCmd, dragAccel);  // launchSimTime=-100 -> no thrust term

        GuidanceSystem::Update(coastWorld, simContext, kDt, 0.0);
        const GuidanceComponent& after = coastWorld.Components<GuidanceComponent>().Get(missile);
        Vec3 observedAccel = coastWorld.Components<KinematicsComponent>().Get(missile).accel;

        double accelErr = Length(Sub(observedAccel, expectedAccel));
        double lastLosRateUnchanged = Length(Sub(after.lastLosRate, presetLastLosRate));
        EXPECT_LT(accelErr, 1e-6);
        EXPECT_TRUE(after.hasLock);
        EXPECT_LT(lastLosRateUnchanged, 1e-12);
        EXPECT_LT(std::fabs(after.timeSinceLockLostS - kDt), 1e-9);
    }

    // V9c: out-of-cone, coast window already expired (timeSinceLockLostS
    // preset to 2.0s, past the AIM-120's 1.5s). Confirms the PN component
    // is exactly zero, hasLock flips to false, and exactly one
    // SEEKER_LOST event is logged.
    {
        World expiredWorld;
        SimContext simContext;
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{0.0, 0.0, 1000.0};
        Vec3 targetPos{17320.5, 25000.0, 10000.0};

        EntityID missile = testrig::SpawnMover(expiredWorld, missilePos, missileVel);
        EntityID target = expiredWorld.CreateEntity();
        expiredWorld.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::GiveAim120(expiredWorld, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        guidance.hasLock = true;
        guidance.timeSinceLockLostS = 2.0;  // past the AIM-120's 1.5s coast window
        guidance.lastLosRate = Vec3{0.01, -0.02, 0.015};
        expiredWorld.Components<GuidanceComponent>().Add(missile, guidance);

        double missileSpeed = Length(missileVel);
        Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(missilePos.y());
        double mach = missileSpeed / atmosphere.speedOfSoundFtPerSec;
        double cd = testrig::Aim120()->aero.ZeroLiftCd(mach);
        double dynamicPressure = 0.5 * atmosphere.densitySlugPerFt3 * missileSpeed * missileSpeed;
        double dragAccelMag = (dynamicPressure * cd * guidance.referenceAreaFt2) / guidance.dryMassSlug;
        Vec3 expectedAccel = Scale(missileVel, -(dragAccelMag / missileSpeed));  // PN term zeroed, no thrust

        GuidanceSystem::Update(expiredWorld, simContext, kDt, 0.0);
        const GuidanceComponent& after = expiredWorld.Components<GuidanceComponent>().Get(missile);
        Vec3 observedAccel = expiredWorld.Components<KinematicsComponent>().Get(missile).accel;

        int seekerLostCount = 0;
        for (const SimEvent& event : simContext.eventLog.Events()) {
            if (event.type == "SEEKER_LOST") ++seekerLostCount;
        }

        double accelErr = Length(Sub(observedAccel, expectedAccel));
        EXPECT_LT(accelErr, 1e-6);
        EXPECT_FALSE(after.hasLock);
        EXPECT_EQ(seekerLostCount, 1);
    }
}

namespace {

// The F-16's radar at the origin, nose along +x (east), with power and
// thresholds so extreme that only a gate can block a detection, and a frame
// every 0.1 s. Shared by V10, V12 and V23, which test gates (blind range,
// horizon, notch) that sit inline in AirborneRadarSystem::Update.
EntityID AddGateTestRadar(World& world) {
    EntityID owner = world.CreateEntity();
    world.Components<Transform>().Add(owner, Transform{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 90.0}});
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*testrig::F16());
    radar.frameTimeS = 0.1;
    radar.averagePowerW = 1e9;
    radar.peakPowerW = 1e9;
    radar.antennaGain = 1e5;
    radar.frequencyHz = 1e10;
    radar.timeOnTargetS = 1.0;
    radar.searchThresholdDb = -100.0;  // trivially cleared at these ranges if not gated
    world.Components<AirborneRadarComponent>().Add(owner, radar);
    return owner;
}

// Detections in the last frames (the 5-frame window) for `target`, or -1
// if no track.
int SearchHits(const SimContext& simContext, EntityID target) {
    for (const AirborneRadarTrack& track : simContext.airborneRadarTracks) {
        if (track.target == target) return HitWindow::Count(track.frameHits);
    }
    return -1;
}

}  // namespace

// [V10] Minimum radar range / blind range, through a real
// AirborneRadarSystem::Update pipeline (the gate is inline in Update(), not
// a separately exposed function).
TEST(PhysicsValidation, MinRadarRange) {
    World world;
    SimContext simContext;
    constexpr double kFeetPerMeter = 3.280839895;
    AddGateTestRadar(world);

    // 250m -- inside the F-16's 500m blind range.
    EntityID targetInside = world.CreateEntity();
    double insideRangeFt = 250.0 * kFeetPerMeter;
    world.Components<Transform>().Add(targetInside, Transform{Vec3{insideRangeFt, 50.0, 0.0}, Vec3{}});
    world.Components<RadarCrossSection>().Add(targetInside, RadarCrossSection{5.0, FluctuationModel::STEADY});
    world.Components<Velocity>().Add(targetInside, Velocity{Vec3{100.0, 0.0, 0.0}});  // ~30.5 m/s radial, well above notch

    // 600m -- just outside the F-16's 500m blind range.
    EntityID targetOutside = world.CreateEntity();
    double outsideRangeFt = 600.0 * kFeetPerMeter;
    world.Components<Transform>().Add(targetOutside, Transform{Vec3{outsideRangeFt, 50.0, 0.0}, Vec3{}});
    world.Components<RadarCrossSection>().Add(targetOutside, RadarCrossSection{5.0, FluctuationModel::STEADY});
    world.Components<Velocity>().Add(targetOutside, Velocity{Vec3{100.0, 0.0, 0.0}});

    for (int i = 1; i <= 10; ++i) {
        AirborneRadarSystem::Update(world, simContext, i * 0.1);
    }

    EXPECT_EQ(SearchHits(simContext, targetInside), 0);
    EXPECT_GE(SearchHits(simContext, targetOutside), 5);  // the whole 5-frame window, even if float spacing skips a frame
}

// [V11] Minimum arming range. Two InterceptSystem::Update pipelines, both
// with the missile already well inside kill_radius_m -- the only
// difference is how far it's flown since launch.
TEST(PhysicsValidation, ArmingRange) {
    ScenarioData scenario;
    // killRadiusFt deliberately huge relative to the 5ft miss distance,
    // with a tight falloffWidthFt, so Pk is computationally
    // indistinguishable from exactly 1.0 -- the "armed" case's hit is
    // deterministic, not just usually true.
    scenario.killRadiusFt = 2000.0;
    scenario.falloffWidthFt = 5.0;
    scenario.maxRangeFt = 100000.0;  // avoid the unrelated exceeded-max-range branch firing in this test

    // V11a: unarmed -- launched from essentially the current position
    // (1ft of travel), well under the AIM-120's ~164ft armingRangeFt.
    {
        World world;
        SimContext simContext;
        Vec3 targetPos{1000.0, 5000.0, 0.0};
        // 5.5ft past the box surface -- InterceptSystem measures distance
        // to the target's bounding-box surface, not its center point.
        Vec3 missilePos{1000.0 + F16Box().halfSpanFt + 5.5, 5000.0, 0.0};

        EntityID missile = world.CreateEntity();
        world.Components<Transform>().Add(missile, Transform{missilePos, Vec3{}});
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});
        testrig::GiveF16Airframe(world, target);

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.active = true;
        guidance.launchSimTime = 0.0;
        guidance.prevRangeFt = 5.0;
        guidance.launchPos = Vec3{missilePos.x() - 1.0, 5000.0, 0.0};  // 1ft of travel -- unarmed
        world.Components<GuidanceComponent>().Add(missile, guidance);

        RunCpaConfirmationTicks(world, simContext, scenario, missile);

        bool stillActive = world.Components<GuidanceComponent>().Get(missile).active;
        bool destroyed = world.Components<DestroyedComponent>().Has(missile);
        int interceptEvents = 0;
        for (const SimEvent& event : simContext.eventLog.Events()) {
            if (event.type == "INTERCEPT") ++interceptEvents;
        }
        // An unarmed pass at closest approach still ends active guidance
        // (InterceptSystem.cpp: MISS always ends active guidance).
        EXPECT_FALSE(stillActive);
        EXPECT_FALSE(destroyed);
        EXPECT_EQ(interceptEvents, 0);
    }

    // V11b: armed -- identical closest approach and current range, but
    // launched from 2000ft away, comfortably past armingRangeFt.
    {
        World world;
        SimContext simContext;
        Vec3 targetPos{1000.0, 5000.0, 0.0};
        Vec3 missilePos{1000.0 + F16Box().halfSpanFt + 5.5, 5000.0, 0.0};

        EntityID missile = world.CreateEntity();
        world.Components<Transform>().Add(missile, Transform{missilePos, Vec3{}});
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});
        testrig::GiveF16Airframe(world, target);

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.active = true;
        guidance.launchSimTime = 0.0;
        guidance.prevRangeFt = 5.0;
        guidance.launchPos = Vec3{missilePos.x() - 2000.0, 5000.0, 0.0};  // 2000ft of travel -- armed
        world.Components<GuidanceComponent>().Add(missile, guidance);

        RunCpaConfirmationTicks(world, simContext, scenario, missile);

        bool destroyed = world.Components<DestroyedComponent>().Has(missile);
        int interceptEvents = 0;
        for (const SimEvent& event : simContext.eventLog.Events()) {
            if (event.type == "INTERCEPT") ++interceptEvents;
        }
        EXPECT_TRUE(destroyed);
        EXPECT_EQ(interceptEvents, 1);
    }
}

// [V12] Radar horizon / Earth curvature. V12a is the pure formula;
// V12b/c run a real AirborneRadarSystem::Update pipeline (the gate is
// inline in Update()) with a target below vs. above the horizon threshold.
TEST(PhysicsValidation, RadarHorizon) {
    constexpr double kEarthRadiusM = 6371000.0;
    constexpr double kEffectiveEarthRadiusM = 4.0 / 3.0 * kEarthRadiusM;  // 4/3-Earth atmospheric refraction model
    constexpr double kFeetPerMeter = 3.280839895;

    // V12a: pure formula check.
    constexpr double kRadarAltM = 0.0, kTargetAltM = 10000.0;
    double observed = RadarEquation::HorizonRangeM(kRadarAltM, kTargetAltM);
    double expected =
        std::sqrt(2.0 * kEffectiveEarthRadiusM * kRadarAltM) + std::sqrt(2.0 * kEffectiveEarthRadiusM * kTargetAltM);
    EXPECT_LT(std::fabs(observed - expected), 1e-6);

    // V12b/V12c: same range, target altitude below vs. above the horizon
    // threshold at that range (~589m at 100km).
    constexpr double kRangeM = 100000.0;
    constexpr double kLowAltM = 500.0;    // below horizon at 100km
    constexpr double kHighAltM = 1500.0;  // above horizon at 100km

    auto runOne = [&](double targetAltM) -> int {
        World world;
        SimContext simContext;
        AddGateTestRadar(world);

        EntityID target = world.CreateEntity();
        double targetXFt = kRangeM * kFeetPerMeter;
        double targetYFt = targetAltM * kFeetPerMeter;
        world.Components<Transform>().Add(target, Transform{Vec3{targetXFt, targetYFt, 0.0}, Vec3{}});
        world.Components<RadarCrossSection>().Add(target, RadarCrossSection{5.0, FluctuationModel::STEADY});
        world.Components<Velocity>().Add(target, Velocity{Vec3{100.0, 0.0, 0.0}});

        for (int i = 1; i <= 10; ++i) {
            AirborneRadarSystem::Update(world, simContext, i * 0.1);
        }
        return SearchHits(simContext, target);
    };

    EXPECT_EQ(runOne(kLowAltM), 0);
    EXPECT_GE(runOne(kHighAltM), 5);
}

// [V23] The Doppler notch needs ground in the beam. V23a is the pure
// geometry; V23b-d run a real AirborneRadarSystem::Update pipeline with a
// beaming target (crossing the line of sight, zero radial speed) level with
// a radar at 25 000 ft (the beam's lower edge reaches below the horizon:
// notched), 4 deg above it (sky behind the whole beam: seen) and closing at
// that same elevation (not beaming: seen).
TEST(PhysicsValidation, NotchNeedsGroundInTheBeam) {
    constexpr double kFeetPerMeter = 3.280839895;
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    auto losAt = [&](double elevationDeg) {
        return Vec3{std::cos(elevationDeg * kDegToRad), std::sin(elevationDeg * kDegToRad), 0.0};
    };

    // V23a: the F-16 type's beam, 1.2 lambda / 0.488 m at 9.86 GHz = 4.28
    // deg. At sea level any beam edge below level is on the ground; at
    // 25 000 ft the 4/3-Earth horizon dips 2.43 deg.
    double beamRad = RadarTrackFilter::BeamwidthRad(WavelengthM(9.86e9), 0.488, 1.2);
    EXPECT_NEAR(beamRad / kDegToRad, 4.283787, 1e-5);
    EXPECT_TRUE(RadarEquation::BeamOnGround(0.0, losAt(4.2), beamRad));
    EXPECT_FALSE(RadarEquation::BeamOnGround(0.0, losAt(4.4), beamRad));
    constexpr double kAltM = 25000.0 / kFeetPerMeter;
    EXPECT_TRUE(RadarEquation::BeamOnGround(kAltM, losAt(1.8), beamRad));
    EXPECT_FALSE(RadarEquation::BeamOnGround(kAltM, losAt(1.9), beamRad));
    EXPECT_TRUE(RadarEquation::BeamOnGround(kAltM, losAt(-30.0), beamRad));

    constexpr double kRangeM = 20000.0;
    auto runOne = [&](double elevationDeg, const Vec3& targetVelFtPerSec) -> int {
        World world;
        SimContext simContext;
        EntityID owner = AddGateTestRadar(world);
        world.Components<Transform>().Get(owner).pos.y() = kAltM * kFeetPerMeter;
        // The F-16's own beam, V23a's: the rig's radar keeps its type's
        // antenna, beamwidth factor and notch.
        world.Components<AirborneRadarComponent>().Get(owner).frequencyHz = 9.86e9;

        EntityID target = world.CreateEntity();
        Vec3 offsetFt = losAt(elevationDeg) * (kRangeM * kFeetPerMeter);
        world.Components<Transform>().Add(target, Transform{Vec3{offsetFt.x(), kAltM * kFeetPerMeter + offsetFt.y(),
                                                                 offsetFt.z()}, Vec3{}});
        world.Components<RadarCrossSection>().Add(target, RadarCrossSection{5.0, FluctuationModel::STEADY});
        world.Components<Velocity>().Add(target, Velocity{targetVelFtPerSec});

        for (int i = 1; i <= 10; ++i) {
            AirborneRadarSystem::Update(world, simContext, i * 0.1);
        }
        return SearchHits(simContext, target);
    };

    const Vec3 beaming{0.0, 0.0, 800.0};  // across the line of sight
    const Vec3 closing{-800.0, 0.0, 0.0};
    EXPECT_EQ(runOne(0.0, beaming), 0);
    EXPECT_GE(runOne(4.0, beaming), 5);
    EXPECT_GE(runOne(4.0, closing), 5);
}

// [V24] The radar's track is a measurement, not truth, and the datalink
// relays it. The F-16 type's radar tracks a steady 5 m2 target closing at
// 150 m/s and crossing at 200 m/s, level at 25 000 ft, for 20 one-second frames: the track is valid and
// close but never exactly on the target, and its cross-range error is
// larger at 40 nmi (low SNR, angle error x range) than at 15 nmi. A missile
// datalinked from that radar then holds exactly the radar's track,
// carried forward to now.
TEST(PhysicsValidation, RadarTrackIsMeasuredAndRelayed) {
    constexpr double kFeetPerMeter = 3.280839895;
    constexpr double kAltFt = 25000.0;
    // Closing as well as crossing, so it never sits in the Doppler notch.
    const Vec3 crossing{-150.0 * kFeetPerMeter, 0.0, 200.0 * kFeetPerMeter};

    struct Run {
        World world;
        SimContext simContext;
        EntityID owner = 0;
        EntityID target = 0;
        Vec3 start;
    };
    auto setUp = [&](Run& run, double rangeNmi, uint64_t seed) {
        run.simContext.rng.Seed(seed);
        run.owner = run.world.CreateEntity();
        run.world.Components<Transform>().Add(run.owner, Transform{Vec3{0.0, kAltFt, 0.0}, Vec3{0.0, 0.0, 90.0}});
        // A 1 s frame rather than the type's 4.4 s, so the 20 s run gives
        // the filter 20 looks: this checks the measurement, not the frame.
        AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*testrig::F16());
        radar.frameTimeS = 1.0;
        run.world.Components<AirborneRadarComponent>().Add(run.owner, radar);
        run.target = run.world.CreateEntity();
        run.start = Vec3{rangeNmi * 1852.0 * kFeetPerMeter, kAltFt, 0.0};
        run.world.Components<Transform>().Add(run.target, Transform{run.start, Vec3{}});
        run.world.Components<RadarCrossSection>().Add(run.target, RadarCrossSection{5.0, FluctuationModel::STEADY});
        run.world.Components<Velocity>().Add(run.target, Velocity{crossing});
    };
    // Frames at t = 0..19 s with the target moved along before each.
    auto fly = [&](Run& run) {
        for (int i = 0; i < 20; ++i) {
            double t = static_cast<double>(i);
            run.world.Components<Transform>().Get(run.target).pos = run.start + crossing * t;
            AirborneRadarSystem::Update(run.world, run.simContext, t);
        }
    };
    auto crossRangeRmsFt = [&](double rangeNmi) {
        double sumSq = 0.0;
        constexpr int kSeeds = 12;
        for (int seed = 1; seed <= kSeeds; ++seed) {
            Run run;
            setUp(run, rangeNmi, static_cast<uint64_t>(seed));
            fly(run);
            const AirborneRadarTrack* track = run.simContext.airborneRadarTracks.Find(run.owner, run.target);
            EXPECT_TRUE(track != nullptr && track->estimate.valid);
            if (track == nullptr || !track->estimate.valid) return -1.0;
            Vec3 truth = run.world.Components<Transform>().Get(run.target).pos;
            Vec3 err = RadarTrackFilter::Position(track->estimate) - truth;
            EXPECT_GT(err.norm(), 0.0);  // a measurement, never truth
            sumSq += err.z() * err.z();                 // across the line of sight, horizontal
        }
        return std::sqrt(sumSq / kSeeds);
    };
    double nearFt = crossRangeRmsFt(15.0);
    double farFt = crossRangeRmsFt(40.0);
    EXPECT_GT(nearFt, 0.0);
    EXPECT_GT(farFt, 3.0 * nearFt);

    // The datalink relays the radar's track, extrapolated from its sweep.
    Run run;
    setUp(run, 20.0, 3);
    fly(run);
    EntityID missile = run.world.CreateEntity();
    GuidanceComponent guidance;
    guidance.launched = true;
    guidance.active = true;
    guidance.launcher = run.owner;
    guidance.target = run.target;
    run.world.Components<GuidanceComponent>().Add(missile, guidance);
    run.world.Components<DatalinkTrackComponent>().Add(missile, DatalinkTrackComponent{});
    constexpr double kNow = 19.5;
    DatalinkSystem::Update(run.world, run.simContext, 1.0 / 120.0, kNow);
    const AirborneRadarTrack* track = run.simContext.airborneRadarTracks.Find(run.owner, run.target);
    ASSERT_TRUE(track != nullptr && track->state == RadarTrackState::CONFIRMED);
    const DatalinkTrackComponent& link = run.world.Components<DatalinkTrackComponent>().Get(missile);
    ASSERT_TRUE(link.hasTrack);
    Vec3 relayed = RadarTrackFilter::Position(track->estimate) +
                   RadarTrackFilter::Velocity(track->estimate) * (kNow - track->estimate.time);
    EXPECT_LT((link.estimatedPos - relayed).norm(), 1e-9);
    EXPECT_LT((link.estimatedVel - RadarTrackFilter::Velocity(track->estimate)).norm(), 1e-9);
}

// [V8] Aircraft G-limit V-n envelope: low speed is aerodynamically
// limited, high speed is structurally limited, negative side mirrors the
// same shape.
TEST(PhysicsValidation, GLimitEnvelope) {
    // The F-16's envelope (clMax 1.65, 300 ft2, +9 / -3 G) at a fixed
    // 25 000 lb.
    GLimitComponent g = AircraftEntity::MakeGLimit(*testrig::F16());
    g.weightLbf = 25000.0;

    // V8a: low speed.
    constexpr double kLowSpeedFtPerSec = 300.0;  // ~178 KTAS -- well below corner speed
    constexpr double kAltFt = 25000.0;
    double rho = Atmosphere::Lookup(kAltFt).densitySlugPerFt3;
    // Lift limit with the FCS's G/AoA blend, CL(n) = clMax - L (n - 1):
    // n = CL(n) Q, Q = q S / W, solves to (clMax + L) Q / (1 + L Q).
    auto liftLimitG = [&](double speedFtPerSec) {
        double q = 0.5 * rho * speedFtPerSec * speedFtPerSec * g.wingAreaFt2 / g.weightLbf;
        return (g.clMax + g.clMaxLossPerG) * q / (1.0 + g.clMaxLossPerG * q);
    };
    double expectedAeroLimit = liftLimitG(kLowSpeedFtPerSec);
    double observedLow = FlightEnvelope::GetMaxAvailableG(kLowSpeedFtPerSec, kAltFt, g);
    EXPECT_LT(expectedAeroLimit, g.structuralGLimitPos);  // genuinely aero-limited, not degenerately equal
    EXPECT_LT(std::fabs(observedLow - expectedAeroLimit), 1e-9);

    // V8b: high speed -- structural cap should dominate instead.
    constexpr double kHighSpeedFtPerSec = 1800.0;  // ~1067 KTAS -- well above corner speed
    double aeroLimitAtHighSpeed = liftLimitG(kHighSpeedFtPerSec);
    double observedHigh = FlightEnvelope::GetMaxAvailableG(kHighSpeedFtPerSec, kAltFt, g);
    EXPECT_GT(aeroLimitAtHighSpeed, g.structuralGLimitPos);  // raw aero limit genuinely exceeds the structural cap
    EXPECT_LT(std::fabs(observedHigh - g.structuralGLimitPos), 1e-9);

    // V8c: negative envelope mirrors the same shape.
    double observedNeg = FlightEnvelope::GetMaxAvailableNegativeG(kHighSpeedFtPerSec, kAltFt, g);
    EXPECT_LT(std::fabs(observedNeg - g.structuralGLimitNeg), 1e-9);
}

// [V7] Three-stage thrust profile energy story: sharp rise during boost,
// shallower rise during sustain, monotonic decay during coast. Target
// placed directly ahead (colinear pursuit) so accelCmd is exactly zero
// throughout, isolating thrust+drag as the only things moving speed.
TEST(PhysicsValidation, ThrustProfile) {
    World world;
    SimContext simContext;

    Vec3 missilePos{0.0, 25000.0, 0.0};
    Vec3 missileVel{900.0, 0.0, 0.0};
    Vec3 targetPos{2000000.0, 25000.0, 0.0};  // far down the missile's own velocity axis -- never reached

    EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);

    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

    GuidanceComponent guidance = testrig::GiveAim120(world, missile);
    guidance.target = target;
    guidance.navGain = 4.0;
    guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
    guidance.active = true;
    guidance.launched = true;
    guidance.launchSimTime = 0.0;
    world.Components<GuidanceComponent>().Add(missile, guidance);

    constexpr double kDt = 0.05;
    double simTime = 0.0;
    double speedAtLaunch = world.Components<KinematicsComponent>().Get(missile).speed;
    double speedAtBoostEnd = 0.0, speedAtSustainEnd = 0.0, speedAtCoastSample = 0.0;
    constexpr int kBoostEndTick = 60;     // 3.0s
    constexpr int kSustainEndTick = 160;  // 8.0s
    constexpr int kCoastSampleTick = 260;  // 13.0s -- 5s into coast
    for (int tick = 1; tick <= kCoastSampleTick; ++tick) {
        GuidanceSystem::Update(world, simContext, kDt, simTime);
        IntegrationSystem::Update(world, kDt);
        simTime += kDt;
        double speedNow = world.Components<KinematicsComponent>().Get(missile).speed;
        if (tick == kBoostEndTick) speedAtBoostEnd = speedNow;
        if (tick == kSustainEndTick) speedAtSustainEnd = speedNow;
        if (tick == kCoastSampleTick) speedAtCoastSample = speedNow;
    }

    double boostRateFtPerSec2 = (speedAtBoostEnd - speedAtLaunch) / (kBoostEndTick * kDt);
    double sustainRateFtPerSec2 = (speedAtSustainEnd - speedAtBoostEnd) / ((kSustainEndTick - kBoostEndTick) * kDt);

    EXPECT_GT(speedAtBoostEnd, speedAtLaunch);
    EXPECT_GE(speedAtSustainEnd, speedAtBoostEnd - 1.0);  // small tolerance, not a hard plateau
    EXPECT_LT(sustainRateFtPerSec2, boostRateFtPerSec2);
    EXPECT_LT(speedAtCoastSample, speedAtSustainEnd);
}

// [V16] Missile mass depletion during burn.
TEST(PhysicsValidation, MassDepletion) {
    // V16a: acceleration magnitude visibly rises within the boost phase
    // under CONSTANT thrust force, purely because mass is depleting
    // (F=ma). Colinear (zero accelCmd) setup isolates thrust+drag.
    auto sampleAccelMag = [](double timeSinceLaunch) -> std::pair<double, double> {
        World world;
        SimContext simContext;
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{900.0, 0.0, 0.0};
        Vec3 targetPos{2000000.0, 25000.0, 0.0};

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -timeSinceLaunch;
        world.Components<GuidanceComponent>().Add(missile, guidance);

        GuidanceSystem::Update(world, simContext, 0.01, 0.0);
        Vec3 observedAccel = world.Components<KinematicsComponent>().Get(missile).accel;

        double burnFraction = std::clamp(timeSinceLaunch / (guidance.boostDurationS + guidance.sustainDurationS),
                                          0.0, 1.0);
        double expectedMassSlug = guidance.dryMassSlug + guidance.propellantMassSlug * (1.0 - burnFraction);
        double expectedThrustAccelMag = guidance.boostThrustLbf / expectedMassSlug;

        Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(missilePos.y());
        double missileSpeed = Length(missileVel);
        double mach = missileSpeed / atmosphere.speedOfSoundFtPerSec;
        double cd = testrig::Aim120()->aero.ZeroLiftCd(mach);  // sampled inside the boost
        double dynamicPressure = 0.5 * atmosphere.densitySlugPerFt3 * missileSpeed * missileSpeed;
        double dragAccelMag = (dynamicPressure * cd * guidance.referenceAreaFt2) / expectedMassSlug;
        double expectedAccelMag = expectedThrustAccelMag - dragAccelMag;  // colinear: thrust and drag both along +x

        return {Length(observedAccel), expectedAccelMag};
    };

    auto [earlyObserved, earlyExpected] = sampleAccelMag(0.5);  // early boost -- mass close to full
    auto [lateObserved, lateExpected] = sampleAccelMag(2.5);    // late boost, still within the 3s window

    EXPECT_LT(std::fabs(earlyObserved - earlyExpected) / earlyExpected, 1e-6);
    EXPECT_LT(std::fabs(lateObserved - lateExpected) / lateExpected, 1e-6);
    EXPECT_GT(lateObserved, earlyObserved);

    // V16b: current mass reaches exactly dryMassSlug at the boost+sustain
    // boundary, not before or after.
    constexpr double kBurnDurationS = 3.0 + 5.0;  // the AIM-120's boost+sustain
    World massWorld;
    GuidanceComponent massGuidance = testrig::GiveAim120(massWorld, testrig::SpawnMover(massWorld, Vec3{}, Vec3{}));
    EXPECT_GT(GuidanceSystem::CurrentMassSlug(massGuidance, kBurnDurationS - 0.01), massGuidance.dryMassSlug);
    EXPECT_DOUBLE_EQ(GuidanceSystem::CurrentMassSlug(massGuidance, kBurnDurationS), massGuidance.dryMassSlug);
    EXPECT_DOUBLE_EQ(GuidanceSystem::CurrentMassSlug(massGuidance, kBurnDurationS + 0.5), massGuidance.dryMassSlug);

    // V16c: a shot that resolved as a miss mid-burn keeps burning: on the
    // unguided branch the motor still pushes along the flight
    // path until burnout, then only drag and gravity act.
    auto unguidedAlongPathAccel = [](double timeSinceLaunch) -> double {
        World world;
        SimContext simContext;
        EntityID missile = testrig::SpawnMover(world, Vec3{0.0, 25000.0, 0.0}, Vec3{900.0, 0.0, 0.0});
        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.active = false;  // resolved: the unguided branch -- see GuidanceSystem.cpp
        guidance.launchSimTime = -timeSinceLaunch;
        world.Components<GuidanceComponent>().Add(missile, guidance);
        GuidanceSystem::Update(world, simContext, 0.01, 0.0);
        return world.Components<KinematicsComponent>().Get(missile).accel.x();  // along the flight path
    };
    const double sustainAccel = massGuidance.sustainThrustLbf /
                                GuidanceSystem::CurrentMassSlug(massGuidance, kBurnDurationS - 0.01);
    double burning = unguidedAlongPathAccel(kBurnDurationS - 0.01);
    double burntOut = unguidedAlongPathAccel(kBurnDurationS + 0.5);
    EXPECT_LT(burntOut, 0.0);  // drag only
    EXPECT_NEAR(burning - burntOut, sustainAccel, 0.01 * sustainAccel);  // the motor, on top of nearly equal drag
}

// [V19] SARH vs. ARH seeker modes. A missile flying dead-on-boresight
// (in-cone the whole time) while the launcher's own AirborneRadarTrack on
// the target drops from confirmed to tentative partway through.
TEST(PhysicsValidation, SarhVsArh) {
    auto runScenario = [](SeekerMode mode) -> bool {
        World world;
        SimContext simContext;
        EntityID launcher = world.CreateEntity();  // no components needed -- only its EntityID matters here
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{1000.0, 0.0, 0.0};
        Vec3 targetPos{2000000.0, 25000.0, 0.0};  // directly ahead -- exactly 0deg boresight, always in-cone

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;  // isolates from thrust -- not under test here
        guidance.seekerMode = mode;
        guidance.launcher = launcher;
        world.Components<GuidanceComponent>().Add(missile, guidance);

        AirborneRadarTrack track;
        track.owner = launcher;
        track.target = target;
        track.state = RadarTrackState::CONFIRMED;
        simContext.airborneRadarTracks.push_back(track);

        constexpr double kDt = 0.1;
        double simTime = 0.0;
        for (int tick = 0; tick < 30; ++tick) {
            // Track drops to tentative a third of the way through -- a
            // notch-style illumination loss for a SARH missile.
            if (tick == 10) simContext.airborneRadarTracks[0].state = RadarTrackState::TENTATIVE;
            GuidanceSystem::Update(world, simContext, kDt, simTime);
            simTime += kDt;
        }

        return world.Components<GuidanceComponent>().Get(missile).hasLock;
    };

    bool sarhHasLockAtEnd = runScenario(SeekerMode::SARH);
    bool arhHasLockAtEnd = runScenario(SeekerMode::ARH);

    // 20 ticks (tick 10 to tick 29) at dt=0.1s = 2.0s elapsed since the
    // track dropped -- past the AIM-120's 1.5s memoryCoastTimeS.
    EXPECT_FALSE(sarhHasLockAtEnd);
    EXPECT_TRUE(arhHasLockAtEnd);
}

// The seeker gimbal limits only a seeker that is measuring. A MIDCOURSE_ARH
// missile whose own seeker is not yet active steers on the launcher's track,
// so a target 60 deg off its nose (outside the AIM-120's 45 deg cone) stays
// guided on past the memory coast; an ARH missile, measuring with that same
// seeker, loses it.
TEST(PhysicsValidation, MidcourseGuidesOutsideTheSeekerGimbal) {
    auto runScenario = [](SeekerMode mode, int& seekerLostEvents) {
        World world;
        SimContext simContext;
        EntityID launcher = world.CreateEntity();
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{1000.0, 0.0, 0.0};
        // 60 deg off the missile's nose and far beyond its seeker's reach,
        // so the seeker never acquires.
        constexpr double kRangeFt = 2000000.0;
        Vec3 targetPos{kRangeFt * 0.5, 25000.0, kRangeFt * std::sqrt(3.0) / 2.0};

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        EXPECT_LT(guidance.seekerFovHalfAngleDeg, 60.0);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        guidance.seekerMode = mode;
        guidance.launcher = launcher;
        world.Components<GuidanceComponent>().Add(missile, guidance);

        AirborneRadarTrack track;
        track.owner = launcher;
        track.target = target;
        track.state = RadarTrackState::CONFIRMED;
        simContext.airborneRadarTracks.push_back(track);

        constexpr double kDt = 0.1;
        double simTime = 0.0;
        for (int tick = 0; tick < 30; ++tick) {  // 3 s, twice the 1.5 s coast
            GuidanceSystem::Update(world, simContext, kDt, simTime);
            simTime += kDt;
        }
        seekerLostEvents = 0;
        for (const SimEvent& event : simContext.eventLog.Events()) seekerLostEvents += event.type == "SEEKER_LOST";
        EXPECT_FALSE(world.Components<GuidanceComponent>().Get(missile).seekerActive);
        return world.Components<GuidanceComponent>().Get(missile).hasLock;
    };

    int midcourseLost = 0, arhLost = 0;
    EXPECT_TRUE(runScenario(SeekerMode::MIDCOURSE_ARH, midcourseLost));
    EXPECT_EQ(midcourseLost, 0);
    EXPECT_FALSE(runScenario(SeekerMode::ARH, arhLost));
    EXPECT_EQ(arhLost, 1);
}

// A MIDCOURSE_ARH missile whose own seeker is active and loses the target
// past the memory coast hands back to datalink midcourse while the launcher
// still tracks the target, and must acquire again; with the launcher's
// track down to tentative it goes ballistic as before.
TEST(PhysicsValidation, ActiveSeekerFallsBackOnTheDatalink) {
    auto runScenario = [](RadarTrackState launcherState, bool& seekerActiveAtEnd, std::string& lostReason) {
        World world;
        SimContext simContext;
        EntityID launcher = world.CreateEntity();
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{1000.0, 0.0, 0.0};
        // 60 deg off the nose, outside the 45 deg gimbal, and too far for
        // the seeker to re-acquire.
        constexpr double kRangeFt = 2000000.0;
        Vec3 targetPos{kRangeFt * 0.5, 25000.0, kRangeFt * std::sqrt(3.0) / 2.0};

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        guidance.seekerMode = SeekerMode::MIDCOURSE_ARH;
        guidance.seekerActive = true;  // pitbull, then the target leaves the cone
        guidance.hasLock = true;
        guidance.launcher = launcher;
        world.Components<GuidanceComponent>().Add(missile, guidance);

        AirborneRadarTrack track;
        track.owner = launcher;
        track.target = target;
        track.state = launcherState;
        track.estimate.valid = true;
        simContext.airborneRadarTracks.push_back(track);

        constexpr double kDt = 0.1;
        double simTime = 0.0;
        for (int tick = 0; tick < 30; ++tick) {  // 3 s, twice the 1.5 s coast
            GuidanceSystem::Update(world, simContext, kDt, simTime);
            simTime += kDt;
        }
        int seekerLostEvents = 0;
        for (const SimEvent& event : simContext.eventLog.Events()) {
            if (event.type != "SEEKER_LOST") continue;
            ++seekerLostEvents;
            lostReason = event.data;
        }
        EXPECT_EQ(seekerLostEvents, 1);
        seekerActiveAtEnd = world.Components<GuidanceComponent>().Get(missile).seekerActive;
        return world.Components<GuidanceComponent>().Get(missile).hasLock;
    };

    bool seekerActive = true;
    std::string reason;
    EXPECT_TRUE(runScenario(RadarTrackState::CONFIRMED, seekerActive, reason));
    EXPECT_FALSE(seekerActive);
    EXPECT_NE(reason.find("fallback=datalink"), std::string::npos) << reason;

    EXPECT_FALSE(runScenario(RadarTrackState::TENTATIVE, seekerActive, reason));
    EXPECT_EQ(reason.find("fallback=datalink"), std::string::npos) << reason;
}

// A MIDCOURSE_ARH missile whose launcher has dropped its
// track (datalink lost) before its own seeker goes active flies on to the
// last datalink estimate, extrapolated, rather than memory-coasting into a
// ballistic flight; one that never received an update still loses lock.
TEST(PhysicsValidation, MidcourseFliesOnToTheLastEstimateAfterDatalinkLoss) {
    auto runScenario = [](bool hadUpdate, int& seekerLostEvents) {
        World world;
        SimContext simContext;
        EntityID launcher = world.CreateEntity();
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{1000.0, 0.0, 0.0};
        // Ahead, and far beyond the seeker's reach so it never acquires.
        Vec3 targetPos{2000000.0, 25000.0, 100000.0};

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        guidance.seekerMode = SeekerMode::MIDCOURSE_ARH;
        guidance.hasLock = true;
        guidance.launcher = launcher;
        world.Components<GuidanceComponent>().Add(missile, guidance);
        if (hadUpdate) {
            DatalinkTrackComponent datalink;
            datalink.hasTrack = true;
            datalink.linkLost = true;
            datalink.estimatedPos = targetPos;
            world.Components<DatalinkTrackComponent>().Add(missile, datalink);
        }

        // The launcher's track has dropped back to tentative: no link.
        AirborneRadarTrack track;
        track.owner = launcher;
        track.target = target;
        track.state = RadarTrackState::TENTATIVE;
        simContext.airborneRadarTracks.push_back(track);

        constexpr double kDt = 0.1;
        double simTime = 0.0;
        for (int tick = 0; tick < 30; ++tick) {  // 3 s, twice the 1.5 s coast
            GuidanceSystem::Update(world, simContext, kDt, simTime);
            simTime += kDt;
        }
        seekerLostEvents = 0;
        for (const SimEvent& event : simContext.eventLog.Events()) seekerLostEvents += event.type == "SEEKER_LOST";
        EXPECT_FALSE(world.Components<GuidanceComponent>().Get(missile).seekerActive);
        return world.Components<GuidanceComponent>().Get(missile).hasLock;
    };

    int lostWithEstimate = 0, lostWithout = 0;
    EXPECT_TRUE(runScenario(true, lostWithEstimate));
    EXPECT_EQ(lostWithEstimate, 0);
    EXPECT_FALSE(runScenario(false, lostWithout));
    EXPECT_EQ(lostWithout, 1);
}

// [V25] The active seeker's Doppler notch: the aircraft radar's rule seen
// from the missile (AIM-120 type: 1.2 lambda / 0.15 m = 13.75 deg beam,
// 15 m/s). A measuring ARH seeker at 25 000 ft against a target 10 nmi
// ahead: beaming level with it (the wide beam reaches below the horizon)
// it rides the 1.5 s memory coast, then loses the target to the notch;
// beaming 20 deg above (sky behind the whole beam) or closing it holds.
// V25b: before pitbull a notched sweep is a miss, so a MIDCOURSE_ARH seeker
// never acquires a beaming target it acquires at once when it is closing.
TEST(PhysicsValidation, ActiveSeekerDopplerNotch) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    double beamRad = RadarTrackFilter::BeamwidthRad(WavelengthM(1.0e10), 0.15, 1.2);
    EXPECT_NEAR(beamRad / kDegToRad, 13.75, 0.01);

    constexpr double kRangeFt = 60761.0;  // 10 nmi
    const Vec3 kBeaming{0.0, 0.0, 900.0};   // across the line of sight
    const Vec3 kClosing{-900.0, 0.0, 0.0};  // straight down it
    auto targetAt = [&](double elevationDeg) {
        return Vec3{kRangeFt * std::cos(elevationDeg * kDegToRad), 25000.0 + kRangeFt * std::sin(elevationDeg * kDegToRad),
                    0.0};
    };

    // V25a: a measuring ARH seeker. Returns whether it still holds the
    // target after 3 s (twice the coast) and the SEEKER_LOST reason.
    auto runArh = [&](const Vec3& targetPos, const Vec3& targetVel, bool& lockAtCoastEnd, std::string& lostReason) {
        World world;
        SimContext simContext;
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{3000.0, 0.0, 0.0};
        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = testrig::SpawnMover(world, targetPos, targetVel);

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.seekerNotchVelocityThresholdMps = testrig::Aim120()->seeker.notchVelocityThresholdMps;
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        guidance.hasLock = true;
        world.Components<GuidanceComponent>().Add(missile, guidance);

        // Geometry frozen (no integration): only the lock state moves.
        constexpr double kDt = 0.1;
        double simTime = 0.0;
        lostReason.clear();
        for (int tick = 0; tick < 30; ++tick) {
            GuidanceSystem::Update(world, simContext, kDt, simTime);
            simTime += kDt;
            if (tick == 13) lockAtCoastEnd = world.Components<GuidanceComponent>().Get(missile).hasLock;
        }
        for (const SimEvent& event : simContext.eventLog.Events())
            if (event.type == "SEEKER_LOST") lostReason = event.data;
        return world.Components<GuidanceComponent>().Get(missile).hasLock;
    };

    bool lockAtCoastEnd = false;
    std::string reason;
    EXPECT_FALSE(runArh(targetAt(0.0), kBeaming, lockAtCoastEnd, reason));
    EXPECT_TRUE(lockAtCoastEnd);  // memory coast rides through the first 1.5 s
    EXPECT_EQ(reason, "doppler_notch");
    EXPECT_TRUE(runArh(targetAt(20.0), kBeaming, lockAtCoastEnd, reason));
    EXPECT_EQ(reason, "");
    EXPECT_TRUE(runArh(targetAt(0.0), kClosing, lockAtCoastEnd, reason));
    EXPECT_EQ(reason, "");

    // V25b: MIDCOURSE_ARH acquisition, 3 nmi out (one look all but certain),
    // the launcher's confirmed track illuminating it.
    auto acquires = [&](const Vec3& targetVel) {
        World world;
        SimContext simContext;
        EntityID launcher = world.CreateEntity();
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 targetPos{18228.0, 25000.0, 0.0};
        EntityID missile = testrig::SpawnMover(world, missilePos, Vec3{3000.0, 0.0, 0.0});
        EntityID target = testrig::SpawnMover(world, targetPos, targetVel);
        world.Components<RadarCrossSection>().Add(target, RadarCrossSection{5.0, FluctuationModel::STEADY});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.seekerNotchVelocityThresholdMps = testrig::Aim120()->seeker.notchVelocityThresholdMps;
        guidance.seekerMode = SeekerMode::MIDCOURSE_ARH;
        guidance.target = target;
        guidance.launcher = launcher;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        world.Components<GuidanceComponent>().Add(missile, guidance);

        AirborneRadarTrack track;
        track.owner = launcher;
        track.target = target;
        track.state = RadarTrackState::CONFIRMED;
        track.estimate.valid = true;
        simContext.airborneRadarTracks.push_back(track);

        constexpr double kDt = 0.1;
        double simTime = 0.0;
        for (int tick = 0; tick < 10; ++tick) {  // 10 sweeps
            GuidanceSystem::Update(world, simContext, kDt, simTime);
            simTime += kDt;
        }
        return world.Components<GuidanceComponent>().Get(missile).seekerActive;
    };
    EXPECT_FALSE(acquires(kBeaming));
    EXPECT_TRUE(acquires(kClosing));
}

// The launch turn: with the target 40 deg off the nose and almost no LOS
// rate, proportional navigation asks for next to nothing, so the autopilot
// instead turns toward the target at the airframe's whole ceiling until the
// target is inside the exit angle. Geometry is frozen (no integration), so
// the lagged lift settles on the command.
TEST(PhysicsValidation, LaunchTurnTurnsTowardAnOffBoresightTarget) {
    auto lateralAccel = [](double offBoresightDeg, bool launchTurn, bool& turnDone, int& gLimitEvents) {
        World world;
        SimContext simContext;
        Vec3 missilePos{0.0, 25000.0, 0.0};
        Vec3 missileVel{2000.0, 0.0, 0.0};
        constexpr double kRangeFt = 200000.0;
        double off = offBoresightDeg * Units::kDegToRad;
        Vec3 targetPos{kRangeFt * std::cos(off), 25000.0, kRangeFt * std::sin(off)};

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = Normalize(Sub(targetPos, missilePos));
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -100.0;
        guidance.launchTurnDone = !launchTurn;
        world.Components<GuidanceComponent>().Add(missile, guidance);

        constexpr double kDt = 0.1;
        double simTime = 0.0;
        for (int tick = 0; tick < 30; ++tick) {  // 3 s, many actuator time constants
            GuidanceSystem::Update(world, simContext, kDt, simTime);
            simTime += kDt;
        }
        gLimitEvents = 0;
        for (const SimEvent& event : simContext.eventLog.Events()) {
            gLimitEvents += event.type == "MISSILE_G_LIMIT_EXCEEDED";
        }
        turnDone = world.Components<GuidanceComponent>().Get(missile).launchTurnDone;
        return world.Components<KinematicsComponent>().Get(missile).accel.z();  // toward the target
    };

    const double exitDeg = testrig::Aim120()->autopilot.launchTurnExitDeg;
    ASSERT_GT(exitDeg, 0.0);
    ASSERT_LT(exitDeg, 40.0);

    bool turnDone = false;
    int gLimitEvents = 0;
    double turning = lateralAccel(40.0, true, turnDone, gLimitEvents);
    EXPECT_FALSE(turnDone);           // geometry is frozen: never inside the exit angle
    EXPECT_EQ(gLimitEvents, 0);       // asking for the ceiling is not an overdemand
    double pnOnly = lateralAccel(40.0, false, turnDone, gLimitEvents);
    EXPECT_GT(turning, 10.0 * Units::kGravityFtPerSec2);
    EXPECT_GT(turning, 5.0 * std::fabs(pnOnly));

    // Inside the exit angle the turn is over on the first tick.
    lateralAccel(exitDeg - 5.0, true, turnDone, gLimitEvents);
    EXPECT_TRUE(turnDone);
}

// [V18] Probabilistic warhead lethality: many seeded InterceptSystem::
// Update trials, identical geometry, only the RNG seed differing --
// confirms the empirical hit rate roughly matches the Pk sigmoid.
TEST(PhysicsValidation, ProbabilisticLethality) {
    ScenarioData scenario;
    scenario.killRadiusFt = 15.0;
    scenario.falloffWidthFt = 5.0;
    scenario.maxRangeFt = 100000.0;
    constexpr double kMissDistanceFt = 10.0;  // inside kill radius, but not by an overwhelming margin

    // The logistic rescaled to 1 at zero miss distance (REFERENCE.md 12.4).
    auto logistic = [&](double d) {
        return 1.0 / (1.0 + std::exp((d - scenario.killRadiusFt) / scenario.falloffWidthFt));
    };
    double expectedPk = logistic(kMissDistanceFt) / logistic(0.0);

    constexpr int kTrials = 300;
    int hits = 0;
    for (int trial = 0; trial < kTrials; ++trial) {
        World world;
        SimContext simContext;
        simContext.rng.Seed(1000ULL + static_cast<uint64_t>(trial));

        Vec3 targetPos{1000.0, 5000.0, 0.0};
        Vec3 missilePos{1000.0 + F16Box().halfSpanFt + kMissDistanceFt + 0.5, 5000.0, 0.0};

        EntityID missile = world.CreateEntity();
        world.Components<Transform>().Add(missile, Transform{missilePos, Vec3{}});
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});
        testrig::GiveF16Airframe(world, target);

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.active = true;
        guidance.launchSimTime = 0.0;
        guidance.prevRangeFt = kMissDistanceFt;
        guidance.launchPos = Vec3{missilePos.x() - 2000.0, 5000.0, 0.0};  // comfortably armed
        world.Components<GuidanceComponent>().Add(missile, guidance);

        RunCpaConfirmationTicks(world, simContext, scenario, missile);
        bool hit = world.Components<DestroyedComponent>().Has(missile);
        if (hit) ++hits;
        // A failed roll inside the kill radius is labelled as the warhead's
        // doing, not the guidance's.
        const std::vector<SimEvent>& events = simContext.eventLog.Events();
        ASSERT_FALSE(events.empty());
        EXPECT_EQ(events.back().data.find("warhead=no_kill") != std::string::npos, !hit);
    }

    double empiricalHitRate = static_cast<double>(hits) / kTrials;
    // Generous tolerance -- binomial stddev at Pk~0.77/n=300 is ~0.026;
    // 0.12 is comfortably wide margin against flakiness while still
    // catching an always-hit (rate=1.0) or always-miss (rate=0.0) bug.
    EXPECT_LT(std::fabs(empiricalHitRate - expectedPk), 0.12);
}

// [V21] Sub-tick closest approach recovers the true CPA exactly, not the
// nearest sample. For constant relative velocity r(t)^2 is exactly
// quadratic in t, so a parabola fitted through any three samples must
// recover the true miss distance and CPA time to machine precision.
TEST(PhysicsValidation, SubTickCpa) {
    // v = 24.06 ft/tick is the closure rate actually measured at terminal
    // approach in scenario_basic.
    constexpr double kVFtPerTick = 24.06;

    struct Case {
        const char* name;
        double trueMissFt;  // m
        double t0Ticks;     // where the true CPA sits relative to the middle sample
    };
    const Case cases[] = {
        {"offset CPA, 12ft true miss", 12.0, 0.4},
        {"CPA exactly between samples, 2ft true miss", 2.0, 0.5},
    };

    for (const Case& c : cases) {
        auto rangeAt = [&](double tTicks) {
            double dt = (tTicks - c.t0Ticks) * kVFtPerTick;
            return std::sqrt(c.trueMissFt * c.trueMissFt + dt * dt);
        };
        double rPrevPrev = rangeAt(-1.0), rPrev = rangeAt(0.0), rCurrent = rangeAt(1.0);

        double tOut = 0.0;
        double interpolated = InterceptSystem::SubTickMinRangeFt(rPrevPrev, rPrev, rCurrent, tOut);

        EXPECT_LT(std::fabs(interpolated - c.trueMissFt), 1e-9) << c.name;
        EXPECT_LT(std::fabs(tOut - c.t0Ticks), 1e-9) << c.name;
        // The sampled minimum alone is wrong here, so this check cannot
        // pass on a degenerate case where it happens to be right.
        EXPECT_GT(rPrev - c.trueMissFt, 1.0) << c.name;
    }

    // Non-convex triples (collinear, or concave from noise on a nearly-
    // flat range profile) have no meaningful vertex and must fall back to
    // the sampled minimum rather than extrapolating nonsense.
    double tFlat = 0.0;
    double flat = InterceptSystem::SubTickMinRangeFt(100.0, 100.0, 100.0, tFlat);
    EXPECT_LT(std::fabs(flat - 100.0), 1e-12);
    EXPECT_EQ(tFlat, 0.0);
    double tConcave = 0.0;
    double concave = InterceptSystem::SubTickMinRangeFt(10.0, 500.0, 10.0, tConcave);
    EXPECT_LT(std::fabs(concave - 500.0), 1e-12);
    EXPECT_EQ(tConcave, 0.0);
}

// [V22] Closest approach must be confirmed over consecutive ticks, not
// triggered by one noisy sample. Drives both halves explicitly: a range
// that wobbles open-closed-open must NOT resolve, and the same missile
// then genuinely opening must resolve normally.
TEST(PhysicsValidation, CpaConfirmation) {
    ScenarioData scenario;
    scenario.killRadiusFt = 15.0;
    scenario.falloffWidthFt = 5.0;
    scenario.maxRangeFt = 100000.0;  // large, so the separate max-range MISS branch never fires here

    World world;
    SimContext simContext;
    Vec3 targetPos{1000.0, 5000.0, 0.0};
    Vec3 missilePos{1000.0 + F16Box().halfSpanFt + 20.0, 5000.0, 0.0};

    EntityID missile = world.CreateEntity();
    world.Components<Transform>().Add(missile, Transform{missilePos, Vec3{}});
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});
        testrig::GiveF16Airframe(world, target);

    GuidanceComponent guidance = testrig::GiveAim120(world, missile);
    guidance.target = target;
    guidance.active = true;
    guidance.launchSimTime = 0.0;
    guidance.prevRangeFt = 20.0;
    guidance.prevPrevRangeFt = 20.0;
    guidance.launchPos = Vec3{missilePos.x() - 5000.0, 5000.0, 0.0};  // comfortably armed
    world.Components<GuidanceComponent>().Add(missile, guidance);

    // First, jitter. Alternating tiny open/close steps, well under the
    // ~24ft/tick a real terminal pass opens at, so the opening run can
    // never reach kCpaConfirmTicks.
    const double kWobble[] = {0.01, -0.02, 0.01, -0.02, 0.01, -0.02, 0.01, -0.02};
    for (double delta : kWobble) {
        world.Components<Transform>().Get(missile).pos.x() += delta;
        InterceptSystem::Update(world, simContext, scenario, 1.0);
    }
    EXPECT_TRUE(world.Components<GuidanceComponent>().Get(missile).active);
    EXPECT_EQ(simContext.eventLog.Events().size(), 0u);

    // Then the same missile genuinely departing.
    for (int tick = 0; tick < 4; ++tick) {
        world.Components<Transform>().Get(missile).pos.x() += 5.0;
        InterceptSystem::Update(world, simContext, scenario, 1.0);
    }
    EXPECT_FALSE(world.Components<GuidanceComponent>().Get(missile).active);
    int missEvents = 0;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "MISS") ++missEvents;
    }
    EXPECT_EQ(missEvents, 1);
}

// A range that has only opened since launch is no closest approach: a
// missile leaving the rail slower than a fleeing target falls behind until
// its motor catches it up. It must not resolve while the range opens, and
// must resolve normally once it has closed and then passes.
TEST(PhysicsValidation, NoClosestApproachBeforeTheRangeCloses) {
    ScenarioData scenario;
    scenario.killRadiusFt = 15.0;
    scenario.falloffWidthFt = 5.0;
    scenario.maxRangeFt = 1.0e7;

    World world;
    SimContext simContext;
    Vec3 targetPos{0.0, 5000.0, 0.0};
    Vec3 missilePos{-6000.0, 5000.0, 0.0};  // dead astern
    EntityID missile = world.CreateEntity();
    world.Components<Transform>().Add(missile, Transform{missilePos, Vec3{}});
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});
    testrig::GiveF16Airframe(world, target);

    GuidanceComponent guidance = testrig::GiveAim120(world, missile);
    guidance.target = target;
    guidance.active = true;
    guidance.launchSimTime = 0.0;
    guidance.prevRangeFt = 6000.0 - F16Box().halfLengthFt;
    guidance.prevPrevRangeFt = guidance.prevRangeFt;
    guidance.launchPos = missilePos;
    world.Components<GuidanceComponent>().Add(missile, guidance);

    constexpr double kDt = 1.0 / 120.0;
    double simTime = 0.0;
    auto step = [&](double missileDxFt, double targetDxFt) {
        simTime += kDt;
        world.Components<Transform>().Get(missile).pos.x() += missileDxFt;
        world.Components<Transform>().Get(target).pos.x() += targetDxFt;
        InterceptSystem::Update(world, simContext, scenario, simTime);
    };

    // Two seconds of the target pulling away (1 300 vs 900 ft/s).
    for (int tick = 0; tick < 240; ++tick) step(900.0 * kDt, 1300.0 * kDt);
    EXPECT_TRUE(world.Components<GuidanceComponent>().Get(missile).active);
    EXPECT_TRUE(simContext.eventLog.Events().empty());

    // The motor catches it up, then the missile overshoots alongside.
    for (int tick = 0; tick < 2400 && world.Components<GuidanceComponent>().Get(missile).active; ++tick) {
        step(3000.0 * kDt, 1300.0 * kDt);
    }
    EXPECT_FALSE(world.Components<GuidanceComponent>().Get(missile).active);
    EXPECT_TRUE(simContext.outcome.resolved);
    EXPECT_LT(simContext.outcome.missDistanceFt, 1.0);  // flew straight through the airframe
}

// A target already pulling away never gives a closest approach; once the
// range opens past max range the shot is a MISS at that range.
TEST(PhysicsValidation, RangeOpeningPastMaxRangeIsAMiss) {
    ScenarioData scenario;
    scenario.killRadiusFt = 15.0;
    scenario.falloffWidthFt = 5.0;
    scenario.maxRangeFt = 10000.0;

    World world;
    SimContext simContext;
    Vec3 missilePos{-6000.0, 5000.0, 0.0};
    EntityID missile = world.CreateEntity();
    world.Components<Transform>().Add(missile, Transform{missilePos, Vec3{}});
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{Vec3{0.0, 5000.0, 0.0}, Vec3{}});
    testrig::GiveF16Airframe(world, target);

    GuidanceComponent guidance = testrig::GiveAim120(world, missile);
    guidance.target = target;
    guidance.active = true;
    guidance.prevRangeFt = 6000.0 - F16Box().halfLengthFt;
    guidance.prevPrevRangeFt = guidance.prevRangeFt;
    guidance.launchPos = missilePos;
    world.Components<GuidanceComponent>().Add(missile, guidance);

    constexpr double kDt = 1.0 / 120.0;
    double simTime = 0.0;
    while (world.Components<GuidanceComponent>().Get(missile).active && simTime < 60.0) {
        simTime += kDt;
        world.Components<Transform>().Get(missile).pos.x() += 900.0 * kDt;
        world.Components<Transform>().Get(target).pos.x() += 1300.0 * kDt;
        InterceptSystem::Update(world, simContext, scenario, simTime);
    }
    ASSERT_TRUE(simContext.outcome.resolved);
    EXPECT_FALSE(simContext.outcome.hit);
    EXPECT_GT(simContext.outcome.missDistanceFt, scenario.maxRangeFt);
    EXPECT_LT(simContext.outcome.missDistanceFt, scenario.maxRangeFt + 10.0);  // the tick it crossed
    ASSERT_EQ(simContext.eventLog.Events().size(), 1u);
    EXPECT_EQ(simContext.eventLog.Events().front().type, "MISS");
    EXPECT_FALSE(world.Components<DestroyedComponent>().Has(missile));  // a miss keeps flying
}

// A missed missile keeps coasting until it crosses the ground, then stops
// there; one starting on the ground and climbing is not stopped.
TEST(PhysicsValidation, CoastingMissileStopsAtTheGround) {
    World world;
    EntityID falling = world.CreateEntity();
    world.Components<Transform>().Add(falling, Transform{Vec3{0.0, 300.0, 0.0}, Vec3{}});
    KinematicsComponent down;
    down.vel = Vec3{800.0, -400.0, 0.0};
    world.Components<KinematicsComponent>().Add(falling, down);

    EntityID climbing = world.CreateEntity();
    world.Components<Transform>().Add(climbing, Transform{Vec3{0.0, 0.0, 0.0}, Vec3{}});
    KinematicsComponent up;
    up.vel = Vec3{0.0, 400.0, 0.0};
    world.Components<KinematicsComponent>().Add(climbing, up);

    constexpr double kDt = 1.0 / 120.0;
    for (int tick = 0; tick < 240; ++tick) IntegrationSystem::Update(world, kDt);

    EXPECT_TRUE(world.Components<DestroyedComponent>().Has(falling));
    EXPECT_EQ(world.Components<Transform>().Get(falling).pos.y(), 0.0);
    // Frozen where it hit: 0.75 s of flight at 800 ft/s.
    EXPECT_NEAR(world.Components<Transform>().Get(falling).pos.x(), 600.0, 800.0 * kDt);
    EXPECT_FALSE(world.Components<DestroyedComponent>().Has(climbing));
}

// [V17] Dynamic missile control-authority G-limit.
TEST(PhysicsValidation, MissileGLimit) {
    MissileGLimitComponent missileGLimit = MissileEntity::MakeGLimit(*testrig::Aim120());
    constexpr double kAltFt = 25000.0;
    double rho = Atmosphere::Lookup(kAltFt).densitySlugPerFt3;

    // The aerodynamic ceiling, computed from the type's normal-force model
    // directly: CN at alpha_max on the reference area, over weight.
    constexpr double kGravityFtPerSec2 = 32.174;
    auto aeroLimitG = [&](double speedFtPerSec, double massSlug) {
        Atmosphere::AtmosphereState atm = Atmosphere::Lookup(kAltFt);
        const MissileNormalForce& nf = *missileGLimit.normalForce;
        double cn = nf.Cn(nf.AlphaMaxRad(), speedFtPerSec / atm.speedOfSoundFtPerSec);
        return 0.5 * rho * speedFtPerSec * speedFtPerSec * cn * nf.ReferenceAreaFt2() / (massSlug * kGravityFtPerSec2);
    };

    // V17a: low speed / full mass -- aerodynamically limited.
    constexpr double kLowSpeedFtPerSec = 900.0;  // just-launched speed
    constexpr double kFullMassSlug = 10.4;       // dryMassSlug + propellantMassSlug at launch
    double expectedAeroLimit = aeroLimitG(kLowSpeedFtPerSec, kFullMassSlug);
    double observedLow = FlightEnvelope::GetMissileMaxAvailableG(kLowSpeedFtPerSec, kAltFt, kFullMassSlug, missileGLimit);
    EXPECT_LT(expectedAeroLimit, missileGLimit.structuralGLimit);
    EXPECT_NEAR(observedLow, expectedAeroLimit, 1e-9 * expectedAeroLimit);

    // V17b: high speed -- structurally limited instead, regardless of mass.
    constexpr double kHighSpeedFtPerSec = 3000.0;
    double aeroLimitAtHighSpeed = aeroLimitG(kHighSpeedFtPerSec, kFullMassSlug);
    double observedHigh =
        FlightEnvelope::GetMissileMaxAvailableG(kHighSpeedFtPerSec, kAltFt, kFullMassSlug, missileGLimit);
    EXPECT_GT(aeroLimitAtHighSpeed, missileGLimit.structuralGLimit);
    EXPECT_LT(std::fabs(observedHigh - missileGLimit.structuralGLimit), 1e-9);

    // V17c: full pipeline -- an aggressively large commanded LOS rate fed
    // through real GuidanceSystem::Update calls, comparing early
    // (high speed, full mass) vs. late (lower speed, depleted mass).
    // Confirms applied acceleration is bounded by GetMissileMaxAvailableG
    // at EACH point, and the two available-G values genuinely differ.
    auto sampleClampedAccel = [&](double missileSpeedFtPerSec, double timeSinceLaunch) -> std::pair<double, double> {
        World world;
        SimContext simContext;
        Vec3 missilePos{0.0, kAltFt, 0.0};
        Vec3 missileVel{missileSpeedFtPerSec, 0.0, 0.0};
        Vec3 targetPos{20000.0, kAltFt, 3000.0};

        EntityID missile = testrig::SpawnMover(world, missilePos, missileVel);
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{targetPos, Vec3{}});

        Vec3 losUnitNow = Normalize(Sub(targetPos, missilePos));
        // Deliberately large angular delta -- guarantees a large raw LOS
        // rate, and therefore a large commanded accel, at both points.
        Vec3 prevLosUnit = Normalize(Vec3{losUnitNow.x() + 0.3, losUnitNow.y() - 0.2, losUnitNow.z() + 0.1});

        GuidanceComponent guidance = testrig::GiveAim120(world, missile);
        guidance.target = target;
        guidance.navGain = 4.0;
        guidance.prevLosUnit = prevLosUnit;
        guidance.active = true;
        guidance.launched = true;
        guidance.launchSimTime = -timeSinceLaunch;
        world.Components<GuidanceComponent>().Add(missile, guidance);
        world.Components<MissileGLimitComponent>().Add(missile, MissileEntity::MakeGLimit(*testrig::Aim120()));

        GuidanceSystem::Update(world, simContext, 0.01, 0.0);
        Vec3 observedAccel = world.Components<KinematicsComponent>().Get(missile).accel;

        double burnFraction =
            std::clamp(timeSinceLaunch / (guidance.boostDurationS + guidance.sustainDurationS), 0.0, 1.0);
        double massSlug = guidance.dryMassSlug + guidance.propellantMassSlug * (1.0 - burnFraction);
        double expectedMaxG =
            FlightEnvelope::GetMissileMaxAvailableG(missileSpeedFtPerSec, kAltFt, massSlug, MissileEntity::MakeGLimit(*testrig::Aim120()));

        return {Length(observedAccel), expectedMaxG * kGravityFtPerSec2};
    };

    auto [earlyAccel, earlyMaxAccel] = sampleClampedAccel(2500.0, 0.0);   // early: high speed, full mass
    auto [lateAccel, lateMaxAccel] = sampleClampedAccel(1200.0, 100.0);  // late: lower speed, fully depleted

    // Total accel includes thrust/drag too, not just the clamped PN term,
    // hence the generous margin -- what matters is the PN-driven lateral
    // component didn't blow past the limit, not an exact equality.
    EXPECT_LE(earlyAccel, earlyMaxAccel * 1.5);
    EXPECT_LE(lateAccel, lateMaxAccel * 1.5);
    EXPECT_GT(std::fabs(earlyMaxAccel - lateMaxAccel), 1.0);
}

// [V15] RCS by view direction (the type's geometric shape ESTIMATE):
// V15a the F-16's silhouette is smallest nose- and tail-on, larger
// beam-on, largest from above or below; V15b bank and pitch turn the
// planform toward a level radar; V15c a scenario override of the nose-on
// value rescales every direction, and an unshaped target is the same from
// everywhere; V15d the shape survives end-to-end into the mean SNR.
TEST(PhysicsValidation, AspectRcs) {
    RadarCrossSection f16 = AircraftEntity::MakeRadarCrossSection(*AircraftCatalog::Default().Get("f16c_block50"));
    const Vec3 north{0.0, 0.0, 1.0}, south{0.0, 0.0, -1.0}, east{1.0, 0.0, 0.0}, up{0.0, 1.0, 0.0};
    const Vec3 headingNorth{0.0, 0.0, 0.0};
    double nose = RadarEquation::MeanRcsM2(f16, headingNorth, north);
    double tail = RadarEquation::MeanRcsM2(f16, headingNorth, south);
    double beam = RadarEquation::MeanRcsM2(f16, headingNorth, east);
    double above = RadarEquation::MeanRcsM2(f16, headingNorth, up);
    EXPECT_DOUBLE_EQ(nose, f16.staticRcsM2);
    EXPECT_NEAR(tail, nose, 1e-12);
    EXPECT_GT(beam, 5.0 * nose);
    EXPECT_GT(above, 2.0 * beam);

    // V15b: a radar due east of a target flying north. Banked 90 deg either
    // way the radar looks straight down (or up) at the planform; pitched 90
    // deg up, a radar dead ahead sees the belly; heading east, it sees the nose.
    EXPECT_NEAR(RadarEquation::MeanRcsM2(f16, Vec3{90.0, 0.0, 0.0}, east), above, 1e-9);
    EXPECT_NEAR(RadarEquation::MeanRcsM2(f16, Vec3{-90.0, 0.0, 0.0}, east), above, 1e-9);
    EXPECT_NEAR(RadarEquation::MeanRcsM2(f16, Vec3{0.0, 90.0, 0.0}, north), above, 1e-9);
    EXPECT_NEAR(RadarEquation::MeanRcsM2(f16, Vec3{0.0, 0.0, 90.0}, east), nose, 1e-9);
    double banked30 = RadarEquation::MeanRcsM2(f16, Vec3{30.0, 0.0, 0.0}, east);
    EXPECT_GT(banked30, beam);
    EXPECT_LT(banked30, above);

    // V15c
    RadarCrossSection doubled = f16;
    doubled.staticRcsM2 *= 2.0;
    EXPECT_NEAR(RadarEquation::MeanRcsM2(doubled, headingNorth, east), 2.0 * beam, 1e-9);
    RadarCrossSection sphere;
    sphere.staticRcsM2 = 5.0;
    EXPECT_DOUBLE_EQ(RadarEquation::MeanRcsM2(sphere, Vec3{40.0, 10.0, 200.0}, east), 5.0);
    EXPECT_DOUBLE_EQ(RadarEquation::MeanRcsM2(sphere, headingNorth, north), 5.0);

    // V15d
    const ReceiverBudget budget{200.0, 2000.0, 0.03, 4.0, 0.047, 6.0};
    constexpr double kRangeM = 20000.0;
    double snrNose = RadarEquation::SnrDb(budget, nose, kRangeM);
    double snrBeam = RadarEquation::SnrDb(budget, beam, kRangeM);
    EXPECT_NEAR(snrBeam - snrNose, 10.0 * std::log10(beam / nose), 1e-9);
}

// Where a pass straight through the airframe struck: the
// segment between two samples, both outside the box, enters it at the nose
// face for a head-on pass and at the tail for a tail chase -- not at the
// exit, which is where the zero-distance closest-approach fit lands. A
// segment that misses enters nowhere; one starting inside enters at once.
TEST(PhysicsValidation, PassThroughStrikesWhereItEnters) {
    const double nose = F16Box().halfLengthFt;
    Vec3 entry{};
    ASSERT_TRUE(AircraftBounds::SegmentEntryFromBodyFt(F16Box(), Vec3{0.0, 0.0, nose + 20.0},
                                                       Vec3{0.0, 0.0, -nose - 20.0}, entry));
    EXPECT_NEAR(entry.z(), nose, 1e-9);
    EXPECT_EQ(AircraftBounds::ImpactRegionName(F16Box(), entry), "nose");
    ASSERT_TRUE(AircraftBounds::SegmentEntryFromBodyFt(F16Box(), Vec3{0.0, 1.0, -nose - 20.0},
                                                       Vec3{0.0, 1.0, nose + 20.0}, entry));
    EXPECT_EQ(AircraftBounds::ImpactRegionName(F16Box(), entry), "tail");

    EXPECT_FALSE(AircraftBounds::SegmentEntryFromBodyFt(
        F16Box(), Vec3{F16Box().halfSpanFt + 1.0, 0.0, nose + 20.0}, Vec3{F16Box().halfSpanFt + 1.0, 0.0, -nose}, entry));

    Vec3 inside{1.0, 2.0, 3.0};
    ASSERT_TRUE(AircraftBounds::SegmentEntryFromBodyFt(F16Box(), inside, Vec3{0.0, 0.0, 100.0}, entry));
    EXPECT_EQ(entry.x(), inside.x());
    EXPECT_EQ(entry.z(), inside.z());
}

// [V20] Aircraft bounding-box surface distance (kill_radius_m follow-up).
TEST(PhysicsValidation, AircraftBoundsSurfaceDistance) {
    // V20a: axis-aligned box -- three closed-form cases (a point beyond
    // one face along each axis, interior on the other two), plus one
    // point fully inside the box.
    Vec3 boxCenter{1000.0, 2000.0, 3000.0};  // arbitrary, non-origin -- confirms centering, not just direction
    Vec3 noRotation{0.0, 0.0, 0.0};

    double distSpan = AircraftBounds::DistanceToSurfaceFt(F16Box(),
        Vec3{boxCenter.x() + F16Box().halfSpanFt + 10.0, boxCenter.y(), boxCenter.z()}, boxCenter, noRotation);
    double distHeight = AircraftBounds::DistanceToSurfaceFt(F16Box(),
        Vec3{boxCenter.x(), boxCenter.y() + F16Box().halfHeightFt + 7.0, boxCenter.z()}, boxCenter, noRotation);
    double distLength = AircraftBounds::DistanceToSurfaceFt(F16Box(),
        Vec3{boxCenter.x(), boxCenter.y(), boxCenter.z() + F16Box().halfLengthFt + 3.0}, boxCenter, noRotation);
    double distInside = AircraftBounds::DistanceToSurfaceFt(F16Box(), boxCenter, boxCenter, noRotation);

    EXPECT_LT(std::fabs(distSpan - 10.0), 1e-9);
    EXPECT_LT(std::fabs(distHeight - 7.0), 1e-9);
    EXPECT_LT(std::fabs(distLength - 3.0), 1e-9);
    EXPECT_EQ(distInside, 0.0);

    // V20b: rotated box, independent cross-check -- builds the expected
    // world-space point via the FORWARD body-to-world rotation (roll,
    // then pitch, then yaw), applied to a point `extra` feet beyond the
    // box's nose along its own local +Z axis. If DistanceToSurfaceFt's
    // inverse rotation is correct, feeding that world point back in must
    // recover `extra` regardless of attitude.
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    Vec3 boxEuler{15.0, -8.0, 200.0};  // arbitrary non-axis-aligned attitude
    double extra = 5.5;

    double rollRad = boxEuler.x() * kDegToRad, pitchRad = boxEuler.y() * kDegToRad, yawRad = boxEuler.z() * kDegToRad;
    double localX = 0.0, localY = 0.0, localZ = F16Box().halfLengthFt + extra;  // extra ft beyond the nose
    double cr = std::cos(rollRad), sr = std::sin(rollRad);
    double x1 = localX * cr + localY * sr, y1 = -localX * sr + localY * cr, z1 = localZ;
    double cp = std::cos(pitchRad), sp = std::sin(pitchRad);
    double y2 = y1 * cp + z1 * sp, z2 = -y1 * sp + z1 * cp, x2 = x1;
    double cy = std::cos(yawRad), sy = std::sin(yawRad);
    double worldDx = x2 * cy + z2 * sy, worldDz = -x2 * sy + z2 * cy;  // worldDy = y2
    Vec3 worldPoint{boxCenter.x() + worldDx, boxCenter.y() + y2, boxCenter.z() + worldDz};

    double distRotated = AircraftBounds::DistanceToSurfaceFt(F16Box(), worldPoint, boxCenter, boxEuler);
    EXPECT_LT(std::fabs(distRotated - extra), 1e-6);

    // V20c: impact-region telemetry. Two things checked together, since
    // either alone would be weak: ImpactRegionName maps known body-frame
    // points to the right airframe region, and ToBodyFrameFt actually
    // recovers those body coordinates after an arbitrary attitude round
    // trip.
    struct RegionCase {
        Vec3 body;
        const char* expected;
    };
    const RegionCase regionCases[] = {
        {{0.0, 0.0, F16Box().halfLengthFt * 0.9}, "nose"},
        {{0.0, 0.0, -F16Box().halfLengthFt * 0.9}, "tail"},
        {{F16Box().halfSpanFt * 0.9, 0.0, 0.0}, "starboard_wing"},
        {{-F16Box().halfSpanFt * 0.9, 0.0, 0.0}, "port_wing"},
        {{0.0, 0.0, 0.0}, "mid_fuselage"},
        {{0.0, 0.0, -F16Box().halfLengthFt * 0.5}, "aft_fuselage"},
        {{0.0, -F16Box().halfHeightFt * 0.9, 0.0}, "mid_fuselage_lower"},
        {{0.0, F16Box().halfHeightFt * 0.9, F16Box().halfLengthFt * 0.3},
         "forward_fuselage_upper"},
        // Near field: outside the airframe but still within a warhead's
        // fragmentation envelope.
        {{F16Box().halfSpanFt * 2.0, 0.0, 0.0}, "near_starboard_wing"},
        // Clear misses must NOT claim an airframe region.
        {{-F16Box().halfSpanFt * 50.0, 0.0, 0.0}, "clear_port"},
        {{0.0, -F16Box().halfHeightFt * 500.0, 0.0}, "clear_below"},
        {{0.0, 0.0, F16Box().halfLengthFt * 100.0}, "clear_ahead"},
    };

    for (const RegionCase& testCase : regionCases) {
        EXPECT_EQ(AircraftBounds::ImpactRegionName(F16Box(), testCase.body), testCase.expected);
    }
    // A target with no airframe has no regions.
    EXPECT_EQ(AircraftBounds::ImpactRegionName(AircraftBounds::HitBox{}, Vec3{1.0, 0.0, 0.0}), "point_target");

    // Round-trip the far-outboard starboard point through the same
    // arbitrary attitude used above.
    Vec3 wingBody{F16Box().halfSpanFt * 0.9, 0.0, 0.0};
    double wr = std::cos(rollRad), ws = std::sin(rollRad);
    double wx1 = wingBody.x() * wr + wingBody.y() * ws, wy1 = -wingBody.x() * ws + wingBody.y() * wr, wz1 = wingBody.z();
    double wcp = std::cos(pitchRad), wsp = std::sin(pitchRad);
    double wy2 = wy1 * wcp + wz1 * wsp, wz2 = -wy1 * wsp + wz1 * wcp, wx2 = wx1;
    double wcy = std::cos(yawRad), wsy = std::sin(yawRad);
    Vec3 wingWorld{boxCenter.x() + wx2 * wcy + wz2 * wsy, boxCenter.y() + wy2, boxCenter.z() + (-wx2 * wsy + wz2 * wcy)};

    Vec3 recovered = AircraftBounds::ToBodyFrameFt(wingWorld, boxCenter, boxEuler);
    double recoverErr = std::sqrt((recovered.x() - wingBody.x()) * (recovered.x() - wingBody.x()) +
                                   (recovered.y() - wingBody.y()) * (recovered.y() - wingBody.y()) +
                                   (recovered.z() - wingBody.z()) * (recovered.z() - wingBody.z()));
    EXPECT_LT(recoverErr, 1e-6);
    EXPECT_EQ(AircraftBounds::ImpactRegionName(F16Box(), recovered), "starboard_wing");
}

// [V13] Guidance response time (sim/ResponseLag.h), the one first-order lag
// between demanded and delivered lateral acceleration.
//   step: after tau/dt ticks the delivered value matches the backward-Euler
//         recurrence exactly, and the continuous 1 - 1/e response to ~1 %
//         (a property of the discretization at 24 steps per time constant)
//   tau 0: the demand comes straight back
//   turn:  the held lift turns with the body -- still across the new axis,
//          magnitude kept, then lagged toward the (zero) demand. Held in
//          world axes it would now lie along the flight path: pure thrust.
TEST(PhysicsValidation, ResponseLag) {
    constexpr double kDt = 1.0 / 120.0;
    constexpr double kTau = 0.35;
    const Vec3 north{0.0, 0.0, 2500.0};
    const Vec3 demand{0.0, 100.0, 0.0};
    const double alpha = kDt / (kTau + kDt);

    Vec3 achieved{};
    int stepsToTau = static_cast<int>(kTau / kDt);
    Vec3 out{};
    for (int i = 0; i < stepsToTau; ++i) out = ResponseLag::Step(achieved, demand, north, kTau, kDt);
    double discrete = 100.0 * (1.0 - std::pow(1.0 - alpha, stepsToTau));
    EXPECT_NEAR(out.y(), discrete, 1e-9);
    EXPECT_NEAR(out.y() / (100.0 * (1.0 - std::exp(-1.0))), 1.0, 0.02);

    Vec3 instant{};
    EXPECT_EQ(ResponseLag::Step(instant, demand, north, 0.0, kDt).y(), demand.y());

    Vec3 held{};
    ResponseLag::Step(held, Vec3{100.0, 0.0, 0.0}, north, kTau, kDt);  // across a northbound body: east
    ASSERT_GT(held.x(), 0.0);
    double heldMag = held.x();
    const Vec3 northEast = Normalize(Vec3{1.0, 0.0, 1.0});
    Vec3 turned = ResponseLag::Step(held, Vec3{}, northEast, kTau, kDt);  // the body turned 45 deg toward it
    EXPECT_NEAR(Dot(turned, northEast), 0.0, 1e-9);
    EXPECT_NEAR(Length(turned), heldMag * (1.0 - alpha), 1e-9);
}

// The missile and radar read Atmosphere; every aircraft flies JSBSim's own
// FGStandardAtmosphere. If the two disagreed, a missile and its target at
// the same altitude would be flying in different air. Checked directly
// against JSBSim, including under a temperature offset, which both apply
// hydrostatically.
TEST(PhysicsValidation, AtmosphereAgreesWithJsbsim) {
    JSBSim::FGFDMExec fdm;
    auto atmosphere = std::dynamic_pointer_cast<JSBSim::FGStandardAtmosphere>(fdm.GetAtmosphere());
    ASSERT_NE(atmosphere, nullptr);

    for (double offsetC : {0.0, -25.0, 20.0}) {
        atmosphere->SetTemperatureBias(JSBSim::FGAtmosphere::eCelsius, offsetC);
        for (double altitudeFt : {0.0, 7500.0, 25000.0, 36089.0, 45000.0, 70000.0, 100000.0}) {
            SCOPED_TRACE(::testing::Message() << altitudeFt << " ft, ISA+" << offsetC);
            Atmosphere::AtmosphereState ours = Atmosphere::Lookup(altitudeFt, offsetC);
            EXPECT_NEAR(ours.densitySlugPerFt3 / atmosphere->GetDensity(altitudeFt), 1.0, 1e-9);
            EXPECT_NEAR(ours.speedOfSoundFtPerSec / atmosphere->GetSoundSpeed(altitudeFt), 1.0, 1e-9);
        }
    }
}
