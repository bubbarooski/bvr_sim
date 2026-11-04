// F-16 maneuvers checked against KINEMATIC IDENTITIES. JSBSim is the aero
// model, so rather than re-deriving aero math, each maneuver is checked
// against relations the motion must satisfy regardless of the aero model
// (turn rate vs bank and speed, climb angle vs vertical speed, and so on).
//
// Each flight condition is its own TEST() calling a shared helper, so a
// failure at one condition reads as a specifically-named ctest failure
// rather than "the level turn check failed" at one of four conditions.

#include <algorithm>
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "AircraftRig.h"
#include "JsbSimDebugSilencer.h"
#include "ValidationAssertions.h"
#include "aircraft/AircraftEntity.h"
#include "components/GLimitComponent.h"
#include "sim/Atmosphere.h"
#include "sim/FlightEnvelope.h"

namespace {

using validation::NearAbs;
using validation::NearPct;

constexpr double kDt = testrig::AircraftRig::kDt;
constexpr double kG = 32.174;  // ft/s^2
constexpr double kKtsToFps = 1.68781;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kDegToRad = 1.0 / kRadToDeg;

double Wrap180(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg > 180.0) deg -= 360.0;
    if (deg < -180.0) deg += 360.0;
    return deg;
}

// An F-16 rig with named shorthands for the properties the checks read.
struct Rig : testrig::AircraftRig {
    Rig(double altFt, double speedKts, double headingDeg, bool glimit)
        : AircraftRig(testrig::kF16, altFt, speedKts, headingDeg) {
        ctx.rng.Seed(1);
        if (glimit) world.Components<GLimitComponent>().Add(id, AircraftEntity::MakeGLimit(*testrig::F16()));
    }

    double V() { return Prop("velocities/vt-fps"); }
    double Bank() { return Prop("attitude/phi-deg"); }
    double Pitch() { return Prop("attitude/theta-deg"); }
    double Heading() { return Prop("attitude/psi-deg"); }
    double Nz() { return Prop("accelerations/Nz"); }
    double Alpha() { return Prop("aero/alpha-deg"); }
    double VDown() { return Prop("velocities/v-down-fps"); }
    double VNorth() { return Prop("velocities/v-north-fps"); }
    double VEast() { return Prop("velocities/v-east-fps"); }
    double Alt() { return Prop("position/h-sl-ft"); }
    double Weight() { return Prop("inertia/weight-lbs"); }
};

// Level coordinated turns -- the core kinematic identities. Commands a fixed
// heading offset every tick so the autopilot holds a steady bank
// (targetBank = 4*offset), then measures the established turn against
// coordinated-turn geometry.
void CheckLevelTurn(double altFt, double speedKts, double headingOffsetDeg) {
    Rig r(altFt, speedKts, 0.0, /*glimit=*/true);
    r.Fly(18.0);  // settle straight-and-level (F-16 trim handoff transient)

    auto commandTurn = [&]() { r.Autopilot().targetHeadingDeg = std::fmod(r.Heading() + headingOffsetDeg, 360.0); };

    // Establish the turn (long enough to reach a steady coordinated state).
    for (int i = 0; i < static_cast<int>(15.0 / kDt); ++i) {
        commandTurn();
        r.Step();
    }

    // Measure over a window: mean bank, Nz, speed, and the heading swept
    // (for turn rate).
    double sumBank = 0.0, sumNz = 0.0, sumV = 0.0;
    double cumHeadingDeg = 0.0, prevHeading = r.Heading();
    int samples = 0;
    int windowTicks = static_cast<int>(8.0 / kDt);
    for (int i = 0; i < windowTicks; ++i) {
        commandTurn();
        r.Step();
        double h = r.Heading();
        cumHeadingDeg += Wrap180(h - prevHeading);
        prevHeading = h;
        sumBank += r.Bank();
        sumNz += r.Nz();
        sumV += r.V();
        ++samples;
    }
    double windowSec = windowTicks * kDt;
    double bankDeg = sumBank / samples;
    double bankRad = bankDeg * kDegToRad;
    double nz = sumNz / samples;
    double v = sumV / samples;
    double omegaRad = std::fabs(cumHeadingDeg * kDegToRad) / windowSec;

    // Coordinated-turn load factor two independent ways: the accelerometer
    // (Nz) vs n = sqrt(1 + (omega*V/g)^2) from the heading trajectory and
    // speed -- the well-conditioned form of omega = g*sqrt(n^2-1)/V.
    double nFromTurn = std::sqrt(1.0 + (omegaRad * v / kG) * (omegaRad * v / kG));
    EXPECT_TRUE(NearPct(nz, nFromTurn, 4.0, "load factor: accel vs turn kinematics"));
    // Load factor vs bank: exact only for a perfectly coordinated level
    // turn. The F-16 holds a few degrees more bank than its turn rate
    // warrants (slight sideslip), hence the loose tolerance.
    EXPECT_TRUE(NearPct(nz, 1.0 / std::cos(bankRad), 14.0, "load factor n = 1/cos(bank)"));
}

// V-n envelope: commands the hardest turn the autopilot will fly and
// compares the load factor actually pulled to FlightEnvelope::
// GetMaxAvailableG at the same live speed/altitude/weight -- must never
// exceed the envelope, that bound is the limiter's job.
void CheckMaxEffortTurn(double altFt, double speedKts) {
    Rig r(altFt, speedKts, 0.0, /*glimit=*/true);
    r.Fly(15.0);

    // Hard, continuous turn command: 90 deg ahead saturates the bank command.
    auto commandHardTurn = [&]() { r.Autopilot().targetHeadingDeg = std::fmod(r.Heading() + 90.0, 360.0); };
    for (int i = 0; i < static_cast<int>(6.0 / kDt); ++i) {
        commandHardTurn();
        r.Step();
    }

    // Compared tick by tick: a hard turn pulls the available G and bleeds
    // speed, so the envelope shrinks during the window, and the peak load
    // factor early on must not be held to the smaller envelope at its end.
    double worstRatio = 0.0, peakNz = 0.0;
    GLimitComponent baseG = AircraftEntity::MakeGLimit(*testrig::F16());  // what ControlSystem enforces
    baseG.weightLbf = 25000.0;  // static reference weight for the checks against baseG itself
    for (int i = 0; i < static_cast<int>(4.0 / kDt); ++i) {
        commandHardTurn();
        r.Step();
        GLimitComponent live = baseG;
        live.weightLbf = r.Weight();
        double available = FlightEnvelope::GetMaxAvailableG(r.V(), r.Alt(), live);
        worstRatio = std::max(worstRatio, r.Nz() / available);
        peakNz = std::max(peakNz, r.Nz());
    }
    EXPECT_LE(worstRatio, 1.05);
    // A hard turn pulls what the envelope allows rather than a level turn's
    // 1/cos(bank) at a bank short of the cap.
    EXPECT_GT(peakNz, 0.9 * std::min(9.0, FlightEnvelope::GetMaxAvailableG(r.V(), r.Alt(), baseG)));
}

// Steep turn: the hardest turn the autopilot will fly, held for `seconds`,
// while its altitude hold works against the bank. Reports the worst
// altitude error and the bank/load factor reached.
struct SteepTurn {
    double maxAltErrorFt = 0.0;
    double finalAltErrorFt = 0.0;
    double maxBankDeg = 0.0;
    double peakNz = 0.0;
    double headingChangeDeg = 0.0;
};

SteepTurn FlySteepTurn(double altFt, double speedKts, double seconds) {
    Rig r(altFt, speedKts, 0.0, /*glimit=*/true);
    r.Fly(15.0);
    SteepTurn out;
    double heading0 = r.Heading(), previous = heading0;
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
        r.Autopilot().targetHeadingDeg = std::fmod(r.Heading() + 90.0, 360.0);
        r.Step();
        double delta = std::fmod(r.Heading() - previous + 540.0, 360.0) - 180.0;
        out.headingChangeDeg += delta;
        previous = r.Heading();
        out.maxAltErrorFt = std::max(out.maxAltErrorFt, std::fabs(r.Alt() - altFt));
        out.maxBankDeg = std::max(out.maxBankDeg, std::fabs(r.Prop("attitude/phi-deg")));
        out.peakNz = std::max(out.peakNz, r.Nz());
    }
    out.finalAltErrorFt = r.Alt() - altFt;
    return out;
}

// The autopilot's hardest turn (heading target 90 deg ahead, bank near the
// 85 deg cap, G near the envelope) must hold altitude the way a pilot flies
// a steep turn: within 100 ft (the FAA's steep-turn standard). The vertical
// channel feeds forward the accelerometers' vertical specific force, which at
// this bank and AoA includes thrust along the nose and the side force; the
// bank alone held the aircraft 190-340 ft high.
void CheckSteepTurnAltitudeHold(double altFt, double speedKts) {
    SteepTurn t = FlySteepTurn(altFt, speedKts, 25.0);
    EXPECT_LT(t.maxAltErrorFt, 100.0) << altFt << " ft, " << speedKts << " kt";
    EXPECT_LT(std::fabs(t.finalAltErrorFt), 60.0) << altFt << " ft, " << speedKts << " kt";
    EXPECT_GT(t.maxBankDeg, 75.0) << "a steep turn";
    EXPECT_GT(t.headingChangeDeg, 180.0) << "25 s of hard turn passes 180 deg";
}

TEST(AircraftManeuvers, SteepTurnAltitudeHold10kft450kts) { CheckSteepTurnAltitudeHold(10000.0, 450.0); }
TEST(AircraftManeuvers, SteepTurnAltitudeHold15kft520kts) { CheckSteepTurnAltitudeHold(15000.0, 520.0); }
TEST(AircraftManeuvers, SteepTurnAltitudeHold25kft500kts) { CheckSteepTurnAltitudeHold(25000.0, 500.0); }
TEST(AircraftManeuvers, SteepTurnAltitudeHold35kft550kts) { CheckSteepTurnAltitudeHold(35000.0, 550.0); }

// Climb and dive -- velocity-vector and flight-path-angle consistency.
void CheckClimbOrDive(double altFt, double speedKts, double targetAltFt, double targetSpeedKts) {
    Rig r(altFt, speedKts, 90.0, /*glimit=*/false);
    r.Fly(20.0);

    r.Autopilot().targetAltitudeFt = targetAltFt;
    r.Autopilot().targetSpeedKts = targetSpeedKts;
    r.Fly(12.0);  // establish a steady climb/dive angle

    double sumV = 0.0, sumVN = 0.0, sumVE = 0.0, sumVD = 0.0, sumTheta = 0.0, sumAlpha = 0.0;
    double altStart = r.Alt();
    double tStart = r.t;
    int samples = 0;
    for (int i = 0; i < static_cast<int>(4.0 / kDt); ++i) {
        r.Step();
        sumV += r.V();
        sumVN += r.VNorth();
        sumVE += r.VEast();
        sumVD += r.VDown();
        sumTheta += r.Pitch();
        sumAlpha += r.Alpha();
        ++samples;
    }
    double altEnd = r.Alt();
    double dtWindow = r.t - tStart;

    double v = sumV / samples;
    double vN = sumVN / samples, vE = sumVE / samples, vD = sumVD / samples;
    double theta = sumTheta / samples, alpha = sumAlpha / samples;

    // Speed scalar vs the magnitude of its own components.
    EXPECT_TRUE(NearPct(std::sqrt(vN * vN + vE * vE + vD * vD), v, 1.0, "|V| vs sqrt(vN^2+vE^2+vD^2)"));
    // Climb rate two ways: velocity state vs position integration.
    double climbRateVel = -vD;
    double climbRatePos = (altEnd - altStart) / dtWindow;
    EXPECT_TRUE(NearPct(climbRateVel, climbRatePos, 3.0, "climb rate: -vDown vs d(alt)/dt"));
    // Flight-path angle from the velocity vector vs pitch minus angle of
    // attack (wings level, so gamma = theta - alpha).
    double gammaVelDeg = std::asin(std::max(-1.0, std::min(1.0, -vD / v))) * kRadToDeg;
    EXPECT_TRUE(NearAbs(gammaVelDeg, theta - alpha, 1.5, "flight-path angle vs (pitch - AoA)"));
}

// Pull-up -- the vertical-plane load-factor/turn-rate identity. In a
// wings-level pull, flight-path angle rotates at dGamma/dt = g*(n-1)/V.
// Enter a dive, then command level flight and measure the pull.
void CheckPullUp(double altFt, double speedKts) {
    Rig r(altFt, speedKts, 90.0, /*glimit=*/true);
    r.Fly(18.0);

    r.Autopilot().targetAltitudeFt = altFt - 8000.0;
    r.Fly(10.0);

    auto gammaDeg = [&]() { return std::asin(std::max(-1.0, std::min(1.0, -r.VDown() / r.V()))) * kRadToDeg; };

    // Command the pull-out and measure while gamma is actively rotating upward.
    r.Autopilot().targetAltitudeFt = altFt + 4000.0;
    double sumNz = 0.0, sumV = 0.0;
    double gammaStart = gammaDeg();
    double tStart = r.t;
    int samples = 0;
    int ticks = static_cast<int>(2.5 / kDt);
    for (int i = 0; i < ticks; ++i) {
        r.Step();
        sumNz += r.Nz();
        sumV += r.V();
        ++samples;
    }
    double gammaEnd = gammaDeg();
    double dtWindow = r.t - tStart;
    double v = sumV / samples;
    double nz = sumNz / samples;

    double gammaDotMeasRad = (gammaEnd - gammaStart) * kDegToRad / dtWindow;
    double gammaDotPredRad = kG * (nz - 1.0) / v;
    EXPECT_TRUE(NearPct(gammaDotMeasRad * kRadToDeg, gammaDotPredRad * kRadToDeg, 12.0, "dGamma/dt = g*(n-1)/V"));
}

// Dive-then-turn -- combined maneuver stays kinematically consistent and
// inside the envelope.
void CheckDiveThenTurn(double altFt, double speedKts) {
    Rig r(altFt, speedKts, 0.0, /*glimit=*/true);
    r.Fly(18.0);

    r.Autopilot().targetAltitudeFt = altFt - 6000.0;  // dive
    r.Fly(6.0);

    // Hard turn while still nose-low.
    auto commandTurn = [&]() { r.Autopilot().targetHeadingDeg = std::fmod(r.Heading() + 60.0, 360.0); };
    for (int i = 0; i < static_cast<int>(4.0 / kDt); ++i) {
        commandTurn();
        r.Step();
    }

    double sumNz = 0.0;
    double maxOverG = 0.0;
    int samples = 0;
    GLimitComponent baseG = AircraftEntity::MakeGLimit(*testrig::F16());
    baseG.weightLbf = 25000.0;
    for (int i = 0; i < static_cast<int>(4.0 / kDt); ++i) {
        commandTurn();
        r.Step();
        GLimitComponent live = baseG;
        live.weightLbf = r.Weight();
        double avail = FlightEnvelope::GetMaxAvailableG(r.V(), r.Alt(), live);
        maxOverG = std::max(maxOverG, r.Nz() - avail);
        sumNz += r.Nz();
        ++samples;
    }
    double nz = sumNz / samples;
    // It genuinely loads up in the descending turn...
    EXPECT_GT(nz, 1.2);
    // ...and the G-limiter kept it inside the envelope (small transient ok).
    EXPECT_LT(maxOverG, 0.75);
    EXPECT_FALSE(r.Destroyed());
}

}  // namespace

// Slow through high-speed -- different banks/radii, same geometry. Offset
// sets the commanded bank (4*offset); ~38-42 deg keeps the turn sustainable
// and the sqrt(n^2-1) identity well-conditioned. The high-speed cases
// exercise the dynamic-pressure roll-gain scheduling in ControlSystem.
TEST(AircraftManeuvers, LevelTurnSlow12kft340kts) { CheckLevelTurn(12000.0, 340.0, 10.5); }   // ~42 deg bank
TEST(AircraftManeuvers, LevelTurnMedium15kft430kts) { CheckLevelTurn(15000.0, 430.0, 9.5); }   // ~38 deg bank
TEST(AircraftManeuvers, LevelTurnFast18kft520kts) { CheckLevelTurn(18000.0, 520.0, 9.5); }     // ~38 deg bank
TEST(AircraftManeuvers, LevelTurnHighSpeed20kft620kts) { CheckLevelTurn(20000.0, 620.0, 9.5); }

TEST(AircraftManeuvers, MaxEffortTurn12kft450kts) { CheckMaxEffortTurn(12000.0, 450.0); }
TEST(AircraftManeuvers, MaxEffortTurn16kft600kts) { CheckMaxEffortTurn(16000.0, 600.0); }

// The V-n envelope model itself, in closed form (no flying): available G
// is the aerodynamic lift limit 0.5*rho*V^2*ClMax*S/W until it reaches the
// structural cap, then flat. Corner velocity is where the two meet.
TEST(AircraftManeuvers, EnvelopeModel) {
    GLimitComponent g = AircraftEntity::MakeGLimit(*testrig::F16());
    g.weightLbf = 25000.0;

    struct Point {
        double vKts;
        double altFt;
        const char* regime;
    };
    const Point points[] = {
        {250.0, 15000.0, "aero-limited"}, {350.0, 15000.0, "aero-limited"}, {700.0, 15000.0, "structural cap"}};
    for (const Point& p : points) {
        double vFps = p.vKts * kKtsToFps;
        double rho = Atmosphere::Lookup(p.altFt).densitySlugPerFt3;
        // Lift limit with the G/AoA blend: n = (clMax + L) Q / (1 + L Q).
        double q = 0.5 * rho * vFps * vFps * g.wingAreaFt2 / g.weightLbf;
        double aero = (g.clMax + g.clMaxLossPerG) * q / (1.0 + g.clMaxLossPerG * q);
        double expected = std::min(aero, g.structuralGLimitPos);
        std::string label = std::string("max-G @ ") + std::to_string(static_cast<int>(p.vKts)) + " kts";
        EXPECT_TRUE(NearPct(FlightEnvelope::GetMaxAvailableG(vFps, p.altFt, g), expected, 0.5, label.c_str()));
    }

    // Corner velocity: the speed at which the aero limit reaches the
    // structural limit. Just above it, available G must equal the cap.
    double altFt = 15000.0;
    double rho = Atmosphere::Lookup(altFt).densitySlugPerFt3;
    double clAtCap = g.clMax - g.clMaxLossPerG * (g.structuralGLimitPos - 1.0);
    double vCornerFps = std::sqrt(2.0 * g.weightLbf * g.structuralGLimitPos / (rho * clAtCap * g.wingAreaFt2));
    EXPECT_TRUE(NearPct(FlightEnvelope::GetMaxAvailableG(vCornerFps * 1.01, altFt, g), g.structuralGLimitPos, 1.0,
                         "max-G just above corner velocity = cap"));
}

TEST(AircraftManeuvers, SteadyClimb) { CheckClimbOrDive(15000.0, 450.0, 30000.0, 550.0); }
TEST(AircraftManeuvers, Dive) { CheckClimbOrDive(30000.0, 450.0, 15000.0, 450.0); }

TEST(AircraftManeuvers, PullUp) { CheckPullUp(20000.0, 550.0); }

TEST(AircraftManeuvers, DiveThenTurn) { CheckDiveThenTurn(22000.0, 550.0); }

// --- The reversal latch commits to a turn direction through a near-180
// error so the sign cannot flip on noise. It commits to one target: when
// the command jumps (the next waypoint), the commitment is dropped, and the
// aircraft turns whichever way is now shorter.
TEST(AircraftManeuvers, ANewHeadingCommandBreaksAReversalLatch) {
    testrig::AircraftRig rig(testrig::kF16, 25000.0, 450.0, 90.0);
    rig.Autopilot().targetHeadingDeg = 90.0 + 178.0;  // 178 deg to the right: latches right
    rig.Step();
    ASSERT_EQ(rig.Autopilot().turnDirection, 1);

    rig.Autopilot().targetHeadingDeg = 90.0 - 160.0 + 360.0;  // now 160 deg to the LEFT
    rig.Fly(1.0);
    EXPECT_LT(rig.Prop("attitude/phi-deg"), -5.0) << "banked right, the long way round";
}

// A bearing that drifts slowly keeps the latch: it is the same command.
TEST(AircraftManeuvers, ASlowlyDriftingHeadingCommandKeepsTheLatch) {
    testrig::AircraftRig rig(testrig::kF16, 25000.0, 450.0, 90.0);
    rig.Autopilot().targetHeadingDeg = 90.0 + 178.0;
    rig.Step();
    ASSERT_EQ(rig.Autopilot().turnDirection, 1);
    for (int i = 0; i < 60; ++i) {
        rig.Autopilot().targetHeadingDeg += 0.2;  // 24 deg/s of drift, far under a jump per tick
        rig.Step();
    }
    EXPECT_EQ(rig.Autopilot().turnDirection, 1);
}
