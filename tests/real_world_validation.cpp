// Measures this simulator against published real-world reference data,
// layer by layer, across the whole modeling stack.
//
// Every other validation binary here checks the sim against ITSELF --
// closed-form integration, analytic error budgets, determinism. Those
// establish that the code computes what it claims to compute. They say
// nothing about whether what it claims to compute resembles reality.
// This one asks that question, and only that question.
//
// Structure follows the modeling stack from the ground up: atmosphere,
// then the aircraft that flies in it across several flight phases, then
// the missile across its burn, then the sensors, then the navigation and
// lethality models. A layer that fails invalidates everything above it,
// so they are reported in dependency order.
//
// It reports rather than merely asserting -- each check prints simulated
// value, published value, and percentage error, because the validation
// document is built from these numbers. Provenance for every target
// value, and whether it is a primary or secondary source, is in
// docs/VALIDATION.md.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "FGFDMExec.h"
#include "input_output/FGXMLElement.h"
#include "input_output/FGXMLFileRead.h"
#include "simgear/misc/sg_path.hxx"

#include "validation/AircraftRig.h"
#include "validation/LevelTurnController.h"
#include "validation/TurnDragSampler.h"
#include "aircraft/AircraftCatalog.h"
#include "aircraft/AircraftEntity.h"
#include "components/AircraftComponent.h"
#include "components/AutopilotComponent.h"
#include "components/GLimitComponent.h"
#include "components/GuidanceComponent.h"
#include "components/JSBSimComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileCatalog.h"
#include "missile/MissileEntity.h"
#include "scenario/ScenarioEntity.h"
#include "sim/Atmosphere.h"
#include "sim/FlightEnvelope.h"
#include "sim/SimContext.h"
#include "systems/ControlSystem.h"
#include "systems/FlightDynamicsSystem.h"
#include "sim/RadarEquation.h"

namespace {

constexpr double kDt = testrig::AircraftRig::kDt;
constexpr double kGravityFtPerSec2 = 32.174;
constexpr double kFeetPerMeter = 3.280839895;
constexpr double kKtsToFtPerSec = 1.68781;
constexpr double kSlugToKg = 14.593903;
constexpr double kRadToDeg = 57.29577951308232;

int gChecks = 0;
int gFailures = 0;

void Section(const char* title) { std::printf("\n%s\n%s\n", title, std::string(std::strlen(title), '-').c_str()); }

void Compare(const char* label, double simulated, double published, const char* unit, double tolerancePercent) {
    ++gChecks;
    double errorPercent = published != 0.0 ? 100.0 * (simulated - published) / published : 0.0;
    bool pass = std::fabs(errorPercent) <= tolerancePercent;
    if (!pass) ++gFailures;
    std::printf("  %-36s %10.3f %-8s %10.3f  %+7.1f%%  %s\n", label, simulated, unit, published, errorPercent,
                pass ? "ok" : "OUT");
}

void CompareRange(const char* label, double simulated, double low, double high, const char* unit) {
    ++gChecks;
    bool pass = simulated >= low && simulated <= high;
    if (!pass) ++gFailures;
    std::printf("  %-36s %10.3f %-8s %5.0f-%-5.0f   %-8s %s\n", label, simulated, unit, low, high, "in range",
                pass ? "ok" : "OUT");
}

// For a property that must hold qualitatively rather than numerically.
void Assert(const char* label, bool condition, const char* detail) {
    ++gChecks;
    if (!condition) ++gFailures;
    std::printf("  %-36s %-42s %s\n", label, detail, condition ? "ok" : "OUT");
}

void Note(const char* text) { std::printf("    %s\n", text); }

// =====================================================================
// LAYER 1 - Atmosphere
// =====================================================================
//
// Everything above this depends on it: lift, drag, thrust, the radar
// horizon. Validated against the closed-form ISA troposphere equations
// rather than against the table the sim interpolates, so this tests the
// table's construction and not just its lookup.

void CheckAtmosphere() {
    Section("LAYER 1  Atmosphere -- U.S. Standard Atmosphere 1976");

    // The 1976 standard's first three layers, written out here in SI and
    // in geopotential altitude, independently of the sim's own English-unit
    // implementation. Points between 5 000 ft multiples and above 65 000 ft
    // are included deliberately: between table rows and past the top of a
    // tabulated atmosphere is where an implementation is weakest.
    constexpr double kR = 287.0531;       // J/(kg K)
    constexpr double kG = 9.80665;        // m/s^2
    constexpr double kEarthM = 6356766.0;
    constexpr double kGamma = 1.4;
    struct Layer { double baseM, baseK, lapse; };
    const Layer layers[] = {{0.0, 288.15, -0.0065}, {11000.0, 216.65, 0.0}, {20000.0, 216.65, 0.001}};

    struct Point { double altitudeFt; const char* label; };
    const Point points[] = {{0.0, "sea level"},   {12500.0, "12 500 ft"}, {25000.0, "25 000 ft"},
                            {40000.0, "40 000 ft"}, {70000.0, "70 000 ft"}, {100000.0, "100 000 ft"}};

    for (const Point& point : points) {
        double geometricM = point.altitudeFt / kFeetPerMeter;
        double h = geometricM * kEarthM / (kEarthM + geometricM);
        double pressurePa = 101325.0;
        int i = 0;
        for (; i < 2 && h >= layers[i + 1].baseM; ++i) {
            double dh = layers[i + 1].baseM - layers[i].baseM;
            pressurePa = layers[i].lapse != 0.0
                             ? pressurePa * std::pow(layers[i].baseK / (layers[i].baseK + layers[i].lapse * dh),
                                                     kG / (kR * layers[i].lapse))
                             : pressurePa * std::exp(-kG * dh / (kR * layers[i].baseK));
        }
        double dh = h - layers[i].baseM;
        double temperatureK = layers[i].baseK + layers[i].lapse * dh;
        pressurePa = layers[i].lapse != 0.0
                         ? pressurePa * std::pow(layers[i].baseK / temperatureK, kG / (kR * layers[i].lapse))
                         : pressurePa * std::exp(-kG * dh / (kR * layers[i].baseK));
        // kg/m^3 to slug/ft^3: kilograms to slugs, cubic metres to cubic feet.
        double densitySlugPerFt3 = pressurePa / (kR * temperatureK) / kSlugToKg / std::pow(kFeetPerMeter, 3.0);
        double speedOfSoundFtPerSec = std::sqrt(kGamma * kR * temperatureK) * kFeetPerMeter;

        Atmosphere::AtmosphereState state = Atmosphere::Lookup(point.altitudeFt);
        std::string soundLabel = std::string("speed of sound, ") + point.label;
        Compare(soundLabel.c_str(), state.speedOfSoundFtPerSec, speedOfSoundFtPerSec, "ft/s", 0.1);
        // Reported in units of 1e-3 so the column stays readable; the
        // tolerance is on the ratio, which is what matters physically.
        std::string densityLabel = std::string("density, ") + point.label;
        Compare(densityLabel.c_str(), state.densitySlugPerFt3 * 1e3, densitySlugPerFt3 * 1e3, "e-3 sl/ft3", 0.1);
    }
    Note("closed-form 1976 standard in SI, not the sim's own English-unit code;");
    Note("agreement with JSBSim's atmosphere is checked to 1e-9 in validation_tests");
}

// =====================================================================
// LAYER 2 - F-16
// =====================================================================
//
// The JSBSim model describes itself as an F-16A Block 32 but carries an
// F100-PW-229 (29 000 lbf), which is a Block 50/52-era engine. Published
// targets below are therefore taken from F-16C Block 50/52 data, which is
// the real aircraft this configuration most closely represents.

// Baugher's F-16C Block 50/52 weights: empty, and normal loaded.
constexpr double kEmptyWeightLbf = 18238.0;
constexpr double kLoadedWeightLbf = 26463.0;

// The envelope every scenario F-16 flies: its type's.
GLimitComponent ScenarioF16GLimit() { return AircraftEntity::MakeGLimit(*testrig::F16()); }

// A trimmed F-16 under autopilot control, heading north, with the
// scenario library's G envelope.
struct Aircraft : testrig::AircraftRig {
    // An aircraft of `type` with its type's envelope, as every scenario
    // aircraft flies.
    Aircraft(const AircraftType& type, double altitudeFt, double airspeedKts,
             std::optional<double> internalFuelFraction = {})
        : AircraftRig(type.id.c_str(), altitudeFt, airspeedKts, 0.0, {}, internalFuelFraction) {
        world.Components<GLimitComponent>().Add(id, AircraftEntity::MakeGLimit(type));
    }
    Aircraft(double altitudeFt, double airspeedKts, std::optional<double> internalFuelFraction = {})
        : Aircraft(*testrig::F16(), altitudeFt, airspeedKts, internalFuelFraction) {}

    const JSBSimPropertyNodes& Props() { return world.Components<JSBSimComponent>().Get(id).props; }
    void Run(double seconds) { Fly(seconds); }
    // The trim-to-controller handoff has its own transient; every
    // measurement below starts after it has settled.
    void Settle() { Fly(20.0); }
};

// Afterburning thrust as declared in the engine file f16.xml loads.
double EngineMaxThrustLbf() {
    JSBSim::FGXMLFileRead reader;
    JSBSim::Element* engine = reader.LoadXMLDocument(SGPath("data/jsbsim_data/engine/F100-PW-229.xml"));
    return engine ? engine->FindElementValueAsNumber("maxthrust") : 0.0;
}

void CheckF16Geometry() {
    Section("LAYER 2a  F-16 -- geometry and installed thrust");
    GLimitComponent g = ScenarioF16GLimit();
    Aircraft f16(25000.0, 400.0);
    double modelWingAreaFt2 = f16.Fdm().GetPropertyValue("metrics/Sw-sqft");
    double maxThrustLbf = EngineMaxThrustLbf();

    Compare("wing area (JSBSim model)", modelWingAreaFt2, 300.0, "ft2", 1.0);
    // The G envelope has its own copy of the wing area; it must describe
    // the same airframe the flight model flies.
    Compare("wing area (G envelope vs model)", g.wingAreaFt2, modelWingAreaFt2, "ft2", 1.0);
    Compare("structural G limit", g.structuralGLimitPos, 9.0, "G", 0.1);
    Compare("installed thrust (afterburner)", maxThrustLbf, 29000.0, "lbf", 0.1);
    // Every entity spawns at the model's own loading, which is what every
    // flown figure below is measured at -- not the envelope's static
    // weight field, which ControlSystem replaces with live weight anyway.
    double spawnWeightLbf = f16.Prop("inertia/weight-lbs");
    CompareRange("spawn weight (JSBSim model)", spawnWeightLbf, kEmptyWeightLbf, kLoadedWeightLbf, "lb");
    // Thrust is exact, so thrust/weight is in its published band exactly
    // when weight is; reported rather than checked twice.
    std::printf("    thrust/weight at spawn weight %.2f (published band %.2f loaded - %.2f empty)\n",
                maxThrustLbf / spawnWeightLbf, maxThrustLbf / kLoadedWeightLbf, maxThrustLbf / kEmptyWeightLbf);
    Note("wing area, thrust and weight are read from the loaded JSBSim model and its engine file");
}

// Ground-track turn rate from the velocity vector, so it measures where
// the aircraft is going rather than where its nose points.
double TrackDeg(Aircraft& a) { return std::atan2(a.Prop("velocities/v-east-fps"), a.Prop("velocities/v-north-fps")) * kRadToDeg; }

double WrapDeg(double deg) {
    if (deg > 180.0) deg -= 360.0;
    if (deg < -180.0) deg += 360.0;
    return deg;
}

// Holds the hardest turn the autopilot will fly: a heading target 90 deg
// ahead saturates the bank command at the envelope's limit.
void CommandHardTurn(Aircraft& a) { a.Autopilot().targetHeadingDeg = std::fmod(a.Pose().euler.z() + 90.0, 360.0); }

// A published gross weight as the internal fuel fraction that gives it on
// the type's flight model (its own zero-fuel weight and tank capacity).
double FuelFractionFor(const AircraftType& type, double grossLbf) {
    static std::map<std::string, FlightDynamicsSystem::FuelCapacity> capacities;
    auto found = capacities.find(type.id);
    if (found == capacities.end()) found = capacities.emplace(type.id, FlightDynamicsSystem::FuelCapacityOf(type)).first;
    const FlightDynamicsSystem::FuelCapacity& capacity = found->second;
    return (grossLbf - capacity.zeroFuelWeightLbf) / capacity.internalLbf;
}

struct HeldTurn {
    double rateDegPerSec = 0.0;
    double mach = 0.0;
    double loadFactor = 0.0;
    double altitudeErrorFt = 0.0;
    const char* limit = "thrust";  // what stopped the turn tightening further
};

// Sustained turn rate, measured the way a flight test measures it: a
// level turn at constant Mach, tightened until the engine can no longer
// hold the speed -- or until the stick reaches its stop or the structure
// reaches 9 G, whichever comes first.
//
// ControlSystem is deliberately not in this loop: it clamps its load-factor
// demand to the scenario G envelope, which would make this measure the
// envelope rather than the aircraft. Instead the loops are split the way a
// steep turn is actually flown: stick holds load factor, bank holds
// altitude, throttle holds speed. All three are
// PI, so none carries a standing error into the result. The load-factor
// command walks up while the throttle has margin and back down once it
// has none; at steady state it sits where full thrust just holds Mach.
//
// With `grossLbf` the aircraft is loaded to that weight and its fuel frozen,
// so the turn is measured at one weight, as a flight manual's chart is.
HeldTurn FlySustainedTurn(const AircraftType& type, double altitudeFt, double mach,
                          std::optional<double> grossLbf = {}) {
    double kts = mach * Atmosphere::Lookup(altitudeFt).speedOfSoundFtPerSec / kKtsToFtPerSec;
    std::optional<double> fuelFraction;
    if (grossLbf) fuelFraction = FuelFractionFor(type, *grossLbf);
    Aircraft a(type, altitudeFt, kts, fuelFraction);
    if (grossLbf) a.Fdm().SetPropertyValue("propulsion/fuel_freeze", 1.0);
    a.Settle();
    const JSBSimPropertyNodes& p = a.Props();

    double loadFactorCmd = 2.0;
    testrig::LevelTurnController turn(altitudeFt, loadFactorCmd);
    double throttleIntegral = 0.0;
    double throttle = 0.5;

    HeldTurn result;
    double swept = 0.0;
    double sumNz = 0.0;
    int stickAtStopTicks = 0;
    int gAtLimitTicks = 0;
    double previousTrack = TrackDeg(a);
    const int totalTicks = static_cast<int>(180.0 / kDt);
    const int windowTicks = static_cast<int>(30.0 / kDt);
    for (int i = 0; i < totalTicks; ++i) {
        double nz = p.nz->getDoubleValue();
        double speedErrorKts = kts - p.vtKts->getDoubleValue();
        double altErrorFt = altitudeFt - p.hSlFt->getDoubleValue();
        bool stickAtStop = turn.StickAtStop();
        bool gAtLimit = loadFactorCmd >= 9.0;

        // Throttle margin left means the turn can tighten; none means it
        // must ease -- and speed already lost means it must ease faster,
        // or at altitude, where an initial 2 G is more than can be held,
        // the aircraft bleeds onto the back of the drag curve before a
        // saturated throttle alone backs the G off. At the stick stop, G is
        // capped at what it delivers.
        loadFactorCmd = std::clamp(
            loadFactorCmd + (0.2 * (0.99 - throttle) - 0.05 * std::max(0.0, speedErrorKts)) * kDt, 1.0, 9.0);
        if (stickAtStop) loadFactorCmd = std::min(loadFactorCmd, std::max(1.0, nz));

        throttleIntegral = std::clamp(throttleIntegral + 0.01 * speedErrorKts * kDt, -1.0, 1.0);
        throttle = std::clamp(0.5 + 0.04 * speedErrorKts + throttleIntegral, 0.0, 1.0);

        turn.Step(a, loadFactorCmd, throttle);

        double track = TrackDeg(a);
        if (i >= totalTicks - windowTicks) {
            swept += WrapDeg(track - previousTrack);
            sumNz += p.nz->getDoubleValue();
            result.altitudeErrorFt = std::max(result.altitudeErrorFt, std::fabs(altErrorFt));
            if (stickAtStop) ++stickAtStopTicks;
            if (gAtLimit) ++gAtLimitTicks;
        }
        previousTrack = track;
    }
    double windowSec = windowTicks * kDt;
    result.rateDegPerSec = std::fabs(swept) / windowSec;
    result.loadFactor = sumNz / windowTicks;
    result.mach = p.vtFps->getDoubleValue() / Atmosphere::Lookup(a.Pose().pos.y()).speedOfSoundFtPerSec;
    if (stickAtStopTicks > windowTicks / 2) result.limit = "the stick stop (the FCS's 9 G command)";
    if (gAtLimitTicks > windowTicks / 2) result.limit = "structure (9 G)";
    return result;
}

// Flight-path angle; a turn-rate window counts only while this stays
// within kLevelTurnMaxGammaDeg of level.
constexpr double kLevelTurnMaxGammaDeg = 10.0;
double FlightPathDeg(Aircraft& a) {
    const JSBSimPropertyNodes& p = a.Props();
    return std::asin(std::clamp(-p.vDownFps->getDoubleValue() / std::max(p.vtFps->getDoubleValue(), 1.0), -1.0, 1.0)) *
           kRadToDeg;
}

struct PeakTurn {
    double rateDegPerSec = 0.0;
    double kts = 0.0;
    double loadFactor = 0.0;  // averaged over the peak window
    double maxAlphaDeg = 0.0;  // in the peak window
};

// Instantaneous turn rate, as a flight test measures it: roll to 80 deg
// of bank in a level turn, then full aft stick and full afterburner, bank
// holding altitude, and take the peak rate over quarter-second windows
// before the aircraft bleeds energy. Rolling first keeps the pull in the
// horizontal plane; pulling while still rolling in climbs the aircraft
// and the track rate stops meaning turn rate. The FCS's own limits (9 G,
// ~25 deg AoA) set the pull. The autopilot is not in this loop.
//
// With `grossLbf` the aircraft is loaded to that weight and its fuel frozen.
PeakTurn FlyPeakTurn(const AircraftType& type, double altitudeFt, double kts,
                     std::optional<double> grossLbf = {}) {
    std::optional<double> fuelFraction;
    if (grossLbf) fuelFraction = FuelFractionFor(type, *grossLbf);
    Aircraft a(type, altitudeFt, kts, fuelFraction);
    if (grossLbf) a.Fdm().SetPropertyValue("propulsion/fuel_freeze", 1.0);
    a.Settle();
    const JSBSimPropertyNodes& p = a.Props();
    PeakTurn peak;
    double previousTrack = TrackDeg(a);
    double swept = 0.0;
    double maxGammaDeg = 0.0;
    double windowNz = 0.0;
    double windowMaxAlpha = 0.0;
    double bankBaseDeg = 80.0;
    const double rollInS = 2.5;
    const int windowTicks = static_cast<int>(0.25 / kDt);
    for (int i = 0; i < static_cast<int>((rollInS + 4.0) / kDt); ++i) {
        bool pulling = i * kDt >= rollInS;
        double bankCmdDeg = bankBaseDeg;
        double elevator;
        if (pulling) {
            // Same altitude-holding bank loop as the sustained turn.
            double altErrorFt = altitudeFt - p.hSlFt->getDoubleValue();
            double climbRateFtPerSec = -p.vDownFps->getDoubleValue();
            double verticalError = 0.05 * altErrorFt - climbRateFtPerSec;
            bankBaseDeg = std::clamp(bankBaseDeg - 0.05 * verticalError * kDt, 0.0, 88.0);
            bankCmdDeg = std::clamp(bankBaseDeg - 0.2 * verticalError, 0.0, 88.0);
            elevator = -1.0;
        } else {
            // Level turn while rolling in: the FCS's stick is a G command.
            double cosBank = std::max(std::cos(p.phiDeg->getDoubleValue() / kRadToDeg), 0.2);
            elevator = std::clamp(-(1.0 / cosBank - 1.0) / 8.0, -1.0, 0.0);
        }
        double aileron = std::clamp((0.04 * (bankCmdDeg - p.phiDeg->getDoubleValue()) -
                                     0.01 * p.pRadSec->getDoubleValue() * kRadToDeg) *
                                        std::min(1.0, 250.0 / std::max(a.Prop("aero/qbar-psf"), 1.0)),
                                    -1.0, 1.0);
        p.aileronCmd->setDoubleValue(aileron);
        p.elevatorCmd->setDoubleValue(elevator);
        p.throttleCmd->setDoubleValue(type.controls.maxThrottleCmd);
        FlightDynamicsSystem::Update(a.world, kDt);

        double track = TrackDeg(a);
        swept += WrapDeg(track - previousTrack);
        previousTrack = track;
        if (!pulling) {
            swept = 0.0;
            continue;
        }
        maxGammaDeg = std::max(maxGammaDeg, std::fabs(FlightPathDeg(a)));
        windowNz += a.Props().nz->getDoubleValue();
        windowMaxAlpha = std::max(windowMaxAlpha, a.Prop("aero/alpha-deg"));
        if ((i + 1) % windowTicks == 0) {
            double rate = std::fabs(swept) / (windowTicks * kDt);
            // Only a level turn's track rate is turn rate: in a climbing or
            // sagging spiral the ground track swings faster than the turn.
            bool level = maxGammaDeg <= kLevelTurnMaxGammaDeg;
            maxGammaDeg = 0.0;
            if (level && rate > peak.rateDegPerSec) {
                peak.rateDegPerSec = rate;
                peak.kts = a.Props().vtKts->getDoubleValue();
                peak.loadFactor = windowNz / windowTicks;
                peak.maxAlphaDeg = windowMaxAlpha;
            }
            swept = 0.0;
            windowNz = 0.0;
            windowMaxAlpha = 0.0;
        }
    }
    return peak;
}

// The quickest turn at an altitude and weight: FlyPeakTurn swept over
// entry speed (10 kt steps, true airspeed) and the best taken, as a flight
// manual's turn chart marks it.
PeakTurn QuickestTurn(const AircraftType& type, double altitudeFt, double grossLbf, double fromKts, double toKts) {
    PeakTurn best;
    for (double kts = fromKts; kts <= toKts; kts += 10.0) {
        PeakTurn turn = FlyPeakTurn(type, altitudeFt, kts, grossLbf);
        if (turn.rateDegPerSec > best.rateDegPerSec) best = turn;
    }
    return best;
}

// The same entry speed flown through the autopilot instead: a heading
// 90 deg ahead, re-commanded every tick, saturates the bank command, so
// this is the hardest turn a scenario's aircraft will actually fly.
PeakTurn FlyAutopilotPeakTurn(double altitudeFt, double kts) {
    Aircraft a(altitudeFt, kts);
    a.Settle();
    PeakTurn peak;
    double previousTrack = TrackDeg(a);
    double swept = 0.0;
    double maxGammaDeg = 0.0;
    const int windowTicks = static_cast<int>(0.25 / kDt);
    for (int i = 0; i < static_cast<int>(8.0 / kDt); ++i) {
        CommandHardTurn(a);
        a.Run(kDt);
        double track = TrackDeg(a);
        swept += WrapDeg(track - previousTrack);
        previousTrack = track;
        maxGammaDeg = std::max(maxGammaDeg, std::fabs(FlightPathDeg(a)));
        if ((i + 1) % windowTicks == 0) {
            double rate = std::fabs(swept) / (windowTicks * kDt);
            // Only a level turn's track rate is turn rate: in a climbing or
            // sagging spiral the ground track swings faster than the turn.
            bool level = maxGammaDeg <= kLevelTurnMaxGammaDeg;
            maxGammaDeg = 0.0;
            if (level && rate > peak.rateDegPerSec) {
                peak.rateDegPerSec = rate;
                peak.kts = a.Props().vtKts->getDoubleValue();
            }
            swept = 0.0;
        }
    }
    return peak;
}

void CheckF16Turn() {
    Section("LAYER 2b  F-16 -- turn performance (flown, not computed)");
    Note("sustained: a level turn held at Mach 0.8 and constant altitude, tightened");
    Note("until thrust, the stick stop, or 9 G stops it; instantaneous: rolled to");
    Note("80 deg, then full aft stick, swept over entry speed");

    struct Condition { double altitudeFt; const char* label; double published; double tol; };
    // Only the first has a published figure at exactly this condition.
    const Condition conditions[] = {
        {15000.0, "sustained rate, M0.8 / 15 000ft", 14.0, 20.0},
        {5000.0, "sustained rate, M0.8 /  5 000ft", 0.0, 0.0},
        {10000.0, "sustained rate, M0.8 / 10 000ft", 0.0, 0.0},
        {25000.0, "sustained rate, M0.8 / 25 000ft", 0.0, 0.0},
    };

    std::vector<double> loads;
    for (const Condition& c : conditions) {
        HeldTurn turn = FlySustainedTurn(*testrig::F16(), c.altitudeFt, 0.8);
        loads.push_back(turn.loadFactor);
        if (c.published > 0.0) {
            Compare(c.label, turn.rateDegPerSec, c.published, "deg/s", c.tol);
        } else {
            std::printf("  %-36s %10.3f %-8s %10s  %7s  %s\n", c.label, turn.rateDegPerSec, "deg/s", "--", "--",
                        "no pub");
        }
        std::printf("    held at Mach %.3f, %.2f G, altitude within %.0f ft; limited by %s\n", turn.mach,
                    turn.loadFactor, turn.altitudeErrorFt, turn.limit);
    }

    // Physical necessity: at constant Mach, thinner air means less thrust
    // and less lift, so the load factor the aircraft can sustain cannot
    // rise with altitude. The RATE need not fall: where 9 G caps both, the
    // lower altitude's warmer air makes Mach 0.8 a higher true airspeed
    // and the same G a slower turn. Checked as a shape property because no
    // published figure covers these conditions. (0.02 G: the hold's noise.)
    Assert("sustained G falls with altitude",
           loads[1] >= loads[2] - 0.02 && loads[2] >= loads[0] - 0.02 && loads[0] >= loads[3] - 0.02,
           "5k >= 10k >= 15k >= 25k ft, all at M0.8");

    // Instantaneous rate and corner velocity, both flown: sweep the entry
    // speed, pull at each, and take the best. Corner velocity is by
    // definition the speed at which that best occurs. Flown at 1 000 ft
    // rather than zero so a pull that sags has room to.
    PeakTurn best;
    PeakTurn bestAutopilot;
    for (double kts = 250.0; kts <= 550.0; kts += 10.0) {
        PeakTurn peak = FlyPeakTurn(*testrig::F16(), 1000.0, kts);
        if (peak.rateDegPerSec > best.rateDegPerSec) best = peak;
        PeakTurn autopilot = FlyAutopilotPeakTurn(1000.0, kts);
        if (autopilot.rateDegPerSec > bestAutopilot.rateDegPerSec) bestAutopilot = autopilot;
    }
    CompareRange("corner velocity, sea level", best.kts, 330.0, 440.0, "kt");
    Compare("peak instantaneous rate, sea level", best.rateDegPerSec, 25.5, "deg/s", 25.0);
    Note("at the 20 630 lb spawn weight; the published figure is at a heavier combat weight");
    // The scenarios fly through the autopilot, so what it delivers is
    // checked against what the aircraft can do: a hard heading change must
    // reach most of the airframe's instantaneous rate, not a level turn's
    // fraction of it.
    std::printf("    through the autopilot: %.2f deg/s at %.0f kt\n", bestAutopilot.rateDegPerSec, bestAutopilot.kts);
    Assert("autopilot hard turn reaches the aircraft's rate", bestAutopilot.rateDegPerSec >= 0.9 * best.rateDegPerSec,
           ">= 90% of the flown peak instantaneous rate");

    // Steep turn: the same hard turn held for 25 s, altitude hold working
    // against it. A pilot holds a steep turn to 100 ft (the FAA standard).
    Note("steep turn, autopilot hard turn held 25 s, worst altitude error (limit 100 ft):");
    struct Steep { double altitudeFt, kts; };
    for (const Steep& c : {Steep{10000.0, 450.0}, Steep{15000.0, 520.0}, Steep{25000.0, 500.0}, Steep{35000.0, 550.0}}) {
        Aircraft a(c.altitudeFt, c.kts);
        a.Settle();
        double worst = 0.0, maxBank = 0.0;
        for (int i = 0; i < static_cast<int>(25.0 / kDt); ++i) {
            CommandHardTurn(a);
            a.Run(kDt);
            worst = std::max(worst, std::fabs(a.Props().hSlFt->getDoubleValue() - c.altitudeFt));
            maxBank = std::max(maxBank, std::fabs(a.Props().phiDeg->getDoubleValue()));
        }
        char label[96];
        std::snprintf(label, sizeof(label), "steep turn altitude error, %.0f ft / %.0f kt", c.altitudeFt, c.kts);
        char detail[96];
        std::snprintf(detail, sizeof(detail), "%.0f ft at up to %.0f deg bank", worst, maxBank);
        Assert(label, worst <= 100.0, detail);
    }
}

struct PsPoint {
    double mach;
    double psFtPerSec;
};

struct PsCurve {
    double altitudeFt = 0.0;
    double weightLbf = 0.0;
    std::vector<PsPoint> points;  // empty: no level 1 g trim at any start Mach
    PsPoint Best() const {
        PsPoint best{0.0, 0.0};
        for (const PsPoint& p : points) {
            if (p.psFtPerSec > best.psFtPerSec) best = p;
        }
        return best;
    }
    // Ps where the acceleration passed `mach`; nullopt if it never flew it.
    std::optional<double> At(double mach) const {
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i - 1].mach <= mach && points[i].mach >= mach) {
                double f = (mach - points[i - 1].mach) / (points[i].mach - points[i - 1].mach);
                return points[i - 1].psFtPerSec + f * (points[i].psFtPerSec - points[i - 1].psFtPerSec);
            }
        }
        return std::nullopt;
    }
};

// One level acceleration, the flight-test way to measure 1 g specific
// excess power: trimmed at a low Mach, then full afterburner with the
// autopilot holding altitude, and Ps = dE/dt (E = h + V^2/2g) sampled
// across the run until the aircraft stops gaining speed or passes
// stopMach. The energy rate absorbs whatever altitude the hold gains or
// loses, so Ps does not depend on how tightly it holds. Fuel is frozen,
// so the whole run is at one weight. It starts at the first of startMachs
// that trims -- by default the lowest, so the curve includes the subsonic
// peak.
// The type with its throttle capped at military power: ControlSystem caps
// the throttle at the type's maximum command, so an aircraft flying this
// copy never lights the afterburner -- no special case anywhere.
std::shared_ptr<const AircraftType> MilitaryPowerOnly(const AircraftType& type) {
    AircraftType military = type;
    military.controls.maxThrottleCmd = type.controls.militaryThrottleCmd;
    return std::make_shared<const AircraftType>(military);
}

PsCurve LevelAcceleration(const AircraftType& type, double altitudeFt, double grossLbf, double stopMach = 3.0,
                          std::vector<double> startMachs = {0.6, 0.7, 0.78, 0.82, 0.86, 0.9},
                          bool militaryPower = false) {
    PsCurve curve;
    curve.altitudeFt = altitudeFt;
    for (double startMach : startMachs) {
        double kts = startMach * Atmosphere::Lookup(altitudeFt).speedOfSoundFtPerSec / kKtsToFtPerSec;
        std::optional<Aircraft> trimmed;
        try {
            trimmed.emplace(type, altitudeFt, kts, FuelFractionFor(type, grossLbf));
        } catch (const std::exception&) {
            continue;  // no level 1 g trim this slow at this altitude
        }
        Aircraft& rig = *trimmed;
        if (militaryPower) rig.world.Components<AircraftComponent>().Get(rig.id).type = MilitaryPowerOnly(type);
        rig.Fdm().SetPropertyValue("propulsion/fuel_freeze", 1.0);
        curve.weightLbf = rig.Prop("inertia/weight-lbs");
        rig.Run(5.0);  // trim-to-controller handoff
        rig.Autopilot().targetSpeedKts = 3000.0;
        rig.Run(2.0);  // throttle up
        auto energyHeight = [&] {
            double v = rig.Props().vtKts->getDoubleValue() * kKtsToFtPerSec;
            return rig.Pose().pos.y() + v * v / (2.0 * kGravityFtPerSec2);
        };
        constexpr double kWindowS = 2.0;
        double previousMach = 0.0;
        for (int i = 0; i < static_cast<int>(900.0 / kWindowS); ++i) {
            double e0 = energyHeight();
            double mach0 = rig.Prop("velocities/mach");
            rig.Run(kWindowS);
            double mach1 = rig.Prop("velocities/mach");
            curve.points.push_back({0.5 * (mach0 + mach1), (energyHeight() - e0) / kWindowS});
            if (mach1 > stopMach) break;
            // At top speed: under 0.001 Mach gained in the last 20 s.
            if (i % 10 == 9) {
                if (i > 10 && mach1 - previousMach < 0.001) break;
                previousMach = mach1;
            }
        }
        break;
    }
    return curve;
}

// The altitude where `psAt` falls to `thresholdFtPerSec`, stepping up by
// 1 000 ft from `fromFt` and interpolating linearly to the crossing.
// psAt returns nullopt where JSBSim's trim cannot set up the point, which
// near the ceiling (16 deg alpha) it often cannot even where the aircraft
// flies; Ps falls almost linearly with altitude there, so the crossing is
// then extrapolated from the last two points measured.
template <typename PsAt>
double CeilingFt(double fromFt, double thresholdFtPerSec, PsAt psAt) {
    double altitudes[2] = {0.0, 0.0};
    double ps[2] = {0.0, 0.0};
    int measured = 0;
    for (double altitude = fromFt; altitude <= 75000.0; altitude += 1000.0) {
        std::optional<double> p = psAt(altitude);
        if (p && *p >= thresholdFtPerSec) {
            altitudes[0] = altitudes[1];
            ps[0] = ps[1];
            altitudes[1] = altitude;
            ps[1] = *p;
            ++measured;
            continue;
        }
        if (p && measured > 0) {
            return altitudes[1] + (altitude - altitudes[1]) * (ps[1] - thresholdFtPerSec) / (ps[1] - *p);
        }
        if (measured < 2) return altitude;
        double slope = (ps[1] - ps[0]) / (altitudes[1] - altitudes[0]);
        return altitudes[1] + (thresholdFtPerSec - ps[1]) / slope;
    }
    return altitudes[1];
}

// A flight manual's fuel-flow chart (reference/fuel_flow.csv: power,
// altitude_ft, mach, fuel_flow_pph): fuel flow against Mach, one curve per
// altitude, as the chart's own vertices.
class FuelFlowChart {
public:
    explicit FuelFlowChart(const std::filesystem::path& csv) {
        std::ifstream in(csv);
        std::string line;
        std::getline(in, line);  // header
        while (std::getline(in, line)) {
            std::istringstream fields(line);
            std::string power, altitude, mach, flow;
            if (!std::getline(fields, power, ',') || !std::getline(fields, altitude, ',') ||
                !std::getline(fields, mach, ',') || !std::getline(fields, flow, ',')) {
                continue;
            }
            curves_[power][std::stod(altitude)].emplace_back(std::stod(mach), std::stod(flow));
        }
    }
    bool Empty() const { return curves_.empty(); }

    // lb/hr on the curve at `altitudeFt` (the type's flown altitudes are all
    // charted), or nullopt outside that curve's Mach range.
    std::optional<double> At(const std::string& power, double altitudeFt, double mach) const {
        auto byPower = curves_.find(power);
        if (byPower == curves_.end()) return std::nullopt;
        // The nearest curve, within 500 ft (sea level is flown at 200 ft).
        auto curve = byPower->second.end();
        for (auto c = byPower->second.begin(); c != byPower->second.end(); ++c) {
            if (std::fabs(c->first - altitudeFt) <= 500.0) curve = c;
        }
        if (curve == byPower->second.end()) return std::nullopt;
        const std::vector<std::pair<double, double>>& points = curve->second;
        if (mach < points.front().first || mach > points.back().first) return std::nullopt;
        for (size_t i = 1; i < points.size(); ++i) {
            if (mach <= points[i].first) {
                double t = (mach - points[i - 1].first) / (points[i].first - points[i - 1].first);
                return points[i - 1].second + t * (points[i].second - points[i - 1].second);
            }
        }
        return points.back().second;
    }

private:
    std::map<std::string, std::map<double, std::vector<std::pair<double, double>>>> curves_;
};

struct FlownAcceleration {
    double seconds = 0.0;  // 0: never reached the end speed
    double fuelLbf = 0.0;
    // Against a fuel-flow chart, over the part of the run after the engine's
    // fuel flow has caught up with its thrust (JSBSim slews fuel flow at
    // 1 000 pph/s at military power and 5 000 pph/s in afterburner, so it
    // lags a throttle step by seconds that a manual's table, which starts
    // at steady power, does not have): fuel burned, and fuel the chart's
    // curve gives for the same Mach history.
    double steadyFuelLbf = 0.0;
    double chartFuelLbf = 0.0;
};

// A full-power level acceleration flown as a flight manual's are: from the
// start Mach at the gross weight, the autopilot holding altitude, until the
// end Mach. Fuel burns, so weight falls as the manual's does.
FlownAcceleration FlyAcceleration(const AircraftType& type, double altitudeFt, double grossLbf, double startMach,
                                  double endMach, const FuelFlowChart* chart = nullptr,
                                  const char* chartPower = "MAXAB") {
    double kts = startMach * Atmosphere::Lookup(altitudeFt).speedOfSoundFtPerSec / kKtsToFtPerSec;
    Aircraft rig(type, altitudeFt, kts, FuelFractionFor(type, grossLbf));
    const bool military = std::strcmp(chartPower, "MIL") == 0;
    if (military) rig.world.Components<AircraftComponent>().Get(rig.id).type = MilitaryPowerOnly(type);
    rig.Run(5.0);  // trim-to-controller handoff
    double fuel0 = rig.Prop("inertia/weight-lbs");
    rig.Autopilot().targetSpeedKts = 3000.0;
    FlownAcceleration flown;
    for (int i = 0; i < static_cast<int>(600.0 / kDt); ++i) {
        rig.Run(kDt);
        double mach = rig.Prop("velocities/mach");
        if (chart != nullptr) {
            double flowPps = rig.Prop("propulsion/engine[0]/fuel-flow-rate-pps");
            double steadyPps = rig.Prop("propulsion/engine[0]/thrust-lbs") *
                               rig.Prop(military ? "propulsion/engine[0]/tsfc" : "propulsion/engine[0]/atsfc") / 3600.0;
            std::optional<double> chartPph = chart->At(chartPower, altitudeFt, mach);
            if (chartPph && steadyPps > 0.0 && std::fabs(flowPps - steadyPps) < 0.02 * steadyPps) {
                flown.steadyFuelLbf += flowPps * kDt;
                flown.chartFuelLbf += *chartPph / 3600.0 * kDt;
            }
        }
        if (mach >= endMach) {
            flown.seconds = (i + 1) * kDt;
            break;
        }
    }
    flown.fuelLbf = fuel0 - rig.Prop("inertia/weight-lbs");
    return flown;
}

// Total fuel flow of a steady level cruise, as a flight manual's cruise
// tables give it: the aircraft spawned at the altitude, true airspeed and
// gross weight with the autopilot holding altitude and speed, so the
// throttle settles where thrust balances drag. The mean over the last 20 s
// of a minute's flight, once the speed is within 2% of the target.
struct FlownCruise {
    double fuelFlowPph = 0.0;
    double ktas = 0.0;
    double thrustLbf = 0.0;
    double n2Percent = 0.0;
    bool steady = false;
};

FlownCruise FlyCruise(const AircraftType& type, double altitudeFt, double ktas, double grossLbf) {
    // Sea level is flown at 200 ft: the rig at 0 ft sits on the terrain.
    Aircraft rig(type, std::max(altitudeFt, 200.0), ktas, FuelFractionFor(type, grossLbf));
    rig.Run(40.0);  // trim-to-controller handoff, and the fuel flow's slew
    FlownCruise cruise;
    const int samples = static_cast<int>(20.0 / kDt);
    for (int i = 0; i < samples; ++i) {
        rig.Run(kDt);
        cruise.fuelFlowPph += rig.Prop("propulsion/engine[0]/fuel-flow-rate-pps") * 3600.0 / samples;
        cruise.ktas += rig.Props().vtKts->getDoubleValue() / samples;
        cruise.thrustLbf += rig.Prop("propulsion/engine[0]/thrust-lbs") / samples;
        cruise.n2Percent += rig.Prop("propulsion/engine[0]/n2") / samples;
    }
    cruise.steady = std::fabs(cruise.ktas - ktas) < 0.02 * ktas;
    return cruise;
}

struct BalancedTurn {
    double rateDegPerSec = 0.0;
    double loadFactor = 0.0;
    double cl = 0.0;
    bool valid = false;
};

// Sustained turn rate from the energy balance: in a sustained level turn
// full-power thrust along the flight path equals drag, T cos(alpha) = q S
// CD(CL). The type is pulled to 0.85, 1.0 and 1.15 x `loadFactorGuess` at
// the point (testrig::SampleTurnDrag, the same measurement
// tools/drivers/turn_sample makes); CD and alpha are fitted against CL across the
// three and the balance solved for CL, hence load factor and rate. Steadier
// than holding the turn closed-loop, which at altitude takes minutes of
// slowly decaying speed to settle and settles badly.
BalancedTurn SustainedTurnByBalance(const AircraftType& type, double altitudeFt, double mach, double grossLbf,
                                    double loadFactorGuess) {
    static std::map<std::string, FlightDynamicsSystem::FuelCapacity> capacities;
    auto found = capacities.find(type.id);
    if (found == capacities.end()) found = capacities.emplace(type.id, FlightDynamicsSystem::FuelCapacityOf(type)).first;

    std::vector<double> cl, cd, alpha;
    double thrust = 0.0;
    for (double scale : {0.85, 1.0, 1.15}) {
        std::optional<testrig::TurnDragSample> sample =
            testrig::SampleTurnDrag(type, found->second, altitudeFt, mach, grossLbf, loadFactorGuess * scale);
        if (!sample) return {};
        cl.push_back(sample->cl);
        cd.push_back(sample->cd);
        alpha.push_back(sample->alphaDeg);
        thrust = sample->thrustLbf;
    }
    // Quadratic CD(CL) through the three, linear alpha(CL).
    auto lagrange = [&](const std::vector<double>& y, double x) {
        double total = 0.0;
        for (int i = 0; i < 3; ++i) {
            double term = y[i];
            for (int j = 0; j < 3; ++j) {
                if (j != i) term *= (x - cl[j]) / (cl[i] - cl[j]);
            }
            total += term;
        }
        return total;
    };
    double alphaSlope = (alpha[2] - alpha[0]) / (cl[2] - cl[0]);
    Atmosphere::AtmosphereState air = Atmosphere::Lookup(altitudeFt);
    double speed = mach * air.speedOfSoundFtPerSec;
    double qs = 0.5 * air.densitySlugPerFt3 * speed * speed * type.wingAreaFt2;
    auto excess = [&](double x) {
        double a = (alpha[1] + alphaSlope * (x - cl[1])) * Units::kDegToRad;
        return thrust * std::cos(a) - qs * lagrange(cd, x);
    };
    // Bisection between well below and above the sampled range.
    double lo = cl[0] * 0.5, hi = cl[2] * 1.5;
    if (excess(lo) <= 0.0 || excess(hi) >= 0.0) return {};
    for (int i = 0; i < 60; ++i) {
        double mid = 0.5 * (lo + hi);
        (excess(mid) > 0.0 ? lo : hi) = mid;
    }
    BalancedTurn turn;
    turn.cl = 0.5 * (lo + hi);
    // Thrust points above the path by alpha, so it carries part of the weight.
    double alphaRad = (alpha[1] + alphaSlope * (turn.cl - cl[1])) * Units::kDegToRad;
    turn.loadFactor = (turn.cl * qs + thrust * std::sin(alphaRad)) / grossLbf;
    turn.rateDegPerSec = turn.loadFactor > 1.0 ? std::sqrt(turn.loadFactor * turn.loadFactor - 1.0) *
                                                     kGravityFtPerSec2 / speed * kRadToDeg
                                               : 0.0;
    turn.valid = true;
    return turn;
}

// The altitude where `climbFpm` of 1 g climb remains at `mach`, at full
// power or military power, stepping up from `fromFt`. Each altitude is a
// level acceleration started just below the Mach it measures: near the
// ceiling a slower start sits on the back of the drag curve with no excess
// power to accelerate through to it.
double CeilingAtMach(const AircraftType& type, double grossLbf, double mach, double climbFpm, bool military,
                     double fromFt) {
    return CeilingFt(fromFt, climbFpm / 60.0, [&](double altitude) {
        return LevelAcceleration(type, altitude, grossLbf, mach + 0.05, {mach - 0.03, mach - 0.06}, military)
            .At(mach);
    });
}

// Performance checks any aircraft type can carry: its
// reference/validation.json lists published level accelerations and combat
// ceilings (and their sources), and each is flown here through the whole
// simulator. A new type with a flight manual gets these rows by adding the
// file, with no code.
void CheckAircraftPerformance(const AircraftType& type, const nlohmann::json& spec) {
    std::string title = "LAYER 2c  " + type.displayName + " -- acceleration, climb and ceiling";
    Section(title.c_str());
    std::printf("    source: %s\n", spec.at("source").get<std::string>().c_str());

    // For the F-16 the engine's thrust tables are FITTED to these
    // accelerations (tools/fit/thrust_fit.py, against the model's
    // measured-data drag), so the times check the fit carried through the
    // whole simulator -- the autopilot holding altitude, fuel burning --
    // not the thrust itself. The fuel used is not fitted to these rows: the
    // engine's TSFC is fitted to the fuel-flow chart (tools/fit/fuel_fit.py).
    // The fuel used here includes JSBSim's slew of fuel flow after the
    // throttle step (see the fuel-flow checks below), which a manual's
    // steady-power table does not have.
    for (const nlohmann::json& a : spec.at("accelerations")) {
        double gross = a.at("gross_weight_lb").get<double>();
        FlownAcceleration flown =
            FlyAcceleration(type, a.at("flown_altitude_ft").get<double>(), gross, a.at("start_mach").get<double>(),
                            a.at("end_mach").get<double>());
        std::string label = a.at("label").get<std::string>();
        Compare(label.c_str(), flown.seconds, a.at("time_s").get<double>(), "s", a.at("tolerance_pct").get<double>());
        double manualFuel = a.at("fuel_lb").get<double>();
        std::printf("    %.0f lb; fuel used %.0f lb (manual %.0f lb, %+.0f%%)\n", gross, flown.fuelLbf, manualFuel,
                    100.0 * (flown.fuelLbf - manualFuel) / manualFuel);
    }

    // Fuel flow against the flight manual's fuel-flow chart (a different
    // measurement from the tables above, which the TSFC is fitted to): the
    // same flown accelerations, fuel counted once the fuel flow has caught up
    // with the engine, against the chart's flow at the Mach flown.
    FuelFlowChart chart(type.directory / "reference" / "fuel_flow.csv");
    if (!chart.Empty() && spec.contains("fuel_flow")) {
        const nlohmann::json& flow = spec.at("fuel_flow");
        std::printf("    fuel flow, flown, against %s\n", flow.at("source").get<std::string>().c_str());
        for (const nlohmann::json& row : flow.at("rows")) {
            std::string power = row.at("power").get<std::string>();
            FlownAcceleration flown =
                FlyAcceleration(type, row.at("flown_altitude_ft").get<double>(),
                                row.at("gross_weight_lb").get<double>(), row.at("start_mach").get<double>(),
                                row.at("end_mach").get<double>(), &chart, power.c_str());
            std::string label = row.at("label").get<std::string>();
            Compare(label.c_str(), flown.steadyFuelLbf, flown.chartFuelLbf, "lb", flow.at("tolerance_pct").get<double>());
        }
    }

    // Fuel flow in steady cruise at part throttle, against the flight
    // manual's cruise tables (reference/cruise_fuel.csv, which these rows
    // are read from; not fitted, so a check of the part-throttle TSFC as
    // well as the drag the thrust balances).
    if (spec.contains("cruise_fuel")) {
        const nlohmann::json& cruise = spec.at("cruise_fuel");
        std::printf("    cruise fuel flow, flown, against %s\n", cruise.at("source").get<std::string>().c_str());
        for (const nlohmann::json& row : cruise.at("rows")) {
            FlownCruise flown = FlyCruise(type, row.at("altitude_ft").get<double>(), row.at("ktas").get<double>(),
                                          row.at("gross_weight_lb").get<double>());
            char label[96];
            std::snprintf(label, sizeof(label), "cruise, %.0f ft, %.0f KTAS", row.at("altitude_ft").get<double>(),
                          row.at("ktas").get<double>());
            Compare(label, flown.fuelFlowPph, row.at("fuel_flow_pph").get<double>(), "lb/hr",
                    cruise.at("tolerance_pct").get<double>());
            std::printf("      thrust %.0f lbf, N2 %.1f%%%s\n", flown.thrustLbf, flown.n2Percent,
                        flown.steady ? "" : "  (not steady)");
        }
    }

    // Climb performance is 1 g specific excess power: a steady climb at
    // full power converts it all into height, so the best Ps at an
    // altitude is the best rate of climb there, and the Mach it occurs at
    // is the climb schedule. Measured by level accelerations rather than
    // by flying the autopilot up, which measures its altitude loop.
    const double psWeight = spec.at("accelerations").at(0).at("gross_weight_lb").get<double>();
    std::printf("    1 g Ps at full power, %.0f lb (best over Mach):\n", psWeight);
    for (double altitude : {200.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0, 55000.0}) {
        PsCurve curve = LevelAcceleration(type, altitude, psWeight);
        PsPoint best = curve.Best();
        std::printf("      %6.0f ft  %7.0f ft/min at Mach %.2f\n", altitude, best.psFtPerSec * 60.0, best.mach);
    }

    // Sustained turns: a flight manual's Ps = 0 line, the turn rate the
    // aircraft can hold at full power without losing speed or height. For
    // the F-16 these points are also what its drag above the polar break is
    // fitted to (tools/fit/lift_drag_fit.py), so they check the fit through the
    // simulator; thrust is fitted at 1 g, separately.
    if (spec.contains("sustained_turns")) {
        for (const nlohmann::json& t : spec.at("sustained_turns")) {
            double altitude = t.at("altitude_ft").get<double>();
            double mach = t.at("mach").get<double>();
            double gross = t.at("gross_weight_lb").get<double>();
            double manualRate = t.at("rate_deg_s").get<double>();
            double speed = mach * Atmosphere::Lookup(altitude).speedOfSoundFtPerSec;
            double omegaVOverG = manualRate * Units::kDegToRad * speed / kGravityFtPerSec2;
            double manualN = std::sqrt(omegaVOverG * omegaVOverG + 1.0);
            BalancedTurn turn = SustainedTurnByBalance(type, altitude, mach, gross, manualN);
            char label[64];
            std::snprintf(label, sizeof(label), "sustained turn, M%.2f / %.0f ft", mach, altitude);
            Compare(label, turn.valid ? turn.rateDegPerSec : 0.0, manualRate, "deg/s",
                    t.at("tolerance_pct").get<double>());
            std::printf("    %.0f lb: %.2f G at CL %.2f (manual %.2f G)\n", gross, turn.loadFactor, turn.cl, manualN);

            // The same point flown closed-loop at the manual's weight: an
            // independent check, since the balance above reads thrust and
            // drag off the same sampler the fit used. Only at 15 000 ft: in
            // thinner air the closed-loop hold takes minutes to settle.
            if (altitude <= 15000.0) {
                HeldTurn flown = FlySustainedTurn(type, altitude, mach, gross);
                std::snprintf(label, sizeof(label), "  flown at %.0f lb", gross);
                Compare(label, flown.rateDegPerSec, manualRate, "deg/s", t.at("tolerance_pct").get<double>());
                std::printf("    held at Mach %.3f, %.2f G; limited by %s\n", flown.mach, flown.loadFactor,
                            flown.limit);
            }
        }
    }

    // Quickest turns: the highest turn rate at an altitude and weight, over
    // every speed -- where the lift the flight control system allows at the
    // load factor runs into 9 G. A flight manual's turn chart marks it.
    if (spec.contains("quickest_turns")) {
        for (const nlohmann::json& q : spec.at("quickest_turns")) {
            double altitude = q.at("flown_altitude_ft").get<double>();
            double gross = q.at("gross_weight_lb").get<double>();
            PeakTurn turn = QuickestTurn(type, altitude, gross, q.at("sweep_from_kts").get<double>(),
                                         q.at("sweep_to_kts").get<double>());
            char label[64];
            std::snprintf(label, sizeof(label), "quickest turn, %.0f ft", altitude);
            Compare(label, turn.rateDegPerSec, q.at("rate_deg_s").get<double>(), "deg/s",
                    q.at("tolerance_pct").get<double>());
            std::printf("    %.0f lb, at %.0f kt true, %.2f G, AoA up to %.1f deg\n", gross, turn.kts, turn.loadFactor,
                        turn.maxAlphaDeg);
        }
    }

    // Climb envelopes: across Mach, the altitude where a stated climb rate
    // remains at 1 g -- the shape of the high-altitude thrust and drag,
    // subsonic and supersonic, where the combat ceiling is one point.
    std::ifstream envelopeFile(type.directory / "reference" / "climb_envelope.csv");
    if (envelopeFile) {
        double tolerance = spec.value("climb_envelope_tolerance_pct", 3.0);
        std::string line;
        std::getline(envelopeFile, line);  // power,gross_weight_lb,mach,climb_fpm,altitude_ft,...
        while (std::getline(envelopeFile, line)) {
            std::stringstream row(line);
            std::string power, cell;
            std::getline(row, power, ',');
            double values[4];
            for (double& v : values) {
                std::getline(row, cell, ',');
                v = std::stod(cell);
            }
            bool military = power == "MIL";
            double gross = values[0], mach = values[1], climbFpm = values[2], published = values[3];
            double altitude = CeilingAtMach(type, gross, mach, climbFpm, military,
                                            std::floor((published - 8000.0) / 1000.0) * 1000.0);
            char label[64];
            std::snprintf(label, sizeof(label), "%.0f ft/min envelope, %s, M%.2f", climbFpm,
                          military ? "MIL" : "MAX AB", mach);
            Compare(label, altitude, published, "ft", tolerance);
        }
    }

    // Combat ceilings: the altitude where a stated climb rate remains at
    // full power, at a stated Mach. For the F-16 no thrust is fitted
    // above 40 000 ft (it scales with pressure from the 40 000 ft fit), so
    // these are independent of the fit and also test drag at the high
    // lift coefficients near the ceiling.
    for (const nlohmann::json& c : spec.at("combat_ceilings")) {
        double gross = c.at("gross_weight_lb").get<double>();
        double mach = c.at("mach").get<double>();
        bool military = c.value("power", std::string("MAXAB")) == "MIL";
        double ceiling = CeilingAtMach(type, gross, mach, c.at("climb_fpm").get<double>(), military,
                                       military ? 40000.0 : 48000.0);
        char label[64];
        std::snprintf(label, sizeof(label), "combat ceiling, %s, %.0f lb", military ? "MIL" : "MAX AB", gross);
        Compare(label, ceiling, c.at("altitude_ft").get<double>(), "ft", c.at("tolerance_pct").get<double>());
        std::printf("    %.0f ft/min at %s, Mach %.2f\n", c.at("climb_fpm").get<double>(),
                    military ? "military power" : "full power", mach);
    }
}

// Runs CheckAircraftPerformance for every aircraft type that ships
// reference/validation.json.
void CheckAllAircraftPerformance() {
    for (const std::string& id : AircraftCatalog::Default().Available()) {
        std::shared_ptr<const AircraftType> type = AircraftCatalog::Default().Get(id);
        std::ifstream specFile(type->directory / "reference" / "validation.json");
        if (!specFile) continue;
        CheckAircraftPerformance(*type, nlohmann::json::parse(specFile));
    }
}

struct TopSpeed {
    double mach = 0.0;
    double elapsedS = 0.0;
    double altitudeFt = 0.0;
    double thrustLbf = 0.0;
};

// Commanded to an unreachable speed, so the aircraft accelerates until
// thrust equals drag and settles at its own limit. Run to convergence
// rather than a fixed clock: transonic acceleration has a long tail. The
// run also stops if the engine does (fuel exhausted). Spawned with full
// internal fuel so the tail converges before the tanks run dry.
TopSpeed FlyToTopSpeed(double altitudeFt, double startKts) {
    Aircraft fast(altitudeFt, startKts, 1.0);
    fast.Settle();
    fast.Autopilot().targetSpeedKts = 3000.0;

    TopSpeed top;
    double previousMach = 0.0;
    for (int i = 0; i < 40; ++i) {  // up to 800 s
        fast.Run(20.0);
        if (fast.Prop("propulsion/engine[0]/thrust-lbs") <= 0.0) break;  // fuel exhausted
        top.elapsedS += 20.0;
        top.mach = fast.Prop("velocities/mach");
        if (i > 2 && top.mach - previousMach < 0.001) break;  // converged
        previousMach = top.mach;
    }
    top.thrustLbf = fast.Prop("propulsion/engine[0]/thrust-lbs");
    top.altitudeFt = fast.Pose().pos.y();
    return top;
}

void CheckF16HighSpeed() {
    Section("LAYER 2d  F-16 -- maximum level speed");

    // The published maximum speeds -- Mach 2.05 at 40 000 ft and 1.2 at
    // sea level -- are airspeed LIMITS, not where thrust meets drag: the
    // flight manual's accelerations reach them (800 KIAS / Mach 1.21 at sea
    // level, Mach 2.03 at 40 000 ft) still accelerating. The thrust-limited
    // top speed beyond them has no published value; the model's is
    // reported, and the times to the limits are checked in LAYER 2c.
    TopSpeed high = FlyToTopSpeed(40000.0, 500.0);
    TopSpeed low = FlyToTopSpeed(1000.0, 600.0);
    std::printf("    thrust-limited top speed (no published value; beyond the placard):\n");
    std::printf("      40 000 ft: Mach %.3f after %.0f s (thrust %.0f lbf)\n", high.mach, high.elapsedS, high.thrustLbf);
    std::printf("       1 000 ft: Mach %.3f after %.0f s (thrust %.0f lbf)\n", low.mach, low.elapsedS, low.thrustLbf);
    Assert("reaches the Mach 2.05 limit at 40 000 ft", high.mach >= 2.0, "thrust-limited top speed >= Mach 2.0");
    Assert("reaches the 800 KIAS limit at sea level", low.mach >= 1.21, "thrust-limited top speed >= Mach 1.21");

    // Military versus afterburner at one fixed condition. ControlSystem is
    // deliberately NOT in this loop: it rewrites the throttle every tick,
    // so holding a chosen setting means driving the flight model directly.
    auto steadyThrust = [](double throttleSetting) {
        Aircraft rig(20000.0, 450.0);
        for (int i = 0; i < static_cast<int>(25.0 / kDt); ++i) {
            rig.Props().throttleCmd->setDoubleValue(throttleSetting);
            FlightDynamicsSystem::Update(rig.world, kDt);
        }
        return rig.Prop("propulsion/engine[0]/thrust-lbs");
    };
    // Command 0.5 is military and 1.0 is full afterburner, NOT 1.0 and
    // 2.0: the F-16 model's FCS puts a gain-of-2 stage between throttle
    // command and throttle position, and JSBSim's augmentation range
    // starts at position 1.0. Comparing 1.0 against 2.0 compares full
    // afterburner against itself and reports a spurious zero difference.
    double militaryThrust = steadyThrust(0.5);
    double afterburnerThrust = steadyThrust(1.0);
    std::printf("    at 20 000 ft / 450 kt:  military %.0f lbf | afterburner %.0f lbf  (+%.0f%%)\n", militaryThrust,
                afterburnerThrust, 100.0 * (afterburnerThrust - militaryThrust) / militaryThrust);
    Assert("afterburner produces thrust", afterburnerThrust > militaryThrust * 1.2, "AB > 1.2x military thrust");
}

// =====================================================================
// LAYER 3 - Missile
// =====================================================================

// The missile type validated here, and its per-shot values as a launched
// missile carries them.
const MissileType& Aim120() { return *MissileCatalog::Default().Get("aim120"); }

void CheckMissilePhysical() {
    Section("LAYER 3a  AIM-120 -- mass, geometry, motor");
    const GuidanceComponent m = MissileEntity::MakeGuidance(Aim120());

    Compare("launch mass", (m.dryMassSlug + m.propellantMassSlug) * kSlugToKg, 152.0, "kg", 5.0);
    double diameterMm = 2.0 * std::sqrt(m.referenceAreaFt2 / 3.14159265358979323846) * 12.0 * 25.4;
    Compare("body diameter", diameterMm, 178.0, "mm", 3.0);

    // Total impulse over propellant WEIGHT is specific impulse -- the
    // motor's efficiency, and a hard chemical limit. This is the single
    // most diagnostic number in the missile model.
    double totalImpulse = m.boostThrustLbf * m.boostDurationS + m.sustainThrustLbf * m.sustainDurationS;
    double propellantWeight = m.propellantMassSlug * kGravityFtPerSec2;
    Compare("specific impulse (implied)", totalImpulse / propellantWeight, 245.0, "s", 10.0);
    Compare("propellant mass", m.propellantMassSlug * kSlugToKg, 60.15, "kg", 10.0);
    CompareRange("propellant mass fraction", 100.0 * m.propellantMassSlug / (m.dryMassSlug + m.propellantMassSlug),
                 30.0, 50.0, "%");
    Note("solid HTPB tops out near 245 s -- this check bounds the motor to real chemistry");
}

// Straight-line, unguided thrust-minus-drag integration. Isolates
// propulsion and drag from guidance entirely: no target, no PN, no seeker.
struct FlightProfile {
    double speedAtBoostEndFtPerSec = 0.0;
    double speedAtSustainEndFtPerSec = 0.0;
    double speedAfterCoastFtPerSec = 0.0;
    double peakMach = 0.0;
    double boostAccelFtPerSec2 = 0.0;
    double sustainAccelFtPerSec2 = 0.0;
    double coastDecelFtPerSec2 = 0.0;
};

FlightProfile FlyMissile(double altitudeFt, double launchSpeedFtPerSec, double coastSeconds) {
    const GuidanceComponent m = MissileEntity::MakeGuidance(Aim120());
    Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(altitudeFt);
    double speed = launchSpeedFtPerSec;
    double t = 0.0;
    FlightProfile profile;
    double burnDuration = m.boostDurationS + m.sustainDurationS;
    double launchSpeed = speed;

    int totalTicks = static_cast<int>((burnDuration + coastSeconds) / kDt);
    for (int i = 0; i < totalTicks; ++i) {
        double fraction = burnDuration > 0.0 ? std::min(t / burnDuration, 1.0) : 1.0;
        double massSlug = m.dryMassSlug + m.propellantMassSlug * (1.0 - fraction);
        double thrustLbf = t < m.boostDurationS ? m.boostThrustLbf : (t < burnDuration ? m.sustainThrustLbf : 0.0);

        double mach = speed / atmosphere.speedOfSoundFtPerSec;
        double drag = 0.5 * atmosphere.densitySlugPerFt3 * speed * speed * Aim120().aero.ZeroLiftCd(mach) * m.referenceAreaFt2;
        speed += ((thrustLbf - drag) / massSlug) * kDt;
        t += kDt;

        profile.peakMach = std::max(profile.peakMach, speed / atmosphere.speedOfSoundFtPerSec);
        if (std::fabs(t - m.boostDurationS) < kDt) profile.speedAtBoostEndFtPerSec = speed;
        if (std::fabs(t - burnDuration) < kDt) profile.speedAtSustainEndFtPerSec = speed;
    }
    profile.speedAfterCoastFtPerSec = speed;
    profile.boostAccelFtPerSec2 = (profile.speedAtBoostEndFtPerSec - launchSpeed) / m.boostDurationS;
    profile.sustainAccelFtPerSec2 =
        (profile.speedAtSustainEndFtPerSec - profile.speedAtBoostEndFtPerSec) / m.sustainDurationS;
    profile.coastDecelFtPerSec2 = (profile.speedAtSustainEndFtPerSec - profile.speedAfterCoastFtPerSec) / coastSeconds;
    return profile;
}

void CheckMissilePhases() {
    Section("LAYER 3b  AIM-120 -- flight phases (boost / sustain / coast)");

    FlightProfile mid = FlyMissile(25000.0, 400.0 * kKtsToFtPerSec, 20.0);
    std::printf("    launched 25 000 ft / 400 kt:\n");
    std::printf("      boost   end %7.0f ft/s   (mean accel %+7.1f ft/s2)\n", mid.speedAtBoostEndFtPerSec,
                mid.boostAccelFtPerSec2);
    std::printf("      sustain end %7.0f ft/s   (mean accel %+7.1f ft/s2)\n", mid.speedAtSustainEndFtPerSec,
                mid.sustainAccelFtPerSec2);
    std::printf("      +20s coast  %7.0f ft/s   (mean decel %+7.1f ft/s2)\n", mid.speedAfterCoastFtPerSec,
                -mid.coastDecelFtPerSec2);

    Compare("peak Mach", mid.peakMach, 4.0, "Mach", 20.0);

    // Phase ordering is a physical necessity, independent of any published
    // figure: thrust exceeds drag hardest during boost, less during
    // sustain, and drag alone decelerates in coast.
    Assert("boost accelerates hardest", mid.boostAccelFtPerSec2 > mid.sustainAccelFtPerSec2,
           "boost > sustain acceleration");
    Assert("sustain still accelerating", mid.sustainAccelFtPerSec2 > 0.0, "sustain thrust exceeds drag");
    Assert("coast decelerates", mid.coastDecelFtPerSec2 > 0.0, "unpowered, drag only");

    // Altitude dependence: thinner air means less drag, so the same motor
    // reaches a higher peak Mach. This is the terminal-energy behavior the
    // engagement model depends on.
    FlightProfile high = FlyMissile(40000.0, 400.0 * kKtsToFtPerSec, 20.0);
    FlightProfile low = FlyMissile(5000.0, 400.0 * kKtsToFtPerSec, 20.0);
    std::printf("    peak Mach vs altitude:  5 000ft %.2f | 25 000ft %.2f | 40 000ft %.2f\n", low.peakMach,
                mid.peakMach, high.peakMach);
    Assert("thinner air -> higher peak Mach", high.peakMach > mid.peakMach && mid.peakMach > low.peakMach,
           "40k > 25k > 5k ft");
    Assert("denser air -> faster energy decay", low.coastDecelFtPerSec2 > high.coastDecelFtPerSec2,
           "coast decel at 5k > at 40k ft");
}

void CheckMissileEnvelope() {
    Section("LAYER 3c  AIM-120 -- control authority across the flight");
    const GuidanceComponent m = MissileEntity::MakeGuidance(Aim120());
    const MissileGLimitComponent limit = MissileEntity::MakeGLimit(Aim120());

    double early = FlightEnvelope::GetMissileMaxAvailableG(2500.0, 25000.0, m.dryMassSlug + m.propellantMassSlug, limit);
    double late = FlightEnvelope::GetMissileMaxAvailableG(1200.0, 25000.0, m.dryMassSlug, limit);
    std::printf("    available lateral G:  early (2500 ft/s, full mass) %.1f | late (1200 ft/s, dry) %.1f\n", early,
                late);
    Compare("structural limit, early flight", early, 30.0, "G", 5.0);
    // Early flight sits on the structural cap, so only the direction is
    // meaningful; no published figure sizes the drop.
    Assert("authority falls as it slows", late < early, "late < early");
    Assert("late flight is aero-limited", late < limit.structuralGLimit, "below the structural cap");
    Note("30 G: no published structural limit; the IASGATG CFD study's test load, inside the 25-40 G claims");

    // IASGATG 2014 CFD study [S], burnt-out AIM-120C at alpha 30 deg.
    constexpr double kMps = kFeetPerMeter;
    double transonic = FlightEnvelope::GetMissileMaxAvailableG(1.2 * Atmosphere::Lookup(8000.0 * kFeetPerMeter)
                                                                         .speedOfSoundFtPerSec,
                                                               8000.0 * kFeetPerMeter, m.dryMassSlug, limit);
    Assert("IASGATG >= 10 G transonic (M1.2 8 km)", transonic >= 10.0, "available G at least 10");
    double corner = FlightEnvelope::MissileAeroLimitG(700.0 * kMps, 10000.0 * kFeetPerMeter, m.dryMassSlug, limit);
    Compare("IASGATG 30 G corner, 700 m/s 10 km", corner, 30.0, "G", 10.0);
    Note("KNOWN DISCREPANCY, not tuned: the CN_max table (fitted to published crossflow theory) gives CN(30 deg)");
    Note("~19 vs the CFD's 12-13; no measured L/d ~20 body data exists to settle it (REFERENCE.md 6.5)");
}

// =====================================================================
// LAYER 4 - Sensors
// =====================================================================

void CheckRadar() {
    Section("LAYER 4  Radar and RWR -- range laws");
    SimContext simContext;
    simContext.rng.Seed(1);


    // The F-16 type's own receiver budget.
    const AircraftType& f16 = *AircraftCatalog::Default().Get("f16c_block50");
    const ReceiverBudget budget = ReceiverOf(AircraftEntity::MakeAirborneRadar(f16));

    // The monostatic radar equation falls off as R^4 -- doubling range
    // costs 12.04 dB. This is the defining property of two-way radar.
    double nearSnrDb = RadarEquation::SnrDb(budget, 5.0, 20000.0);
    double farSnrDb = RadarEquation::SnrDb(budget, 5.0, 40000.0);
    Compare("radar falloff over 2x range", nearSnrDb - farSnrDb, 12.0412, "dB", 0.5);
    double quadrupleDb = nearSnrDb - RadarEquation::SnrDb(budget, 5.0, 80000.0);
    Compare("radar falloff over 4x range", quadrupleDb, 24.0824, "dB", 0.5);

    // SNR is linear in RCS: a target twice as reflective returns twice
    // the signal, exactly.
    double doubledSnrDb = RadarEquation::SnrDb(budget, 10.0, 20000.0);
    Compare("signal ratio for 2x RCS", std::pow(10.0, (doubledSnrDb - nearSnrDb) / 10.0), 2.0, "x", 0.5);

    // Radar horizon against the standard 4/3-Earth formula.
    // The 4/3-Earth model: atmospheric refraction bends the beam down,
    // pushing the radar horizon ~15% past the purely geometric one.
    double horizonM = RadarEquation::HorizonRangeM(25000.0 / kFeetPerMeter, 25000.0 / kFeetPerMeter);
    constexpr double kEffectiveEarthRadiusM = 4.0 / 3.0 * 6371000.0;
    double h = 25000.0 / kFeetPerMeter;
    double expectedHorizonM = 2.0 * std::sqrt(2.0 * kEffectiveEarthRadiusM * h);
    Compare("radar horizon, both at 25 000 ft", horizonM / 1000.0, expectedHorizonM / 1000.0, "km", 1.0);

    // And confirm refraction is actually modeled rather than the
    // geometric horizon being reported: the ratio is sqrt(4/3).
    double geometricM = 2.0 * std::sqrt(2.0 * 6371000.0 * h);
    Compare("refraction gain over geometric", horizonM / geometricM, std::sqrt(4.0 / 3.0), "x", 1.0);
    Note("4/3-Earth model, standard radar practice for a refracting atmosphere");

    // One look at a Swerling 1 target (the fighter model): the mean SNR
    // for Pd 0.9 at Pfa 1e-6, against MathWorks' published detectability.
    Compare("Swerling 1 SNR for Pd 0.9, Pfa 1e-6",
            RadarEquation::SwerlingOneRequiredSnrDb(RadarEquation::DetectionThresholdDb(1e-6), 0.9), 21.1436, "dB",
            0.5);
    Note("MathWorks, Modeling Radar Detectability Factors: D1 = 21.1436 dB");
}

// =====================================================================
// LAYER 6 - Lethality
// =====================================================================

void CheckLethality() {
    Section("LAYER 6  Lethality -- sigmoid shape");
    // At exactly the kill radius the sigmoid must return 0.5: that is the
    // definition of the radius, not a tunable.
    Note("Pk = 0.5 at exactly the kill radius, by construction");
    Note("500-run empirical convergence to nominal is reported in REFERENCE.md 15.6");
    Note("(0.928 empirical vs 0.915 nominal -- 1.3 points, one binomial standard error)");
}

}  // namespace

int main() {
    JSBSim::FGJSBBase::debug_lvl = 0;  // its per-model banner would swamp the report

    std::printf("=================================================================\n");
    std::printf(" intercept_sim -- external validation against published data\n");
    std::printf(" columns: simulated | published | error\n");
    std::printf(" source provenance (primary vs secondary) in docs/VALIDATION.md\n");
    std::printf("=================================================================\n");

    CheckAtmosphere();
    CheckF16Geometry();
    CheckF16Turn();
    CheckAllAircraftPerformance();
    CheckF16HighSpeed();
    CheckMissilePhysical();
    CheckMissilePhases();
    CheckMissileEnvelope();
    CheckRadar();
    CheckLethality();

    std::printf("\n=================================================================\n");
    std::printf(" %d checks, %d outside tolerance\n", gChecks, gFailures);
    std::printf("=================================================================\n");
    return 0;  // a measurement instrument; the regression suite gates correctness
}
