// Radar behavior through the real pipeline: unlike the guidance tests,
// which build entities directly, these load real scenario JSON through
// ScenarioLoader and drive the real systems -- M-of-N confirmation, the
// firing-solution launch gate, the pulse-Doppler notch, and FOV gating.

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "aircraft/AircraftCatalog.h"
#include "aircraft/AircraftEntity.h"
#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Transform.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileType.h"
#include "scenario/ScenarioLoader.h"
#include "scenario/ScenarioLoaderJson.h"
#include "sim/RadarEquation.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "systems/AirborneRadarSystem.h"
#include "systems/ControlSystem.h"
#include "systems/FlightDynamicsSystem.h"
#include "systems/GuidanceSystem.h"
#include "systems/IntegrationSystem.h"
#include "systems/InterceptSystem.h"
#include "systems/LaunchSystem.h"
#include "systems/RwrSystem.h"
#include "systems/WaypointSystem.h"
#include "util/EntityLookup.h"
#include "util/EntityVelocity.h"

namespace {

using EntityLookup::FindByTagName;

constexpr double kFeetPerMeter = 3.280839895;

double RangeFt(const Vec3& a, const Vec3& b) {
    double dx = a.x() - b.x(), dy = a.y() - b.y(), dz = a.z() - b.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// The target's own GROUND-relative velocity resolved onto the line of
// sight to the observer -- deliberately the exact quantity
// AirborneRadarSystem's pulse-Doppler notch gate tests, so a "beaming"
// window detected here is by construction the same condition the radar
// is filtering on. Sign is irrelevant to callers (they take |.|). See
// AirborneRadarSystem.cpp -- the observer's own motion shifts the
// clutter notch by exactly as much as it shifts the target return, so
// the observer's velocity cancels out of the comparison entirely.
double GroundRadialVelocityFtPerSec(const Vec3& targetPos, const Vec3& targetVel, const Vec3& observerPos) {
    double rangeFt = RangeFt(targetPos, observerPos);
    if (rangeFt < 1e-6) return 0.0;
    Vec3 losUnit{(observerPos.x() - targetPos.x()) / rangeFt, (observerPos.y() - targetPos.y()) / rangeFt,
                 (observerPos.z() - targetPos.z()) / rangeFt};
    return targetVel.x() * losUnit.x() + targetVel.y() * losUnit.y() + targetVel.z() * losUnit.z();
}

constexpr double kDt = 0.25;

// --- Full-pipeline regression. LaunchSystem's firing-solution
// gate replaces a pure range trigger, so this re-runs the original
// intercept check through the REAL pipeline -- ScenarioLoader + every
// system's Update runs, at a fixed 120Hz substep -- to confirm a missile
// still launches and still hits under 15m.
TEST(RadarValidation, FullPipelineRegressionScenarioBasic) {
    World world;
    SimContext simContext;

    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load("tests/fixtures/scenarios/scenario_basic.json", world);
    }) << "failed to load scenario_basic.json";
    SimLoop::BeginRun(scenario, simContext, scenario.seed);

    EntityID pursuer = FindByTagName(world, "Pursuer").value_or(0);
    EntityID target = FindByTagName(world, "Target").value_or(0);
    ASSERT_NE(pursuer, 0u) << "scenario_basic.json missing Pursuer tag";
    ASSERT_NE(target, 0u) << "scenario_basic.json missing Target tag";
    LaunchSystem::Init(world, pursuer, target, scenario);

    constexpr double kFixedDt = 1.0 / 120.0;
    // scenario_basic.json's Pursuer/Target start ~91km apart (F-16 BVR
    // geometry) -- generous enough to cover closing to search range
    // (~20s), the 3-of-5/4-of-5/5-of-5 tier climb, and missile flight to
    // intercept.
    constexpr double kMaxSimTime = 300.0;
    double simTime = 0.0;
    double bestMissileRangeFt = -1.0;  // stays -1 until the missile actually exists

    ComponentStore<Transform>& transforms = world.Components<Transform>();
    while (simTime < kMaxSimTime && !simContext.outcome.resolved) {
        ControlSystem::Update(world, simContext, kFixedDt, simTime);
        FlightDynamicsSystem::Update(world, kFixedDt);
        GuidanceSystem::Update(world, simContext, kFixedDt, simTime);
        IntegrationSystem::Update(world, kFixedDt);
        simTime += kFixedDt;
        AirborneRadarSystem::Update(world, simContext, simTime);
        LaunchSystem::Update(world, simContext, scenario, simTime);
        InterceptSystem::Update(world, simContext, scenario, simTime);

        EntityID missile = FindByTagName(world, "Missile").value_or(0);
        if (missile != 0 && transforms.Has(missile) && transforms.Has(target)) {
            double range = RangeFt(transforms.Get(missile).pos, transforms.Get(target).pos);
            if (bestMissileRangeFt < 0.0 || range < bestMissileRangeFt) bestMissileRangeFt = range;
        }
    }

    bool launched = false;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "LAUNCH") launched = true;
    }
    EXPECT_TRUE(launched);
    ASSERT_GE(bestMissileRangeFt, 0.0) << "missile never spawned";
    double bestMissDistanceM = bestMissileRangeFt / kFeetPerMeter;
    EXPECT_LT(bestMissDistanceM, 15.0) << "closest miss=" << bestMissDistanceM << "m";
}

// AirborneRadarComponent's default -- scenario_notch_beam.json
// doesn't override it, so this is the literal threshold AirborneRadarSystem's
// Doppler notch filter gates on for this scenario.
constexpr double kNotchVelocityThresholdMps = 15.0;

// Shared trial logic behind both the single-seed and many-seed checks:
// runs one full scenario_notch_beam.json engagement and reports whether
// the Target's scripted beaming leg (a waypoint-authored turn to a
// perpendicular heading) produced at least one RADAR_TRACK_DROPPED
// correlated with a real notch-condition window. seedOverride selects an
// arbitrary RNG seed (for many-seed sweeps); std::nullopt uses the
// scenario's own authored seed. Returns false only on scenario-load/tag
// failure (caller reports the specific error via *errorOut).
bool RunNotchTrial(std::optional<uint64_t> seedOverride, size_t& beamWindowsOut, size_t& downgradeEventsOut,
                    int& correlatedOut, std::string& errorOut) {
    beamWindowsOut = 0;
    downgradeEventsOut = 0;
    correlatedOut = 0;

    World world;
    SimContext simContext;

    ScenarioData scenario;
    try {
        scenario = ScenarioLoader::Load("tests/fixtures/scenarios/scenario_notch_beam.json", world);
    } catch (const std::exception& e) {
        errorOut = std::string("failed to load scenario_notch_beam.json: ") + e.what();
        return false;
    }
    SimLoop::BeginRun(scenario, simContext, seedOverride.value_or(scenario.seed));

    EntityID pursuer = FindByTagName(world, "Pursuer").value_or(0);
    EntityID target = FindByTagName(world, "Target").value_or(0);
    if (pursuer == 0 || target == 0) {
        errorOut = "scenario_notch_beam.json missing Pursuer/Target tags";
        return false;
    }
    LaunchSystem::Init(world, pursuer, target, scenario);

    constexpr double kFixedDt = 1.0 / 120.0;
    constexpr double kMaxSimTime = 300.0;
    constexpr double kNotchVelocityThresholdFtPerSec = kNotchVelocityThresholdMps * kFeetPerMeter;
    double simTime = 0.0;

    // Interval-based, not point-based: collect [start, end) windows where
    // the Target's real closing rate on its resolved threat stayed below
    // the notch filter's own threshold (i.e. genuinely beaming), then
    // check whether any track drop fell inside [start, end + buffer].
    std::vector<std::pair<double, double>> beamIntervals;
    double beamStart = -1.0;

    ComponentStore<Transform>& transforms = world.Components<Transform>();

    while (simTime < kMaxSimTime) {
        WaypointSystem::Update(world, scenario.environment);
        ControlSystem::Update(world, simContext, kFixedDt, simTime);
        FlightDynamicsSystem::Update(world, kFixedDt);
        GuidanceSystem::Update(world, simContext, kFixedDt, simTime);
        IntegrationSystem::Update(world, kFixedDt);
        simTime += kFixedDt;
        AirborneRadarSystem::Update(world, simContext, simTime);
        RwrSystem::Update(world, simContext, simTime);
        LaunchSystem::Update(world, simContext, scenario, simTime);
        InterceptSystem::Update(world, simContext, scenario, simTime);

        bool beaming = false;
        if (transforms.Has(target) && transforms.Has(pursuer)) {
            const Vec3& targetPos = transforms.Get(target).pos;
            const Vec3& pursuerPos = transforms.Get(pursuer).pos;
            double targetRadialFtPerSec =
                GroundRadialVelocityFtPerSec(targetPos, EntityVelocity::GetWithVelocityFallback(world, target),
                                             pursuerPos);
            beaming = std::fabs(targetRadialFtPerSec) < kNotchVelocityThresholdFtPerSec;
        }
        if (beaming) {
            if (beamStart < 0.0) beamStart = simTime;
        } else if (beamStart >= 0.0) {
            beamIntervals.push_back({beamStart, simTime});
            beamStart = -1.0;
        }
    }
    if (beamStart >= 0.0) beamIntervals.push_back({beamStart, kMaxSimTime});

    std::vector<double> downgradeTimes;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "RADAR_TRACK_DROPPED" && event.entity == target) downgradeTimes.push_back(event.simTime);
    }

    // Buffer past the interval's own end -- a confirmed track drops only
    // after 3 missed 4.4 s frames (13.2 s), plus up to one frame of phase
    // between the notch starting and the next look.
    constexpr double kCorrelationBufferS = 18.0;
    int correlatedCount = 0;
    for (const auto& [start, end] : beamIntervals) {
        for (double downgradeTime : downgradeTimes) {
            if (downgradeTime >= start && downgradeTime <= end + kCorrelationBufferS) {
                ++correlatedCount;
                break;
            }
        }
    }

    beamWindowsOut = beamIntervals.size();
    downgradeEventsOut = downgradeTimes.size();
    correlatedOut = correlatedCount;
    return true;
}

// --- Confirm the pulse-Doppler notch filter is what's actually
// dropping tracks during scenario_notch_beam.json's scripted beaming leg,
// not just aspect-angle RCS reduction -- single run at the scenario's own
// authored seed.
TEST(RadarValidation, NotchFilterDropsTracksDuringBeamingLeg) {
    size_t beamWindows = 0, downgradeEvents = 0;
    int correlated = 0;
    std::string error;
    ASSERT_TRUE(RunNotchTrial(std::nullopt, beamWindows, downgradeEvents, correlated, error)) << error;

    EXPECT_GT(beamWindows, 0u);
    EXPECT_GT(correlated, 0) << "beam_windows=" << beamWindows << " downgrade_events=" << downgradeEvents;
}

// --- Track life cycle: detection, confirmation at 3 of 5 frames, and the
// drop once the contact leaves the cone. TargetInFOV starts well inside the
// 60-degree FOV cone and close enough to detect; the Pursuer's own
// straight-and-level flight path carries its nose past TargetInFOV's
// along-track position over the run, growing the aspect angle until it
// exceeds the FOV half-angle, after which the confirmed track drops after
// 3 frames without a look. TargetOutFOV sits directly behind the Pursuer's
// start position for the whole run, as a negative control proving the FOV
// gate actually excludes an off-boresight contact rather than everything
// just happening to detect.
TEST(RadarValidation, AirborneTrackConfirmsThenDropsOutsideTheCone) {
    World world;
    SimContext simContext;

    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load("tests/fixtures/scenarios/scenario_airborne_radar_test.json", world);
    }) << "failed to load scenario_airborne_radar_test.json";
    SimLoop::BeginRun(scenario, simContext, scenario.seed);

    EntityID targetIn = FindByTagName(world, "TargetInFOV").value_or(0);
    EntityID targetOut = FindByTagName(world, "TargetOutFOV").value_or(0);
    ASSERT_NE(targetIn, 0u) << "scenario_airborne_radar_test.json missing TargetInFOV tag";
    ASSERT_NE(targetOut, 0u) << "scenario_airborne_radar_test.json missing TargetOutFOV tag";

    constexpr double kFixedDt = 1.0 / 120.0;
    constexpr double kTotalSimTime = 90.0;  // long enough to confirm, leave the cone and drop
    double simTime = 0.0;
    while (simTime < kTotalSimTime) {
        ControlSystem::Update(world, simContext, kFixedDt, simTime);
        FlightDynamicsSystem::Update(world, kFixedDt);
        IntegrationSystem::Update(world, kFixedDt);
        simTime += kFixedDt;
        AirborneRadarSystem::Update(world, simContext, simTime);
    }

    double detectionAt = -1.0, establishedAt = -1.0, droppedAt = -1.0;
    bool outFovEverAppeared = false;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.entity == targetOut && (event.type == "DETECTION" || event.type == "RADAR_TRACK_ESTABLISHED")) {
            outFovEverAppeared = true;
        }
        if (event.entity != targetIn) continue;
        if (event.type == "DETECTION" && detectionAt < 0.0) detectionAt = event.simTime;
        if (event.type == "RADAR_TRACK_ESTABLISHED" && establishedAt < 0.0) establishedAt = event.simTime;
        if (event.type == "RADAR_TRACK_DROPPED" && droppedAt < 0.0) droppedAt = event.simTime;
    }

    const AirborneRadarComponent& radar = world.Components<AirborneRadarComponent>().begin()->second;
    EXPECT_GE(detectionAt, 0.0);
    // Confirmation takes at least confirmFrames - 1 further frames.
    EXPECT_GE(establishedAt, detectionAt + (radar.confirmFrames - 1) * radar.frameTimeS - 1e-6);
    EXPECT_GT(droppedAt, establishedAt);
    EXPECT_FALSE(outFovEverAppeared);
}

// --- Robustness of the scripted-beaming-defeats-tracking mechanism
// across many seeds: does the scripted beaming leg reliably defeat the
// Pursuer's radar track despite the run-to-run randomness in RCS
// fluctuation, sweep jitter, and GPS/INS noise? Reuses RunNotchTrial's
// single-trial logic across kTrials distinct seeds and requires a
// majority to show at least one correlated downgrade.
// The F-16 type's RWR hears the F-16 type's radar out to exactly the
// one-way link budget's range: the illuminator's peak power and main-lobe
// gain into the RWR's antenna, against its sensitivity. Placed just inside
// and just outside that range, with jitter off.
namespace {

// One RWR sweep: an F-16 radar at (0, radarAltFt) and an F-16 RWR (no
// jitter) rangeM east of it at listenerAltFt. Whether it reads illuminated.
bool RwrIlluminatedAt(double rangeM, double radarAltFt, double listenerAltFt) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    World world;
    SimContext simContext;
    EntityID illuminator = world.CreateEntity();
    world.Components<Transform>().Add(illuminator, Transform{Vec3{0.0, radarAltFt, 0.0}, Vec3{0.0, 0.0, 90.0}});
    world.Components<AirborneRadarComponent>().Add(illuminator, AircraftEntity::MakeAirborneRadar(*f16));
    EntityID listener = world.CreateEntity();
    world.Components<Transform>().Add(listener,
                                      Transform{Vec3{rangeM * 3.280839895, listenerAltFt, 0.0}, Vec3{0.0, 0.0, 270.0}});
    RwrComponent rwr = AircraftEntity::MakeRwr(*f16);
    rwr.signalJitterDb = 0.0;
    world.Components<RwrComponent>().Add(listener, rwr);
    RwrSystem::Update(world, simContext, 0.0);
    return world.Components<RwrComponent>().Get(listener).currentlyIlluminated;
}

}  // namespace

TEST(RadarValidation, RwrHearsTheTypeRadarOutToItsLinkBudgetRange) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*f16);
    RwrComponent rwrTemplate = AircraftEntity::MakeRwr(*f16);
    double lambdaM = WavelengthM(radar.frequencyHz);
    double sensitivityW = std::pow(10.0, (rwrTemplate.sensitivityDbm - 30.0) / 10.0);
    double linkRangeM = lambdaM / (4.0 * 3.14159265358979323846) *
                        std::sqrt(radar.peakPowerW * radar.antennaGain * rwrTemplate.antennaGain / sensitivityW);

    // ~465 nmi for the F-16's values, far beyond its own radar's 50 nmi.
    // Both at 40 000 ft, so the horizon (~490 nmi) is not what limits it.
    const double altFt = 40000.0;
    EXPECT_GT(linkRangeM / 1852.0, 300.0);
    ASSERT_GT(RadarEquation::HorizonRangeM(altFt / 3.280839895, altFt / 3.280839895), 1.01 * linkRangeM);
    EXPECT_TRUE(RwrIlluminatedAt(0.99 * linkRangeM, altFt, altFt));
    EXPECT_FALSE(RwrIlluminatedAt(1.01 * linkRangeM, altFt, altFt));
}

TEST(RadarValidation, RwrDoesNotHearARadarBelowItsHorizon) {
    // Radar at 25 000 ft, listener at 500 ft: the 4/3-Earth line of sight
    // is ~411 km, well inside the ~861 km link budget, so the horizon is
    // what cuts the signal off.
    const double radarAltFt = 25000.0, listenerAltFt = 500.0;
    double horizonM = RadarEquation::HorizonRangeM(radarAltFt / 3.280839895, listenerAltFt / 3.280839895);
    EXPECT_TRUE(RwrIlluminatedAt(0.99 * horizonM, radarAltFt, listenerAltFt));
    EXPECT_FALSE(RwrIlluminatedAt(1.01 * horizonM, radarAltFt, listenerAltFt));
    // Climbing the listener out of the shadow brings the signal back.
    EXPECT_TRUE(RwrIlluminatedAt(1.01 * horizonM, radarAltFt, 5000.0));
}

// A destroyed aircraft's RWR goes dark without logging a "clear": after
// the kill there is no one left to hear it.
TEST(RadarValidation, RwrGoesQuietWhenItsAircraftIsDestroyed) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    World world;
    SimContext simContext;
    EntityID illuminator = world.CreateEntity();
    world.Components<Transform>().Add(illuminator, Transform{Vec3{0.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 90.0}});
    world.Components<AirborneRadarComponent>().Add(illuminator, AircraftEntity::MakeAirborneRadar(*f16));
    EntityID listener = world.CreateEntity();
    world.Components<Transform>().Add(listener, Transform{Vec3{100000.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 270.0}});
    RwrComponent rwr = AircraftEntity::MakeRwr(*f16);
    rwr.signalJitterDb = 0.0;
    world.Components<RwrComponent>().Add(listener, rwr);

    RwrSystem::Update(world, simContext, 0.0);
    ASSERT_TRUE(world.Components<RwrComponent>().Get(listener).currentlyIlluminated);

    world.Components<DestroyedComponent>().Add(listener, DestroyedComponent{});
    RwrSystem::Update(world, simContext, 10.0);
    const RwrComponent& after = world.Components<RwrComponent>().Get(listener);
    EXPECT_FALSE(after.currentlyIlluminated);
    EXPECT_EQ(after.perceivedMode, RwrMode::NONE);
    for (const SimEvent& event : simContext.eventLog.Events()) {
        EXPECT_NE(event.type, "RWR_CLEAR") << "at t=" << event.simTime;
    }
}

// Illumination ending -- here the illuminator is shot down -- clears the
// warning.
TEST(RadarValidation, RwrClearsWhenTheIlluminatorStops) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    World world;
    SimContext simContext;
    EntityID illuminator = world.CreateEntity();
    world.Components<Transform>().Add(illuminator, Transform{Vec3{0.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 90.0}});
    world.Components<AirborneRadarComponent>().Add(illuminator, AircraftEntity::MakeAirborneRadar(*f16));
    EntityID listener = world.CreateEntity();
    world.Components<Transform>().Add(listener, Transform{Vec3{100000.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 270.0}});
    RwrComponent rwr = AircraftEntity::MakeRwr(*f16);
    rwr.signalJitterDb = 0.0;
    world.Components<RwrComponent>().Add(listener, rwr);

    RwrSystem::Update(world, simContext, 0.0);
    ASSERT_EQ(world.Components<RwrComponent>().Get(listener).perceivedMode, RwrMode::SEARCH);

    world.Components<DestroyedComponent>().Add(illuminator, DestroyedComponent{});
    RwrSystem::Update(world, simContext, 1.0);
    EXPECT_EQ(world.Components<RwrComponent>().Get(listener).perceivedMode, RwrMode::NONE);
    const std::vector<SimEvent>& events = simContext.eventLog.Events();
    ASSERT_FALSE(events.empty());
    EXPECT_EQ(events.back().type, "RWR_CLEAR");
    EXPECT_EQ(events.back().simTime, 1.0);
}

TEST(RadarValidation, NotchDefeatsTrackingAcrossSeededRuns) {
    constexpr int kTrials = 15;
    int correlatedTrials = 0;

    for (int trial = 0; trial < kTrials; ++trial) {
        size_t beamWindows = 0, downgradeEvents = 0;
        int correlated = 0;
        std::string error;
        // A different seed per trial (not the scenario's own fixed seed)
        // -- that's the whole point of a many-seeded-runs robustness check.
        ASSERT_TRUE(RunNotchTrial(5000ULL + static_cast<uint64_t>(trial), beamWindows, downgradeEvents, correlated,
                                   error))
            << error;
        if (correlated > 0) ++correlatedTrials;
    }

    double correlatedRate = static_cast<double>(correlatedTrials) / kTrials;
    // >= 0.40: a "reliably, not just occasionally" bar for a stochastic
    // mechanism -- a judgement, not a derived figure.
    EXPECT_GE(correlatedRate, 0.40) << correlatedTrials << "/" << kTrials << " seeds correlated";
}

// Frames: a radar that detects anything it looks at (a huge budget, a
// threshold far below the noise) looks once per 4.4 s frame, confirms at the
// third frame, and -- with the contact moved behind it, out of the cone --
// drops the confirmed track after three frames without a look.
TEST(RadarValidation, OneLookPerFrameConfirmAtThreeDropAfterThree) {
    World world;
    SimContext simContext;
    EntityID owner = world.CreateEntity();
    world.Components<Transform>().Add(owner, Transform{Vec3{0.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 90.0}});
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*AircraftCatalog::Default().Get("f16c_block50"));
    radar.searchThresholdDb = -100.0;  // every look detects
    world.Components<AirborneRadarComponent>().Add(owner, radar);
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{Vec3{60000.0, 25000.0, 0.0}, Vec3{}});
    world.Components<RadarCrossSection>().Add(target, RadarCrossSection{5.0, FluctuationModel::STEADY});
    world.Components<Velocity>().Add(target, Velocity{Vec3{-800.0, 0.0, 0.0}});  // closing, out of the notch

    constexpr double kTickS = SimClock::kFixedDt;
    std::vector<double> lookTimes;
    double simTime = 0.0, lastSeen = -1e18;
    for (int tick = 1; simTime < 40.0; ++tick) {
        simTime = tick * kTickS;
        // Behind the radar from 20 s on: outside the cone, so no more looks.
        if (simTime >= 20.0) world.Components<Transform>().Get(target).pos = Vec3{-60000.0, 25000.0, 0.0};
        AirborneRadarSystem::Update(world, simContext, simTime);
        const AirborneRadarTrack* track = simContext.airborneRadarTracks.Find(owner, target);
        if (track != nullptr && track->lastSeenTime != lastSeen) {
            lastSeen = track->lastSeenTime;
            lookTimes.push_back(lastSeen);
        }
    }

    // Detections only on frame ticks, one frame apart.
    ASSERT_GE(lookTimes.size(), 4u);
    for (size_t i = 1; i < lookTimes.size(); ++i) EXPECT_NEAR(lookTimes[i] - lookTimes[i - 1], 4.4, kTickS + 1e-9);
    double detectionAt = -1.0, establishedAt = -1.0, droppedAt = -1.0;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "DETECTION") detectionAt = event.simTime;
        if (event.type == "RADAR_TRACK_ESTABLISHED") establishedAt = event.simTime;
        if (event.type == "RADAR_TRACK_DROPPED") droppedAt = event.simTime;
    }
    EXPECT_NEAR(detectionAt, lookTimes.front(), 1e-9);
    EXPECT_NEAR(establishedAt, lookTimes[2], 1e-9);  // the third frame
    // Three frames after the last look, the confirmed track is gone.
    EXPECT_NEAR(droppedAt - lookTimes.back(), 3 * 4.4, 3 * kTickS + 1e-9);
    const AirborneRadarTrack* track = simContext.airborneRadarTracks.Find(owner, target);
    ASSERT_NE(track, nullptr);
    EXPECT_EQ(track->state, RadarTrackState::NONE);
    EXPECT_FALSE(track->estimate.valid);
}

// Single-target track, on the radar alone: once locked, the locked target
// is looked at every STT frame and nothing else is looked at, so a second
// confirmed track coasts and drops on the scan-frame rule. A lock whose
// target leaves the cone breaks after stt_drop_frames STT frames.
TEST(RadarValidation, SttLooksOnlyAtTheLockedTargetAndBreaksOnMissedFrames) {
    World world;
    SimContext simContext;
    EntityID owner = world.CreateEntity();
    world.Components<Transform>().Add(owner, Transform{Vec3{0.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 90.0}});
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*AircraftCatalog::Default().Get("f16c_block50"));
    radar.searchThresholdDb = -100.0;  // every look detects
    world.Components<AirborneRadarComponent>().Add(owner, radar);
    auto addTarget = [&](double z) {
        EntityID target = world.CreateEntity();
        world.Components<Transform>().Add(target, Transform{Vec3{60000.0, 25000.0, z}, Vec3{}});
        world.Components<RadarCrossSection>().Add(target, RadarCrossSection{5.0, FluctuationModel::STEADY});
        world.Components<Velocity>().Add(target, Velocity{Vec3{-800.0, 0.0, 0.0}});  // closing, out of the notch
        return target;
    };
    EntityID locked = addTarget(0.0);
    EntityID other = addTarget(5000.0);
    AirborneRadarComponent& liveRadar = world.Components<AirborneRadarComponent>().Get(owner);

    constexpr double kTickS = SimClock::kFixedDt;
    constexpr double kLockAt = 10.0, kLeaveAt = 30.0;
    std::vector<double> lockedLooks;
    double simTime = 0.0, lastSeen = -1e18, otherLastSeen = -1.0;
    bool otherSeenUnderLock = false;
    for (int tick = 1; simTime < 32.0; ++tick) {
        simTime = tick * kTickS;
        if (simTime >= kLockAt && simTime - kTickS < kLockAt) {
            // What fire control does (LaunchSystem): both tracks confirmed
            // by the frame at 8.8 s.
            ASSERT_EQ(simContext.airborneRadarTracks.Find(owner, locked)->state, RadarTrackState::CONFIRMED);
            ASSERT_EQ(simContext.airborneRadarTracks.Find(owner, other)->state, RadarTrackState::CONFIRMED);
            otherLastSeen = simContext.airborneRadarTracks.Find(owner, other)->lastSeenTime;
            liveRadar.sttTarget = locked;
            liveRadar.lastSttFrameTime = -1e18;
        }
        if (simTime >= kLeaveAt) world.Components<Transform>().Get(locked).pos = Vec3{-60000.0, 25000.0, 0.0};
        AirborneRadarSystem::Update(world, simContext, simTime);
        // While locked, the other target is never looked at.
        if (liveRadar.sttTarget == locked &&
            simContext.airborneRadarTracks.Find(owner, other)->lastSeenTime != otherLastSeen)
            otherSeenUnderLock = true;
        const AirborneRadarTrack* track = simContext.airborneRadarTracks.Find(owner, locked);
        if (simTime >= kLockAt && track->lastSeenTime != lastSeen) {
            lastSeen = track->lastSeenTime;
            lockedLooks.push_back(lastSeen);
        }
    }

    // One look per STT frame on the locked target, from the lock to the
    // moment it leaves the cone.
    double sttFrameS = radar.sttFrameTimeS;
    ASSERT_GE(lockedLooks.size(), 150u);
    for (size_t i = 1; i < lockedLooks.size(); ++i)
        EXPECT_NEAR(lockedLooks[i] - lockedLooks[i - 1], sttFrameS, kTickS + 1e-9);
    EXPECT_LT(lockedLooks.back(), kLeaveAt);

    // The other target was not looked at under the lock, and dropped three
    // scan frames after its last look.
    double otherDroppedAt = -1.0, lockLostAt = -1.0;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "RADAR_TRACK_DROPPED" && event.entity == other) otherDroppedAt = event.simTime;
        if (event.type == "RADAR_LOCK_LOST") {
            EXPECT_EQ(event.entity, locked);
            lockLostAt = event.simTime;
        }
    }
    EXPECT_FALSE(otherSeenUnderLock);
    EXPECT_NEAR(otherDroppedAt - otherLastSeen, 3 * radar.frameTimeS, 3 * kTickS + 1e-9);

    // stt_drop_frames missed STT frames after the last look: lock and track
    // gone, radar back to scanning.
    EXPECT_NEAR(lockLostAt - lockedLooks.back(), radar.sttDropFrames * sttFrameS, radar.sttDropFrames * kTickS + 1e-9);
    EXPECT_EQ(liveRadar.sttTarget, kNoEntity);
    EXPECT_EQ(simContext.airborneRadarTracks.Find(owner, locked)->state, RadarTrackState::NONE);
}

// Fire control's release, with two SARH missiles from one launcher on one
// target: the lock holds while either still flies on it, and the release
// re-counts the track's missed STT frames as scan frames, so the scan-frame
// drop rule does not read 0.1 s frames as 4.4 s ones.
// A lock on something the radar cannot look at (an entity with no cross
// section, or one that is not there) can never break by missed frames, and
// while it stands the radar is blind to everything else. It is released at
// once.
TEST(RadarValidation, ALockOnSomethingThatCannotBeSeenIsReleased) {
    World world;
    SimContext simContext;
    EntityID owner = world.CreateEntity();
    world.Components<Transform>().Add(owner, Transform{Vec3{0.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 90.0}});
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*AircraftCatalog::Default().Get("f16c_block50"));
    radar.searchThresholdDb = -100.0;  // every look detects
    world.Components<AirborneRadarComponent>().Add(owner, radar);
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{Vec3{60000.0, 25000.0, 0.0}, Vec3{}});
    world.Components<RadarCrossSection>().Add(target, RadarCrossSection{5.0, FluctuationModel::STEADY});
    world.Components<Velocity>().Add(target, Velocity{Vec3{-800.0, 0.0, 0.0}});
    EntityID ghost = world.CreateEntity();  // a Transform but no cross section: never a radar target
    world.Components<Transform>().Add(ghost, Transform{Vec3{30000.0, 25000.0, 0.0}, Vec3{}});
    AirborneRadarComponent& liveRadar = world.Components<AirborneRadarComponent>().Get(owner);
    liveRadar.sttTarget = ghost;
    liveRadar.lastSttFrameTime = -1e18;

    constexpr double kTickS = SimClock::kFixedDt;
    double simTime = 0.0;
    for (int tick = 1; simTime < 20.0; ++tick) {
        simTime = tick * kTickS;
        AirborneRadarSystem::Update(world, simContext, simTime);
    }

    EXPECT_EQ(liveRadar.sttTarget, kNoEntity);
    bool lockLost = false;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "RADAR_LOCK_LOST" && event.entity == ghost) lockLost = true;
    }
    EXPECT_TRUE(lockLost);
    // And the radar is looking at the real target again.
    const AirborneRadarTrack* track = simContext.airborneRadarTracks.Find(owner, target);
    ASSERT_NE(track, nullptr);
    EXPECT_EQ(track->state, RadarTrackState::CONFIRMED);
}

// A destroyed aircraft's radar goes quiet, as its RWR does: it takes the
// same no-return path a destroyed target does, so its tracks drop by the
// normal rule and the lock breaks, rather than the wreck going on scanning,
// keeping a confirmed track alive and relaying it to a missile in flight.
TEST(RadarValidation, ADestroyedAircraftsRadarStopsTrackingAndLosesItsLock) {
    World world;
    SimContext simContext;
    EntityID owner = world.CreateEntity();
    world.Components<Transform>().Add(owner, Transform{Vec3{0.0, 25000.0, 0.0}, Vec3{0.0, 0.0, 90.0}});
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*AircraftCatalog::Default().Get("f16c_block50"));
    radar.searchThresholdDb = -100.0;  // every look detects
    world.Components<AirborneRadarComponent>().Add(owner, radar);
    EntityID target = world.CreateEntity();
    world.Components<Transform>().Add(target, Transform{Vec3{60000.0, 25000.0, 0.0}, Vec3{}});
    world.Components<RadarCrossSection>().Add(target, RadarCrossSection{5.0, FluctuationModel::STEADY});
    world.Components<Velocity>().Add(target, Velocity{Vec3{-800.0, 0.0, 0.0}});  // closing, out of the notch
    AirborneRadarComponent& liveRadar = world.Components<AirborneRadarComponent>().Get(owner);

    constexpr double kTickS = SimClock::kFixedDt;
    constexpr double kLockAt = 10.0, kDestroyedAt = 14.0;
    double simTime = 0.0, lastSeenWhenDestroyed = -1.0;
    for (int tick = 1; simTime < 24.0; ++tick) {
        simTime = tick * kTickS;
        if (simTime >= kLockAt && simTime - kTickS < kLockAt) {
            ASSERT_EQ(simContext.airborneRadarTracks.Find(owner, target)->state, RadarTrackState::CONFIRMED);
            liveRadar.sttTarget = target;
            liveRadar.lastSttFrameTime = -1e18;
        }
        if (simTime >= kDestroyedAt && simTime - kTickS < kDestroyedAt) {
            ASSERT_EQ(liveRadar.sttTarget, target) << "the lock should still hold before the owner is destroyed";
            world.Components<DestroyedComponent>().Add(owner, DestroyedComponent{});
            lastSeenWhenDestroyed = simContext.airborneRadarTracks.Find(owner, target)->lastSeenTime;
        }
        AirborneRadarSystem::Update(world, simContext, simTime);
    }

    const AirborneRadarTrack* track = simContext.airborneRadarTracks.Find(owner, target);
    ASSERT_NE(track, nullptr);
    EXPECT_LE(track->lastSeenTime, lastSeenWhenDestroyed + 1e-9) << "a destroyed radar must not make new detections";
    EXPECT_EQ(track->state, RadarTrackState::NONE);
    EXPECT_FALSE(track->estimate.valid);
    EXPECT_EQ(liveRadar.sttTarget, kNoEntity);
    bool lockLost = false;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "RADAR_LOCK_LOST" && event.simTime > kDestroyedAt) lockLost = true;
    }
    EXPECT_TRUE(lockLost);
}

TEST(RadarValidation, LockReleasesOnlyWhenNoMissileNeedsItAndRecountsMissedFrames) {
    World world;
    SimContext simContext;
    ScenarioData scenario;
    EntityID launcher = world.CreateEntity();
    EntityID target = world.CreateEntity();
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*AircraftCatalog::Default().Get("f16c_block50"));
    radar.sttTarget = target;
    world.Components<AirborneRadarComponent>().Add(launcher, radar);
    AirborneRadarTrack& track = simContext.airborneRadarTracks.FindOrCreate(launcher, target);
    track.owner = launcher;
    track.target = target;
    track.state = RadarTrackState::CONFIRMED;
    track.missedFrames = 7;  // 0.7 s of missed STT frames
    auto addMissile = [&] {
        EntityID missile = world.CreateEntity();
        GuidanceComponent guidance;
        guidance.launcher = launcher;
        guidance.target = target;
        guidance.seekerMode = SeekerMode::SARH;
        guidance.launched = true;
        guidance.active = true;
        world.Components<GuidanceComponent>().Add(missile, guidance);
        return missile;
    };
    EntityID first = addMissile();
    EntityID second = addMissile();
    const AirborneRadarComponent& liveRadar = world.Components<AirborneRadarComponent>().Get(launcher);

    world.Components<GuidanceComponent>().Get(first).active = false;  // resolved
    LaunchSystem::Update(world, simContext, scenario, 1.0);
    EXPECT_EQ(liveRadar.sttTarget, target);
    EXPECT_EQ(simContext.airborneRadarTracks.Find(launcher, target)->missedFrames, 7);

    world.Components<GuidanceComponent>().Get(second).active = false;
    LaunchSystem::Update(world, simContext, scenario, 2.0);
    EXPECT_EQ(liveRadar.sttTarget, kNoEntity);
    EXPECT_EQ(simContext.airborneRadarTracks.Find(launcher, target)->missedFrames,
              static_cast<int>(std::ceil(7 * radar.sttFrameTimeS / radar.frameTimeS)));
    int released = 0;
    for (const SimEvent& event : simContext.eventLog.Events()) released += event.type == "RADAR_LOCK_RELEASED";
    EXPECT_EQ(released, 1);
}

// launch_mode on scenario_basic: STT locks the target when the TWS gate
// opens, fires from the lock, holds it until the MIDCOURSE_ARH seeker goes
// active, and the target's RWR reads LOCK; TWS never locks. A SARH missile
// fires and flies on a lock whatever launch_mode says. The target's RWR
// warns of the missile once its seeker goes active -- never for SARH, whose
// seeker only listens.
TEST(RadarValidation, LaunchModeSttLocksBeforeFiringAndTheRwrReadsTheLock) {
    struct Run {
        std::vector<std::pair<double, std::string>> events;  // (time, type) of the ones collected below
        std::string launchData;
        bool lockedThroughFlight = true;  // the launcher held the lock every tick after launch
        double Time(const std::string& type) const {
            for (const auto& [t, name] : events)
                if (name == type) return t;
            return -1.0;
        }
    };
    auto fly = [](const char* launchMode, const char* seekerMode, double flightS) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        json["missile_doctrine"]["launch_mode"] = launchMode;
        json["missile_doctrine"]["seeker"]["mode"] = seekerMode;
        // The target's RWR, as scenario_notch_beam carries one.
        for (nlohmann::json& entity : json["entities"])
            if (entity["name"] == "Target") entity["components"]["evasion"] = {{"rwr", nlohmann::json::object()}};
        World world;
        SimContext simContext;
        ScenarioData scenario = ScenarioLoader::LoadFromJson(json, world);
        SimLoop::BeginRun(scenario, simContext, scenario.seed);
        LaunchSystem::InitFromTags(world, scenario);
        EntityID pursuer = *FindByTagName(world, "Pursuer");
        EntityID target = *FindByTagName(world, "Target");
        Run run;
        double simTime = 0.0, launchAt = -1.0;
        while (simTime < 300.0 && !simContext.outcome.resolved) {
            SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
            if (simContext.outcome.launched && launchAt < 0.0) launchAt = simTime;
            if (launchAt >= 0.0 && world.Components<AirborneRadarComponent>().Get(pursuer).sttTarget != target)
                run.lockedThroughFlight = false;
            if (launchAt >= 0.0 && simTime - launchAt >= flightS) break;
        }
        for (const SimEvent& event : simContext.eventLog.Events()) {
            for (const char* type : {"RADAR_LOCK", "RADAR_LOCK_RELEASED", "RADAR_LOCK_LOST", "LAUNCH_ZONE_ENTERED",
                                     "LAUNCH", "SEEKER_ACTIVE", "RWR_LOCK", "RWR_LOCK_CLEAR", "RWR_MISSILE"}) {
                if (event.type == type) run.events.emplace_back(event.simTime, event.type);
            }
            if (event.type == "LAUNCH") run.launchData = event.data;
        }
        return run;
    };

    Run tws = fly("TWS", "MIDCOURSE_ARH", 300.0);
    ASSERT_GE(tws.Time("LAUNCH"), 0.0);
    EXPECT_NE(tws.launchData.find("gate=tws"), std::string::npos);
    EXPECT_LT(tws.Time("RADAR_LOCK"), 0.0);
    EXPECT_LT(tws.Time("RWR_LOCK"), 0.0);
    // A TWS shot gives the target no warning until the seeker goes active,
    // then the RWR hears it within one of its 0.5 s sweeps.
    ASSERT_GE(tws.Time("SEEKER_ACTIVE"), 0.0);
    ASSERT_GE(tws.Time("RWR_MISSILE"), 0.0);
    EXPECT_GE(tws.Time("RWR_MISSILE"), tws.Time("SEEKER_ACTIVE"));
    EXPECT_LE(tws.Time("RWR_MISSILE") - tws.Time("SEEKER_ACTIVE"), 0.5 + 1e-9);

    Run stt = fly("STT", "MIDCOURSE_ARH", 300.0);
    double delayS = AircraftCatalog::Default().Get("f16c_block50")->radar.fireControlDelayS;
    ASSERT_GE(stt.Time("LAUNCH"), 0.0);
    EXPECT_NE(stt.launchData.find("gate=stt"), std::string::npos);
    // Locked the tick the gate opened; fired the fire-control delay later.
    EXPECT_NEAR(stt.Time("RADAR_LOCK"), stt.Time("LAUNCH_ZONE_ENTERED"), 1e-9);
    EXPECT_NEAR(stt.Time("LAUNCH") - stt.Time("RADAR_LOCK"), delayS, 2.0 * SimClock::kFixedDt);
    // The target's RWR hears the lock within one of its sweeps.
    ASSERT_GE(stt.Time("RWR_LOCK"), 0.0);
    EXPECT_LE(stt.Time("RWR_LOCK") - stt.Time("RADAR_LOCK"), 0.5 + 1e-9);
    // Released on the tick the seeker goes active (GuidanceSystem stamps its
    // event before the tick's time step, LaunchSystem after), and the RWR
    // clears.
    ASSERT_GE(stt.Time("SEEKER_ACTIVE"), 0.0);
    EXPECT_NEAR(stt.Time("RADAR_LOCK_RELEASED") - stt.Time("SEEKER_ACTIVE"), SimClock::kFixedDt, 1e-9);
    EXPECT_GE(stt.Time("RWR_LOCK_CLEAR"), stt.Time("RADAR_LOCK_RELEASED"));

    Run sarh = fly("TWS", "SARH", 5.0);
    ASSERT_GE(sarh.Time("LAUNCH"), 0.0);
    EXPECT_NE(sarh.launchData.find("gate=stt"), std::string::npos);
    EXPECT_TRUE(sarh.lockedThroughFlight);
    EXPECT_LT(sarh.Time("RADAR_LOCK_RELEASED"), 0.0);
    EXPECT_LT(sarh.Time("RWR_MISSILE"), 0.0);
}

// The loft, on scenario_basic: a long MIDCOURSE_ARH shot climbs well above
// the launch altitude, pushes over (LOFT_END) before its seeker goes active,
// and comes down faster than the same shot flown straight. With the knob off,
// for a SARH missile, or inside the type's start range it never lofts.
TEST(RadarValidation, LoftClimbsOnALongShotAndPushesOverBeforeTheSeeker) {
    struct Shot {
        double launchAltFt = 0.0, apexFt = 0.0, launchRangeFt = 0.0;
        double loftEndAt = -1.0, seekerActiveAt = -1.0;
        double speedAtSeekerFps = 0.0;  // the missile's speed when its seeker went active
    };
    auto fly = [](bool loft, const char* seekerMode, double fraction) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        json["missile_doctrine"]["loft"] = loft;
        json["missile_doctrine"]["seeker"]["mode"] = seekerMode;
        json["missile_doctrine"]["launch_range_fraction"] = fraction;
        World world;
        SimContext simContext;
        ScenarioData scenario = ScenarioLoader::LoadFromJson(json, world);
        SimLoop::BeginRun(scenario, simContext, scenario.seed);
        LaunchSystem::InitFromTags(world, scenario);
        EntityID missile = world.Components<GuidanceComponent>().begin()->first;
        Shot shot;
        double simTime = 0.0;
        while (simTime < 300.0 && !simContext.outcome.resolved) {
            SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
            const Transform* transform = world.Components<Transform>().TryGet(missile);
            if (transform == nullptr) continue;
            if (shot.apexFt == 0.0) shot.launchAltFt = transform->pos.y();
            shot.apexFt = std::max(shot.apexFt, transform->pos.y());
            if (shot.seekerActiveAt < 0.0 && world.Components<GuidanceComponent>().Get(missile).seekerActive) {
                shot.seekerActiveAt = simTime;
                shot.speedAtSeekerFps = world.Components<KinematicsComponent>().Get(missile).speed;
            }
        }
        for (const SimEvent& event : simContext.eventLog.Events())
            if (event.type == "LOFT_END" && shot.loftEndAt < 0.0) shot.loftEndAt = event.simTime;
        shot.launchRangeFt = simContext.outcome.launchRangeFt;
        std::printf("loft %d %s: launch %.1f nmi, rise %.0f ft, loft end %.1f s, seeker %.1f s at %.0f ft/s\n", loft,
                    seekerMode, shot.launchRangeFt / Units::kFeetPerNmi, shot.apexFt - shot.launchAltFt,
                    shot.loftEndAt, shot.seekerActiveAt, shot.speedAtSeekerFps);
        return shot;
    };

    const MissileLoft& loft = MissileCatalog::Default().Get("aim120")->loft;
    Shot lofted = fly(true, "MIDCOURSE_ARH", 1.0);
    Shot straight = fly(false, "MIDCOURSE_ARH", 1.0);
    ASSERT_GT(lofted.launchRangeFt / Units::kFeetPerNmi, loft.startRangeNmi + 5.0);  // a long shot
    EXPECT_GT(lofted.apexFt - lofted.launchAltFt, 5000.0);
    ASSERT_GE(lofted.loftEndAt, 0.0);
    ASSERT_GE(lofted.seekerActiveAt, 0.0);
    EXPECT_LT(lofted.loftEndAt, lofted.seekerActiveAt);
    ASSERT_GE(straight.seekerActiveAt, 0.0);
    EXPECT_GT(lofted.speedAtSeekerFps, straight.speedAtSeekerFps);
    // The straight shot drifts up a little on its own way to the target;
    // the lofted one climbs several times as high.
    EXPECT_GT(lofted.apexFt - lofted.launchAltFt, 3.0 * (straight.apexFt - straight.launchAltFt));
    EXPECT_LT(straight.loftEndAt, 0.0);

    // SARH homes on the lock from the rail: it never lofts.
    EXPECT_LT(fly(true, "SARH", 1.0).loftEndAt, 0.0);
    // Inside the start range there is nothing to gain: no loft.
    Shot close = fly(true, "MIDCOURSE_ARH", 0.15);
    ASSERT_GT(close.launchRangeFt, 0.0);
    ASSERT_LT(close.launchRangeFt / Units::kFeetPerNmi, loft.startRangeNmi);
    EXPECT_LT(close.loftEndAt, 0.0);
}

// The launch gate, on scenario_basic: the missile fires the radar's
// fire-control delay after the gate opens, and the gate opens on the radar
// track's estimated range inside Rmax x launch_range_fraction -- so a lower
// fraction fires later, closer in.
TEST(RadarValidation, LaunchWaitsForTheZoneFractionAndTheFireControlDelay) {
    struct Shot {
        double trackAt = -1.0, gateAt = -1.0, launchAt = -1.0, estRangeFt = 0.0, rmaxFt = 0.0, launchRangeFt = 0.0;
    };
    auto fly = [](double fraction, bool withZone = true) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        json["missile_doctrine"]["launch_range_fraction"] = fraction;
        World world;
        SimContext simContext;
        ScenarioData scenario = ScenarioLoader::LoadFromJson(json, world);
        if (!withZone) {
            auto noZone = std::make_shared<MissileType>(*scenario.missileType);
            noZone->launchZone = nullptr;
            scenario.missileType = noZone;
        }
        SimLoop::BeginRun(scenario, simContext, scenario.seed);
        LaunchSystem::InitFromTags(world, scenario);
        double simTime = 0.0;
        while (!simContext.outcome.launched && simTime < 300.0) {
            SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
        }
        Shot shot;
        for (const SimEvent& event : simContext.eventLog.Events()) {
            if (event.type == "LAUNCH_ZONE_ENTERED" && shot.gateAt < 0.0) {
                shot.gateAt = event.simTime;
                shot.estRangeFt = std::stod(event.data.substr(event.data.find("est_range_ft=") + 13));
                shot.rmaxFt = std::stod(event.data.substr(event.data.find("rmax_ft=") + 8));
            }
            if (event.type == "RADAR_TRACK_ESTABLISHED" && shot.trackAt < 0.0) shot.trackAt = event.simTime;
            if (event.type == "LAUNCH") shot.launchAt = event.simTime;
        }
        shot.launchRangeFt = simContext.outcome.launchRangeFt;
        return shot;
    };
    double delayS = AircraftCatalog::Default().Get("f16c_block50")->radar.fireControlDelayS;
    Shot full = fly(1.0);
    Shot half = fly(0.5);
    for (const Shot& shot : {full, half}) {
        ASSERT_GE(shot.gateAt, 0.0);
        ASSERT_GE(shot.launchAt, 0.0);
        EXPECT_NEAR(shot.launchAt - shot.gateAt, delayS, 2.0 * SimClock::kFixedDt);
    }
    EXPECT_LE(full.estRangeFt, full.rmaxFt);
    EXPECT_LE(half.estRangeFt, 0.5 * half.rmaxFt + 1.0);
    EXPECT_GT(half.launchAt, full.launchAt);
    EXPECT_LT(half.launchRangeFt, full.launchRangeFt);

    // A missile type with no launch zone is gated on the confirmed track
    // alone: the gate opens the tick the track confirms.
    // At half the zone this geometry waits for the range to close; with no
    // zone it does not.
    Shot noZone = fly(0.5, false);
    ASSERT_GE(noZone.trackAt, 0.0);
    EXPECT_NEAR(noZone.gateAt, noZone.trackAt, 2.0 * SimClock::kFixedDt);
    EXPECT_LT(noZone.gateAt, half.gateAt);
}

// The outcome's first-detection time is when the launcher's radar first saw
// ITS target. A radar on the Target that sees the Pursuer earlier must not
// move it: the Target's radar is given a 300 nmi reference range and the
// Pursuer's 20 nmi, so the Target picks the Pursuer up long before the
// Pursuer's own radar reaches the Target.
TEST(RadarValidation, FirstDetectionTimeIsTheLaunchersDetectionOfItsTarget) {
    nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    for (nlohmann::json& entity : json["entities"])
        entity["components"]["airborne_radar"] = {{"reference_range_nmi", entity["name"] == "Target" ? 300.0 : 20.0}};
    World world;
    SimContext simContext;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(json, world);
    SimLoop::BeginRun(scenario, simContext, scenario.seed);
    LaunchSystem::InitFromTags(world, scenario);
    const EntityID pursuer = *FindByTagName(world, "Pursuer");
    const EntityID target = *FindByTagName(world, "Target");

    auto firstDetectionOf = [&](EntityID seen) {
        for (const SimEvent& event : simContext.eventLog.Events())
            if (event.type == "DETECTION" && event.entity == seen) return event.simTime;
        return -1.0;
    };
    double simTime = 0.0;
    while (simTime < 900.0 && firstDetectionOf(target) < 0.0) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
    }

    const double pursuerSeenAt = firstDetectionOf(pursuer);  // by the Target's radar
    const double targetSeenAt = firstDetectionOf(target);    // by the Pursuer's: the launcher's
    // The rig is only meaningful if the Target's radar saw the Pursuer well first.
    ASSERT_GE(pursuerSeenAt, 0.0);
    ASSERT_GE(targetSeenAt, 0.0);
    ASSERT_LT(pursuerSeenAt, targetSeenAt - 5.0);
    EXPECT_DOUBLE_EQ(simContext.outcome.firstDetectionSimTime, targetSeenAt);
}

// A scene with no missile has no launcher, so the radar-only teaching scenario
// still reports when its radar first saw the target.
TEST(RadarValidation, FirstDetectionTimeStillReportedWithoutAMissile) {
    World world;
    SimContext simContext;
    ScenarioData scenario = ScenarioLoader::Load("scenarios/1_core_02_radar_track.json", world);
    SimLoop::BeginRun(scenario, simContext, scenario.seed);
    double simTime = 0.0;
    while (simTime < 300.0 && simContext.outcome.firstDetectionSimTime < 0.0) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
    }
    double firstEvent = -1.0;
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (event.type == "DETECTION") { firstEvent = event.simTime; break; }
    }
    ASSERT_GE(firstEvent, 0.0) << "the radar never detected the target";
    EXPECT_DOUBLE_EQ(simContext.outcome.firstDetectionSimTime, firstEvent);
}

TEST(RadarValidation, LaunchSystemNeedsAMissileType) {
    World world;
    ScenarioData scenario;
    EXPECT_THROW(LaunchSystem::Init(world, world.CreateEntity(), world.CreateEntity(), scenario), std::logic_error);
}

}  // namespace
