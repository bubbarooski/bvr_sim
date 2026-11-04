#include "ParamRegistry.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "components/AirborneRadarComponent.h"
#include "components/AutopilotComponent.h"
#include "components/GLimitComponent.h"
#include "components/RadarCrossSection.h"
#include "components/RwrComponent.h"
#include "components/SpawnPose.h"
#include "ecs/ComponentStore.h"
#include "util/Units.h"

namespace ParamRegistry {

namespace {

using Units::kFeetPerMeter;
using Units::kFeetSqPerMeterSq;
using Units::kLbfPerNewton;
using Units::kKgPerSlug;

using Owner = ParamOwner;
using Timing = ParamTiming;
using Type = ParamType;

// --- Shorthand constructors -------------------------------------------
// The table is long enough that the entries have to read as data, not as
// aggregate-initialisation noise.

ParamDef Dbl(const char* pointer, const char* group, const char* label, const char* unit, Owner owner,
             Timing timing, double lo, double hi, double step, JsonScale scale,
             std::function<double*(ScenarioData&, World&, EntityID)> access) {
    ParamDef p;
    p.pointer = pointer;
    p.group = group;
    p.label = label;
    p.unit = unit;
    p.owner = owner;
    p.timing = timing;
    p.type = Type::Double;
    p.minValue = lo;
    p.maxValue = hi;
    p.dragStep = step;
    p.jsonScale = scale;
    p.dbl = std::move(access);
    return p;
}

ParamDef Flag(const char* pointer, const char* group, const char* label, Owner owner, Timing timing,
              std::function<bool*(ScenarioData&, World&, EntityID)> access) {
    ParamDef p;
    p.pointer = pointer;
    p.group = group;
    p.label = label;
    p.unit = "";
    p.owner = owner;
    p.timing = timing;
    p.type = Type::Bool;
    p.flag = std::move(access);
    return p;
}

ParamDef Enum(const char* pointer, const char* group, const char* label, Owner owner, Timing timing,
              EnumAccess access) {
    ParamDef p;
    p.pointer = pointer;
    p.group = group;
    p.label = label;
    p.unit = "";
    p.owner = owner;
    p.timing = timing;
    p.type = Type::Enum;
    p.enumAccess = std::move(access);
    return p;
}

// --- Component accessors ----------------------------------------------
// Every entity-scoped entry goes through one of these, so a missing
// component yields nullptr and the caller skips the row rather than
// faulting. An entity legitimately may not have every component.

template <typename T>
T* Comp(World& world, EntityID entity) {
    return world.Components<T>().TryGet(entity);
}

// Member-pointer helper: turns "this field of this component" into the
// accessor signature, with the has-component check folded in.
template <typename T, typename M>
std::function<M*(ScenarioData&, World&, EntityID)> Field(M T::*member) {
    return [member](ScenarioData&, World& world, EntityID entity) -> M* {
        T* component = Comp<T>(world, entity);
        return component ? &(component->*member) : nullptr;
    };
}

// Spawn position of an aircraft. Returns nullptr for entities with no
// authored start (the missile).
std::function<double*(ScenarioData&, World&, EntityID)> SpawnField(int axis) {
    return [axis](ScenarioData&, World& world, EntityID entity) -> double* {
        SpawnPose* spawn = Comp<SpawnPose>(world, entity);
        return spawn ? &spawn->pos[axis] : nullptr;
    };
}

void BuildDoctrine(std::vector<ParamDef>& out) {
    const char* kCore = "Missile / Core";
    out.push_back(Dbl("/missile_doctrine/kill_radius_m", kCore, "Kill radius", "m", Owner::Doctrine, Timing::Live,
                      0.0, 200.0, 0.5, kFeetPerMeter,
                      [](ScenarioData& s, World&, EntityID) { return &s.killRadiusFt; }));
    out.push_back(Dbl("/missile_doctrine/max_range_m", kCore, "Max range", "m", Owner::Doctrine, Timing::Live, 0.0,
                      500000.0, 1000.0, kFeetPerMeter,
                      [](ScenarioData& s, World&, EntityID) { return &s.maxRangeFt; }));
    out.push_back(Dbl("/missile_doctrine/launch_range_fraction", kCore, "Launch range fraction", "",
                      Owner::Doctrine, Timing::Live, 0.05, 1.0, 0.01, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.launchRangeFraction; }));
    out.push_back(Enum("/missile_doctrine/launch_mode", kCore, "Launch mode", Owner::Doctrine, Timing::NextLaunch,
                       EnumAccess{{"TWS", "STT"},
                                  [](ScenarioData& s, World&, EntityID) { return static_cast<int>(s.missileLaunchMode); },
                                  [](ScenarioData& s, World&, EntityID, int v) {
                                      s.missileLaunchMode = static_cast<LaunchMode>(v);
                                  }}));
    out.push_back(Flag("/missile_doctrine/loft", kCore, "Loft", Owner::Doctrine, Timing::NextLaunch,
                       [](ScenarioData& s, World&, EntityID) { return &s.missileLoft; }));
    out.push_back(Dbl("/missile_doctrine/arming_range_m", kCore, "Arming range", "m", Owner::Doctrine,
                      Timing::NextLaunch, 0.0, 1000.0, 5.0, kFeetPerMeter,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileArmingRangeFt; }));
    out.push_back(Dbl("/missile_doctrine/falloff_width_m", kCore, "Pk falloff width", "m", Owner::Doctrine,
                      Timing::Live, 0.1, 100.0, 0.5, kFeetPerMeter,
                      [](ScenarioData& s, World&, EntityID) { return &s.falloffWidthFt; }));
    out.push_back(Dbl("/missile_doctrine/reference_area_m2", kCore, "Reference area", "m2", Owner::Doctrine,
                      Timing::NextLaunch, 0.001, 1.0, 0.005, kFeetSqPerMeterSq,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileReferenceAreaFt2; }));

    // The guidance law's own gain gets its own heading rather than sitting
    // among the core geometry values. It is the one knob that changes how
    // the missile flies rather than what it is made of, and it is the field
    // most often swept, so it should be findable without reading the Core
    // list top to bottom.
    const char* kPn = "Missile / Proportional Navigation";
    out.push_back(Dbl("/missile_doctrine/nav_gain", kPn, "Nav gain (N)", "", Owner::Doctrine, Timing::NextLaunch,
                      0.5, 10.0, 0.05, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileNavGain; }));

    const char* kAutopilot = "Missile / Autopilot";
    out.push_back(Dbl("/missile_doctrine/autopilot/launch_turn_exit_deg", kAutopilot, "Launch turn exit", "deg",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 90.0, 1.0, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileLaunchTurnExitDeg; }));

    const char* kSelfDestruct = "Missile / Self-destruct";
    out.push_back(Dbl("/missile_doctrine/self_destruct/min_available_g", kSelfDestruct, "Min available G", "G",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 29.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSelfDestructMinAvailableG; }));
    out.push_back(Dbl("/missile_doctrine/self_destruct/max_time_of_flight_s", kSelfDestruct, "Max time of flight",
                      "s", Owner::Doctrine, Timing::NextLaunch, 0.0, 600.0, 1.0, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSelfDestructMaxTimeOfFlightS; }));

    const char* kThrust = "Missile / Thrust";
    out.push_back(Dbl("/missile_doctrine/thrust_profile/boost_duration_s", kThrust, "Boost duration", "s",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 60.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileBoostDurationS; }));
    out.push_back(Dbl("/missile_doctrine/thrust_profile/boost_thrust_n", kThrust, "Boost thrust", "N",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 200000.0, 250.0, kLbfPerNewton,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileBoostThrustLbf; }));
    out.push_back(Dbl("/missile_doctrine/thrust_profile/sustain_duration_s", kThrust, "Sustain duration", "s",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 120.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSustainDurationS; }));
    out.push_back(Dbl("/missile_doctrine/thrust_profile/sustain_thrust_n", kThrust, "Sustain thrust", "N",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 200000.0, 250.0, kLbfPerNewton,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSustainThrustLbf; }));
    // The mass split belongs with the thrust profile rather than in its
    // own group: together they determine specific impulse, and editing
    // either in isolation is how you end up with an impossible motor.
    out.push_back(Dbl("/missile_doctrine/thrust_profile/dry_mass_kg", kThrust, "Dry mass", "kg", Owner::Doctrine,
                      Timing::NextLaunch, 1.0, 1000.0, 1.0, JsonScale::DividedBy(kKgPerSlug),
                      [](ScenarioData& s, World&, EntityID) { return &s.missileDryMassSlug; }));
    out.push_back(Dbl("/missile_doctrine/thrust_profile/propellant_mass_kg", kThrust, "Propellant mass", "kg",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 1000.0, 1.0, JsonScale::DividedBy(kKgPerSlug),
                      [](ScenarioData& s, World&, EntityID) { return &s.missilePropellantMassSlug; }));

    const char* kSeeker = "Missile / Seeker";
    out.push_back(Dbl("/missile_doctrine/seeker/fov_half_angle_deg", kSeeker, "Gimbal half-cone", "deg",
                      Owner::Doctrine, Timing::NextLaunch, 5.0, 90.0, 0.5, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerFovHalfAngleDeg; }));
    out.push_back(Dbl("/missile_doctrine/seeker/memory_coast_time_s", kSeeker, "Memory coast", "s",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 30.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileMemoryCoastTimeS; }));
    out.push_back(Dbl("/missile_doctrine/seeker/reference_range_nmi", kSeeker, "Reference acquisition range",
                      "nmi", Owner::Doctrine, Timing::NextLaunch, 0.5, 100.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerReferenceRangeNmi; }));
    out.push_back(Enum("/missile_doctrine/seeker/mode", kSeeker, "Seeker mode", Owner::Doctrine, Timing::NextLaunch,
                       EnumAccess{{"ARH", "SARH", "MIDCOURSE_ARH"},
                                  [](ScenarioData& s, World&, EntityID) { return static_cast<int>(s.missileSeekerMode); },
                                  [](ScenarioData& s, World&, EntityID, int v) {
                                      s.missileSeekerMode = static_cast<SeekerMode>(v);
                                  }}));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/average_power_w", kSeeker, "Seeker average power", "W",
                      Owner::Doctrine, Timing::NextLaunch, 1.0, 10000.0, 5.0, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerAveragePowerW; }));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/antenna_gain", kSeeker, "Seeker antenna gain", "",
                      Owner::Doctrine, Timing::NextLaunch, 1.0, 100000.0, 10.0, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerAntennaGain; }));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/frequency_hz", kSeeker, "Seeker frequency", "Hz",
                      Owner::Doctrine, Timing::NextLaunch, 1e8, 1e11, 1e7, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerFrequencyHz; }));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/noise_figure_db", kSeeker, "Seeker noise figure", "dB",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 20.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerNoiseFigureDb; }));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/integration_time_s", kSeeker, "Seeker integration time", "s",
                      Owner::Doctrine, Timing::NextLaunch, 0.0001, 1.0, 0.001, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerIntegrationTimeS; }));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/system_loss_db", kSeeker, "Seeker system loss", "dB",
                      Owner::Doctrine, Timing::NextLaunch, 0.0, 40.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerSystemLossDb; }));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/detection_threshold_db", kSeeker, "Seeker threshold", "dB",
                      Owner::Doctrine, Timing::NextLaunch, -20.0, 60.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerThresholdDb; }));
    out.push_back(Dbl("/missile_doctrine/seeker/active_seeker/sweep_period_s", kSeeker, "Seeker sweep period", "s",
                      Owner::Doctrine, Timing::NextLaunch, 0.01, 10.0, 0.01, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileSeekerSweepPeriodS; }));

    const char* kLink = "Missile / Datalink";
    out.push_back(Dbl("/missile_doctrine/datalink/update_interval_s", kLink, "Update interval", "s",
                      Owner::Doctrine, Timing::NextLaunch, 0.01, 60.0, 0.1, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.missileDatalink.updateIntervalS; }));
}

void BuildEnvironment(std::vector<ParamDef>& out) {
    const char* kEnvironment = "Environment";
    // Live: EnvironmentSystem re-applies a change to every aircraft's
    // JSBSim atmosphere on the next tick, and every other reader goes
    // through SimContext, which SimLoop refreshes each tick.
    out.push_back(Dbl("/environment/temperature_offset_c", kEnvironment, "ISA temperature offset", "C",
                      Owner::Environment, Timing::Live, -60.0, 60.0, 0.5, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.environment.temperatureOffsetC; }));
    // Live, the same way as the temperature: EnvironmentSystem re-applies a
    // change to JSBSim, and the missile and waypoint steering read it from
    // SimContext every tick.
    out.push_back(Dbl("/environment/wind_speed_kts", kEnvironment, "Wind speed", "kt", Owner::Environment,
                      Timing::Live, 0.0, 300.0, 1.0, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.environment.windSpeedKts; }));
    out.push_back(Dbl("/environment/wind_from_deg", kEnvironment, "Wind from", "deg", Owner::Environment,
                      Timing::Live, 0.0, 360.0, 1.0, 1.0,
                      [](ScenarioData& s, World&, EntityID) { return &s.environment.windFromDeg; }));
}

void BuildEntity(std::vector<ParamDef>& out) {
    // Setup-timed, and backed by SpawnPose rather than the live Transform,
    // so a save or Reset mid-run writes the start line, not wherever the
    // entity has flown to. See ParamTiming.
    const char* kPlace = "Placement";
    out.push_back(Dbl("/entities/{}/transform/pos/0", kPlace, "East", "ft", Owner::Entity, Timing::Setup, -1e7,
                      1e7, 100.0, 1.0, SpawnField(0)));
    out.push_back(Dbl("/entities/{}/transform/pos/1", kPlace, "Altitude", "ft", Owner::Entity, Timing::Setup, 0.0,
                      100000.0, 100.0, 1.0, SpawnField(1)));
    out.push_back(Dbl("/entities/{}/transform/pos/2", kPlace, "North", "ft", Owner::Entity, Timing::Setup, -1e7,
                      1e7, 100.0, 1.0, SpawnField(2)));
    out.push_back(Dbl("/entities/{}/transform/heading", kPlace, "Heading", "deg", Owner::Entity, Timing::Setup,
                      0.0, 360.0, 1.0, 1.0,
                      [](ScenarioData&, World& w, EntityID e) -> double* {
                          SpawnPose* spawn = Comp<SpawnPose>(w, e);
                          return spawn ? &spawn->headingDeg : nullptr;
                      }));

    // Autopilot setpoints are deliberately NOT here. They have no schema
    // keys of their own -- the loader seeds them from the jsbsim block's
    // altitude_ft / heading_deg / airspeed_kts, which are initial
    // conditions -- and the live setpoint is a command, not a scenario
    // parameter. Persisting one would invent a key the loader cannot read
    // back. EntityInspector keeps its own live editor for them.
    //
    // The jsbsim block itself is excluded for the same reason: once the
    // FDMExec is running there is no live value to round-trip an initial
    // condition against.

    const char* kGLimit = "Flight envelope";
    out.push_back(Dbl("/entities/{}/components/g_limit/clmax", kGLimit, "CL max", "", Owner::Entity, Timing::Live,
                      0.1, 3.0, 0.05, 1.0, Field(&GLimitComponent::clMax)));
    out.push_back(Dbl("/entities/{}/components/g_limit/cl_max_loss_per_g", kGLimit, "CL loss per G", "", Owner::Entity,
                      Timing::Live, 0.0, 0.2, 0.005, 1.0, Field(&GLimitComponent::clMaxLossPerG)));
    out.push_back(Dbl("/entities/{}/components/g_limit/wing_area_m2", kGLimit, "Wing area", "m2", Owner::Entity,
                      Timing::Live, 1.0, 200.0, 0.5, kFeetSqPerMeterSq, Field(&GLimitComponent::wingAreaFt2)));
    out.push_back(Dbl("/entities/{}/components/g_limit/structural_g_limit_pos", kGLimit, "Structural +G", "G",
                      Owner::Entity, Timing::Live, 1.0, 20.0, 0.1, 1.0,
                      Field(&GLimitComponent::structuralGLimitPos)));
    out.push_back(Dbl("/entities/{}/components/g_limit/structural_g_limit_neg", kGLimit, "Structural -G", "G",
                      Owner::Entity, Timing::Live, -20.0, 0.0, 0.1, 1.0,
                      Field(&GLimitComponent::structuralGLimitNeg)));
    out.push_back(Dbl("/entities/{}/components/g_limit/damage_accumulation_rate", kGLimit, "Damage rate", "",
                      Owner::Entity, Timing::Live, 0.0, 1.0, 0.005, 1.0,
                      Field(&GLimitComponent::damageAccumulationRate)));

    const char* kSignature = "Radar signature";
    out.push_back(Dbl("/entities/{}/components/signature/rcs_scale", kSignature, "RCS scale", "x", Owner::Entity,
                      Timing::Live, 0.001, 1000.0, 0.05, 1.0, Field(&RadarCrossSection::rcsScale)));

    const char* kAir = "Airborne radar";
    out.push_back(Dbl("/entities/{}/components/airborne_radar/reference_range_nmi", kAir,
                      "Reference detection range", "nmi", Owner::Entity, Timing::Live, 1.0, 500.0, 0.5, 1.0,
                      Field(&AirborneRadarComponent::referenceRangeNmi)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/frame_time_s", kAir, "Frame time", "s",
                      Owner::Entity, Timing::Live, 0.1, 60.0, 0.1, 1.0, Field(&AirborneRadarComponent::frameTimeS)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/stt_frame_time_s", kAir, "STT frame time", "s",
                      Owner::Entity, Timing::Live, 0.01, 60.0, 0.01, 1.0,
                      Field(&AirborneRadarComponent::sttFrameTimeS)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/fire_control_delay_s", kAir, "Fire-control delay",
                      "s", Owner::Entity, Timing::Live, 0.0, 30.0, 0.1, 1.0,
                      Field(&AirborneRadarComponent::fireControlDelayS)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/average_power_w", kAir, "Average power", "W",
                      Owner::Entity, Timing::Live, 1.0, 100000.0, 10.0, 1.0,
                      Field(&AirborneRadarComponent::averagePowerW)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/peak_power_w", kAir, "Peak power", "W",
                      Owner::Entity, Timing::Live, 1.0, 10000000.0, 1000.0, 1.0,
                      Field(&AirborneRadarComponent::peakPowerW)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/antenna_gain", kAir, "Antenna gain", "",
                      Owner::Entity, Timing::Live, 1.0, 1000000.0, 50.0, 1.0,
                      Field(&AirborneRadarComponent::antennaGain)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/frequency_hz", kAir, "Frequency", "Hz",
                      Owner::Entity, Timing::Live, 1e8, 1e11, 1e7, 1.0,
                      Field(&AirborneRadarComponent::frequencyHz)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/noise_figure_db", kAir, "Noise figure", "dB",
                      Owner::Entity, Timing::Live, 0.0, 20.0, 0.1, 1.0,
                      Field(&AirborneRadarComponent::noiseFigureDb)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/time_on_target_s", kAir, "Time on target", "s",
                      Owner::Entity, Timing::Live, 0.0001, 10.0, 0.001, 1.0,
                      Field(&AirborneRadarComponent::timeOnTargetS)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/system_loss_db", kAir, "System loss", "dB",
                      Owner::Entity, Timing::Live, 0.0, 40.0, 0.1, 1.0,
                      Field(&AirborneRadarComponent::systemLossDb)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/fov_half_angle_deg", kAir, "FOV half-angle", "deg",
                      Owner::Entity, Timing::Live, 5.0, 180.0, 1.0, 1.0,
                      Field(&AirborneRadarComponent::fovHalfAngleDeg)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/search_threshold_db", kAir, "Search threshold", "dB",
                      Owner::Entity, Timing::Live, -20.0, 60.0, 0.1, 1.0,
                      Field(&AirborneRadarComponent::searchThresholdDb)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/notch_velocity_threshold_mps", kAir,
                      "Notch threshold", "m/s", Owner::Entity, Timing::Live, 0.0, 200.0, 1.0, 1.0,
                      Field(&AirborneRadarComponent::notchVelocityThresholdMps)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/antenna_height_m", kAir, "Antenna height", "m",
                      Owner::Entity, Timing::Live, 0.05, 5.0, 0.01, 1.0,
                      Field(&AirborneRadarComponent::antennaHeightM)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/min_range_m", kAir, "Blind range", "m",
                      Owner::Entity, Timing::Live, 0.0, 50000.0, 50.0, 1.0,
                      Field(&AirborneRadarComponent::minRangeM)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/antenna_width_m", kAir, "Antenna width", "m",
                      Owner::Entity, Timing::Live, 0.05, 5.0, 0.01, 1.0,
                      Field(&AirborneRadarComponent::antennaWidthM)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/beamwidth_factor", kAir, "Beamwidth factor",
                      "lambda/D", Owner::Entity, Timing::Live, 0.5, 3.0, 0.05, 1.0,
                      Field(&AirborneRadarComponent::beamwidthFactor)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/monopulse_slope", kAir, "Monopulse slope", "",
                      Owner::Entity, Timing::Live, 0.5, 5.0, 0.05, 1.0,
                      Field(&AirborneRadarComponent::monopulseSlope)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/pulse_width_s", kAir, "Pulse width", "s",
                      Owner::Entity, Timing::Live, 1e-8, 1e-4, 1e-7, 1.0,
                      Field(&AirborneRadarComponent::pulseWidthS)));
    out.push_back(Dbl("/entities/{}/components/airborne_radar/track_maneuver_g", kAir, "Track maneuver", "g",
                      Owner::Entity, Timing::Live, 0.0, 20.0, 0.1, 1.0,
                      Field(&AirborneRadarComponent::trackManeuverG)));

    const char* kRwr = "RWR";
    out.push_back(Dbl("/entities/{}/components/evasion/rwr/antenna_gain", kRwr, "Antenna gain", "", Owner::Entity,
                      Timing::Live, 0.1, 10000.0, 1.0, 1.0, Field(&RwrComponent::antennaGain)));
    out.push_back(Dbl("/entities/{}/components/evasion/rwr/sensitivity_dbm", kRwr, "Sensitivity", "dBm",
                      Owner::Entity, Timing::Live, -120.0, 0.0, 1.0, 1.0, Field(&RwrComponent::sensitivityDbm)));
    out.push_back(Dbl("/entities/{}/components/evasion/rwr/signal_jitter_db", kRwr, "Signal jitter", "dB", Owner::Entity,
                      Timing::Live, 0.0, 20.0, 0.1, 1.0, Field(&RwrComponent::signalJitterDb)));
    out.push_back(Dbl("/entities/{}/components/evasion/rwr/sweep_period_s", kRwr, "Sweep period", "s", Owner::Entity,
                      Timing::Live, 0.05, 60.0, 0.1, 1.0, Field(&RwrComponent::sweepPeriodS)));
}

// One-line tooltip text per field, keyed by the same JSON Pointer template
// the table uses. Kept separate from the shorthand constructors so those
// stay readable as data; assigned onto each ParamDef in All(). Sourced from
// docs/SCHEMA.md -- keep the two in step. A field with no entry here simply
// shows a tooltip with its range and timing but no prose.
const char* DescriptionFor(const char* pointer) {
    static const std::unordered_map<std::string, const char*> kDesc = {
        // --- Environment ---
        {"/environment/temperature_offset_c",
         "ISA deviation: every layer of the atmosphere runs this much hotter (+) or colder. Hot air is thinner "
         "near the ground and denser high up; aircraft and missile both fly it."},
        {"/environment/wind_speed_kts",
         "Steady wind, the same at every altitude. Aircraft and missile both fly relative to the moving air, and "
         "waypoint steering crabs into it to hold the track."},
        {"/environment/wind_from_deg",
         "Compass direction the wind blows FROM: 270 is a westerly, moving air eastward."},
        // --- Missile / Core ---
        {"/missile_doctrine/kill_radius_m",
         "Distance from the nearest point on the target's airframe at which lethality is evaluated."},
        {"/missile_doctrine/max_range_m",
         "Range beyond which guidance gives up: the missile logs a MISS and coasts ballistically."},
        {"/missile_doctrine/launch_range_fraction",
         "Fire once the target is inside this fraction of the missile's maximum launch range (1 = at Rmax). "
         "Shooting closer gives up the first shot for a missile that arrives with more energy."},
        {"/missile_doctrine/launch_mode",
         "TWS fires on the scanning radar's track. STT locks the target first: a fresher track for the missile, "
         "but the target's RWR shows a lock and the radar sees nothing else. A SARH missile always uses STT."},
        {"/missile_doctrine/loft",
         "On a long shot the missile climbs into thin air, where drag is low, then dives onto the target: it "
         "reaches farther and arrives faster. The climb angle grows with launch range (the missile type's)."},
        {"/missile_doctrine/arming_range_m",
         "Minimum distance the missile must fly from its launch point before a kill can register."},
        {"/missile_doctrine/falloff_width_m",
         "Width of the Pk-vs-miss-distance sigmoid around the kill radius; larger is more gradual."},
        {"/missile_doctrine/reference_area_m2",
         "Cross-sectional area used for aerodynamic drag (distinct from radar cross-section)."},
        // --- Missile / Proportional Navigation ---
        {"/missile_doctrine/nav_gain",
         "Proportional-navigation gain N: how hard the missile turns per unit line-of-sight rate. Typical 3-6."},
        // --- Missile / Autopilot ---
        {"/missile_doctrine/autopilot/launch_turn_exit_deg",
         "After launch the autopilot turns toward the target at full available G until it is within this many "
         "degrees of the missile's flight path, then proportional navigation takes over. Starts as the missile "
         "type's (AIM-120: 15 deg, an estimate); 0 = no launch turn."},
        // --- Missile / Self-destruct ---
        {"/missile_doctrine/self_destruct/min_available_g",
         "After motor burnout, the missile self-destructs once the G it can still pull falls below this: too slow "
         "to steer. 0 disables. Starts as the missile type's."},
        {"/missile_doctrine/self_destruct/max_time_of_flight_s",
         "Backstop: the missile self-destructs this long after launch, guided or coasting. 0 disables. Starts as "
         "the missile type's."},
        // --- Missile / Thrust ---
        {"/missile_doctrine/thrust_profile/boost_duration_s", "Duration of the high-thrust boost stage."},
        {"/missile_doctrine/thrust_profile/boost_thrust_n", "Axial thrust during the boost stage."},
        {"/missile_doctrine/thrust_profile/sustain_duration_s", "Duration of the lower-thrust sustain stage."},
        {"/missile_doctrine/thrust_profile/sustain_thrust_n", "Axial thrust during the sustain stage."},
        {"/missile_doctrine/thrust_profile/dry_mass_kg", "Missile mass with the motor burnt out."},
        {"/missile_doctrine/thrust_profile/propellant_mass_kg",
         "Propellant load, burnt off linearly across boost + sustain. With thrust this sets specific impulse."},
        // --- Missile / Seeker ---
        {"/missile_doctrine/seeker/fov_half_angle_deg",
         "Seeker gimbal cone half-angle around the missile's velocity vector."},
        {"/missile_doctrine/seeker/reference_range_nmi",
         "Range at which one seeker look at a 5 m2 target is a coin flip. Raising it scales every acquisition range "
         "by the same ratio (a more sensitive receiver: 2x range needs 16x, +12 dB)."},
        {"/missile_doctrine/seeker/memory_coast_time_s",
         "How long the seeker coasts on its last line-of-sight rate after losing lock before going ballistic."},
        {"/missile_doctrine/seeker/mode",
         "ARH (active), SARH (semi-active), or MIDCOURSE_ARH (datalink midcourse, then own active seeker)."},
        {"/missile_doctrine/seeker/active_seeker/average_power_w", "Average power of the missile's own active seeker."},
        {"/missile_doctrine/seeker/active_seeker/antenna_gain", "Seeker antenna gain (linear, not dB)."},
        {"/missile_doctrine/seeker/active_seeker/frequency_hz", "Seeker operating frequency (default X-band, 10 GHz)."},
        {"/missile_doctrine/seeker/active_seeker/noise_figure_db", "Seeker receiver noise figure."},
        {"/missile_doctrine/seeker/active_seeker/integration_time_s", "Coherent integration time per seeker sweep."},
        {"/missile_doctrine/seeker/active_seeker/system_loss_db",
         "All other seeker losses; the type's value is fitted to its cited acquisition range."},
        {"/missile_doctrine/seeker/active_seeker/detection_threshold_db",
         "Detector output over noise a seeker sweep needs to count as a detection; the type's is ln(1/pfa)."},
        {"/missile_doctrine/seeker/active_seeker/sweep_period_s", "Seeker sampling interval."},
        // --- Missile / Datalink ---
        {"/missile_doctrine/datalink/update_interval_s",
         "How often the launcher sends the missile its radar's current track of the target."},
        // --- Entity / Placement ---
        {"/entities/{}/transform/pos/0", "East position of the entity (world X)."},
        {"/entities/{}/transform/pos/1", "Altitude of the entity (world Y, up)."},
        {"/entities/{}/transform/pos/2", "North position of the entity (world Z)."},
        {"/entities/{}/transform/heading", "Initial compass heading (0 = north, 90 = east)."},
        // --- Entity / Flight envelope ---
        {"/entities/{}/components/g_limit/clmax", "Maximum lift coefficient; sets the aerodynamic G limit. Starts as the aircraft type's."},
        {"/entities/{}/components/g_limit/cl_max_loss_per_g",
         "Lift the FCS gives up per G above 1 (its AoA limit falls with load factor). Starts as the aircraft type's."},
        {"/entities/{}/components/g_limit/wing_area_m2", "Wing reference area used in the lift/G calculation. Starts as the aircraft type's."},
        {"/entities/{}/components/g_limit/structural_g_limit_pos", "Positive structural G cap. Starts as the aircraft type's."},
        {"/entities/{}/components/g_limit/structural_g_limit_neg",
         "Negative structural G cap (itself negative). Starts as the aircraft type's."},
        {"/entities/{}/components/g_limit/damage_accumulation_rate",
         "How fast cumulative structural damage accrues during overstress."},
        // --- Entity / Airborne radar ---
        {"/entities/{}/components/airborne_radar/reference_range_nmi",
         "Range at which one look at a 5 m2 target is a coin flip. Raising it scales every detection range by the "
         "same ratio, as a more sensitive receiver would (2x range needs 16x, +12 dB); what an RWR hears is "
         "unchanged."},
        {"/entities/{}/components/signature/rcs_scale",
         "Multiplies this aircraft's radar cross-section as every radar and seeker sees it. Detection range goes "
         "as its fourth root: 16x the RCS is seen twice as far."},
        {"/entities/{}/components/airborne_radar/frame_time_s",
         "One search / track-while-scan frame: every target in the cone gets one look per frame. 4.4 s is a "
         "+/-60 deg, 2-bar raster at 60 deg/s."},
        {"/entities/{}/components/airborne_radar/stt_frame_time_s",
         "One single-target-track frame: while locked, the radar looks at its one target this often and at "
         "nothing else."},
        {"/entities/{}/components/airborne_radar/fire_control_delay_s",
         "How long the launch gate must stay open before the missile fires."},
        {"/entities/{}/components/airborne_radar/average_power_w",
         "Average transmitted power, what the integrated return scales with."},
        {"/entities/{}/components/airborne_radar/peak_power_w", "Peak pulse power, what a radar warning receiver hears."},
        {"/entities/{}/components/airborne_radar/antenna_gain", "Antenna gain G (linear, not dB)."},
        {"/entities/{}/components/airborne_radar/frequency_hz", "Operating frequency; sets the wavelength."},
        {"/entities/{}/components/airborne_radar/noise_figure_db", "Receiver system noise figure."},
        {"/entities/{}/components/airborne_radar/time_on_target_s",
         "Coherent time the beam dwells on a target per sweep."},
        {"/entities/{}/components/airborne_radar/system_loss_db",
         "All other losses together; the type's value is fitted to its cited detection range."},
        {"/entities/{}/components/airborne_radar/fov_half_angle_deg",
         "Half-angle of the nose cone the radar can see; outside it is an automatic miss."},
        {"/entities/{}/components/airborne_radar/search_threshold_db",
         "Detector output over noise one look needs to count as a detection; the type's is ln(1/pfa)."},
        {"/entities/{}/components/airborne_radar/notch_velocity_threshold_mps",
         "Ground-relative radial speed below which a target is filtered out as clutter, while the beam touches the ground."},
        {"/entities/{}/components/airborne_radar/antenna_height_m",
         "Vertical size of the antenna aperture; with the wavelength it sets how far the beam spreads up and down."},
        {"/entities/{}/components/airborne_radar/min_range_m",
         "Blind range: closer than this is a forced miss regardless of signal strength."},
        {"/entities/{}/components/airborne_radar/antenna_width_m",
         "Horizontal size of the antenna aperture; with the wavelength it sets the azimuth beamwidth."},
        {"/entities/{}/components/airborne_radar/beamwidth_factor",
         "Half-power beamwidth in wavelengths per aperture size (~1.2 for a tapered aperture)."},
        {"/entities/{}/components/airborne_radar/monopulse_slope",
         "Angle error per look = beamwidth / (this x sqrt(2 SNR)); 1.6 splits the beam ~10:1 at 13 dB."},
        {"/entities/{}/components/airborne_radar/pulse_width_s",
         "Range error per look = c x this / (2 sqrt(2 SNR))."},
        {"/entities/{}/components/airborne_radar/track_maneuver_g",
         "The tracker's 1-sigma allowance for target acceleration: higher follows maneuvers faster, smooths noise less."},
        // --- Entity / RWR ---
        {"/entities/{}/components/evasion/rwr/antenna_gain", "RWR receive antenna gain in the link budget."},
        {"/entities/{}/components/evasion/rwr/sensitivity_dbm",
         "Weakest received pulse that registers as illuminated, at the antenna port."},
        {"/entities/{}/components/evasion/rwr/signal_jitter_db",
         "Standard deviation of Gaussian jitter on received power each sweep (the type's ESTIMATE)."},
        {"/entities/{}/components/evasion/rwr/sweep_period_s", "RWR sampling interval."},
    };
    auto it = kDesc.find(pointer);
    return it == kDesc.end() ? nullptr : it->second;
}

// The teaching knobs (docs/SCHEMA.md - Teaching knobs): the only entries a
// user edits. Each earns its place by teaching a concept
// visible in the result; everything else is the type file's.
bool IsKnob(const char* pointer) {
    static const std::unordered_set<std::string> kKnobs = {
        // Missile
        "/missile_doctrine/nav_gain",
        "/missile_doctrine/seeker/mode",
        "/missile_doctrine/seeker/fov_half_angle_deg",
        "/missile_doctrine/seeker/memory_coast_time_s",
        "/missile_doctrine/datalink/update_interval_s",
        "/missile_doctrine/self_destruct/max_time_of_flight_s",
        "/missile_doctrine/kill_radius_m",
        "/missile_doctrine/max_range_m",
        "/missile_doctrine/launch_range_fraction",
        "/missile_doctrine/launch_mode",
        "/missile_doctrine/loft",
        // Sensors
        "/entities/{}/components/airborne_radar/reference_range_nmi",
        "/missile_doctrine/seeker/reference_range_nmi",
        "/entities/{}/components/signature/rcs_scale",
        // Environment
        "/environment/temperature_offset_c",
        "/environment/wind_speed_kts",
        "/environment/wind_from_deg",
        // Placement
        "/entities/{}/transform/pos/0",
        "/entities/{}/transform/pos/1",
        "/entities/{}/transform/pos/2",
        "/entities/{}/transform/heading",
    };
    return kKnobs.count(pointer) != 0;
}

// The knobs the GUI keeps for Advanced mode: finer points of the
// engagement, none of them a lesson's suggested change.
bool IsAdvancedKnob(const char* pointer) {
    static const std::unordered_set<std::string> kAdvanced = {
        "/missile_doctrine/kill_radius_m",
        "/missile_doctrine/max_range_m",
        "/missile_doctrine/seeker/fov_half_angle_deg",
        "/missile_doctrine/seeker/memory_coast_time_s",
        "/missile_doctrine/seeker/reference_range_nmi",
        "/entities/{}/components/signature/rcs_scale",
    };
    return kAdvanced.count(pointer) != 0;
}

}  // namespace

const std::vector<ParamDef>& All() {
    static const std::vector<ParamDef> table = [] {
        std::vector<ParamDef> out;
        BuildDoctrine(out);
        BuildEnvironment(out);
        BuildEntity(out);
        for (ParamDef& p : out) {
            p.description = DescriptionFor(p.pointer);
            p.knob = IsKnob(p.pointer);
            p.advanced = p.knob && IsAdvancedKnob(p.pointer);
        }
        return out;
    }();
    return table;
}

std::string ResolvePointer(const ParamDef& param, int index) {
    std::string pointer = param.pointer;
    size_t brace = pointer.find("{}");
    if (brace == std::string::npos) return pointer;
    return pointer.substr(0, brace) + std::to_string(index) + pointer.substr(brace + 2);
}

const ParamDef* Find(const std::string& pointer) {
    for (const ParamDef& param : All()) {
        std::string pattern = param.pointer;
        if (pointer == pattern) return &param;
        size_t brace = pattern.find("{}");
        if (brace == std::string::npos) continue;
        std::string prefix = pattern.substr(0, brace);
        std::string suffix = pattern.substr(brace + 2);
        if (pointer.size() <= prefix.size() + suffix.size()) continue;
        if (pointer.compare(0, prefix.size(), prefix) != 0) continue;
        if (pointer.compare(pointer.size() - suffix.size(), suffix.size(), suffix) != 0) continue;
        std::string index = pointer.substr(prefix.size(), pointer.size() - prefix.size() - suffix.size());
        if (std::all_of(index.begin(), index.end(), [](char c) { return c >= '0' && c <= '9'; })) return &param;
    }
    return nullptr;
}

int EnumIndex(const ParamDef& param, const std::string& name) {
    const std::vector<const char*>& names = param.enumAccess.names;
    for (size_t i = 0; i < names.size(); ++i) {
        if (name == names[i]) return static_cast<int>(i);
    }
    return -1;
}

}  // namespace ParamRegistry
