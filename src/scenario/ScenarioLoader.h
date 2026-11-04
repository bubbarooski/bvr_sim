#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "components/DatalinkTrackComponent.h"
#include "components/GuidanceComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileType.h"
#include "sim/Environment.h"
#include "util/Units.h"

struct ScenarioData {
    uint64_t seed = 0;
    double simSpeed = 1.0;

    // The optional top-level "description": what the scenario teaches and
    // what to watch, shown in the GUI. The sim never reads it.
    std::string description;

    // Scenario-wide atmospheric conditions from the optional top-level
    // "environment" block. Absent means a standard day.
    Environment environment;

    // Missile doctrine, parsed from the optional top-level
    // "missile_doctrine" object and converted from the JSON's meters to
    // the feet every other distance in this sim already uses.
    double killRadiusFt = 0.0;
    double maxRangeFt = 0.0;
    // The teaching knob on the launch gate: fire once the target is inside
    // this fraction of the missile's maximum launch range (1 = at Rmax).
    double launchRangeFraction = 1.0;
    // The teaching knob on how the launcher holds the target for the shot
    // (missile_doctrine.launch_mode, "TWS" or "STT").
    LaunchMode missileLaunchMode = LaunchMode::TWS;
    // The teaching knob on the loft (missile_doctrine.loft): whether a long
    // shot climbs on the type's loft schedule or flies straight at the
    // target. On by default: that is how a long shot is flown.
    bool missileLoft = true;
    double missileNavGain = GuidanceComponent{}.navGain;

    // The missile the doctrine fires (missile_doctrine.missile), or null
    // for a scenario with no missile. Its values seed the per-shot fields
    // below (SetMissileType), which the doctrine may then override.
    std::shared_ptr<const MissileType> missileType;

    // The launcher-to-missile datalink: how good the target position the
    // missile is steering to actually is during midcourse. Defaults live
    // on the component; overridden from missile_doctrine.datalink.
    DatalinkTrackComponent missileDatalink;

    // The missile type's thrust profile, seeker, and drag reference area.
    // Only the seeker's mode, gimbal cone, memory coast and reference range
    // are teaching knobs (missile_doctrine.seeker); the rest is the type's.
    double missileBoostDurationS = 0.0;
    double missileBoostThrustLbf = 0.0;
    double missileSustainDurationS = 0.0;
    double missileSustainThrustLbf = 0.0;

    // Mass split. Tunable because it is the other half of the thrust
    // profile: the four fields above set total impulse, and these two set
    // the specific impulse that impulse implies. Changing one without the
    // other produces a motor that cannot exist.
    double missileDryMassSlug = 0.0;
    double missilePropellantMassSlug = 0.0;
    double missileReferenceAreaFt2 = 0.0;
    double missileSeekerFovHalfAngleDeg = 0.0;
    double missileMemoryCoastTimeS = 0.0;

    // Minimum arming range: the type's (fuze).
    double missileArmingRangeFt = 0.0;

    // Self-destruct limits: the type's. The time limit alone is a teaching
    // knob (missile_doctrine.self_destruct.max_time_of_flight_s).
    double missileSelfDestructMinAvailableG = 0.0;
    double missileSelfDestructMaxTimeOfFlightS = 0.0;

    // Probabilistic warhead lethality: how gradual the Pk-vs-miss-distance
    // sigmoid is around killRadiusFt. The missile type's (fuze), or this for
    // a scenario with no missile type.
    static constexpr double kNoTypeFalloffWidthM = 5.0;
    double falloffWidthFt = kNoTypeFalloffWidthM * Units::kFeetPerMeter;

    // The type's seeker mode, optionally overridden by the teaching knob
    // missile_doctrine.seeker.mode ("ARH", "SARH", or "MIDCOURSE_ARH").
    SeekerMode missileSeekerMode = SeekerMode::ARH;

    // The missile's own active-seeker radar: the type's. Its reference range
    // alone is a teaching knob (missile_doctrine.seeker.reference_range_nmi).
    double missileSeekerAveragePowerW = 0.0;
    double missileSeekerAntennaGain = 0.0;
    double missileSeekerFrequencyHz = 0.0;
    double missileSeekerNoiseFigureDb = 0.0;
    double missileSeekerIntegrationTimeS = 0.0;
    double missileSeekerSystemLossDb = 0.0;
    double missileSeekerReferenceRangeNmi = 0.0;  // the teaching knob; the type's fit by default
    double missileSeekerThresholdDb = 0.0;
    double missileSeekerSweepPeriodS = 0.0;

    // The type's launch turn (read-only).
    double missileLaunchTurnExitDeg = 0.0;

    // Makes `type` the doctrine's missile and resets every per-shot field
    // above to its value. The loader calls this before applying the
    // doctrine's overrides, so a field the scenario omits is the type's.
    void SetMissileType(std::shared_ptr<const MissileType> type) {
        missileType = std::move(type);
        const MissileType& t = *missileType;
        missileBoostDurationS = t.propulsion.boostDurationS;
        missileBoostThrustLbf = t.propulsion.boostThrustLbf;
        missileSustainDurationS = t.propulsion.sustainDurationS;
        missileSustainThrustLbf = t.propulsion.sustainThrustLbf;
        missileDryMassSlug = t.mass.dryMassSlug;
        missilePropellantMassSlug = t.mass.propellantMassSlug;
        missileReferenceAreaFt2 = t.aero.referenceAreaFt2;
        missileSeekerMode = t.seeker.mode;
        missileSeekerFovHalfAngleDeg = t.seeker.fovHalfAngleDeg;
        missileMemoryCoastTimeS = t.seeker.memoryCoastTimeS;
        missileSeekerAveragePowerW = t.seeker.averagePowerW;
        missileSeekerAntennaGain = t.seeker.antennaGain;
        missileSeekerFrequencyHz = t.seeker.frequencyHz;
        missileSeekerNoiseFigureDb = t.seeker.noiseFigureDb;
        missileSeekerIntegrationTimeS = t.seeker.integrationTimeS;
        missileSeekerSystemLossDb = t.seeker.systemLossDb;
        missileSeekerReferenceRangeNmi = t.seeker.referenceRangeNmi;
        missileSeekerThresholdDb = t.seeker.thresholdDb;
        missileSeekerSweepPeriodS = t.seeker.sweepPeriodS;
        missileLaunchTurnExitDeg = t.autopilot.launchTurnExitDeg;
        missileArmingRangeFt = t.fuze.armingRangeFt;
        falloffWidthFt = t.fuze.falloffWidthFt;
        missileSelfDestructMinAvailableG = t.selfDestruct.minAvailableG;
        missileSelfDestructMaxTimeOfFlightS = t.selfDestruct.maxTimeOfFlightS;
    }
};

namespace ScenarioLoader {

// Parses the scenario JSON at `path`, validates the required fields exist
// (throws std::runtime_error naming the missing field otherwise), and spawns
// one entity per entry in `entities` with Transform + Tag attached.
ScenarioData Load(const std::string& path, World& world);

}  // namespace ScenarioLoader
