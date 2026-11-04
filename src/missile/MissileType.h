#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "components/GuidanceComponent.h"
#include "missile/LaunchZone.h"
#include "missile/MissileNormalForce.h"
#include "sim/DragTable.h"

// Everything that is true of every missile of one type, as opposed to one
// shot in one scenario: its motor, its mass, its aerodynamics, its seeker
// hardware and its fuze. Loaded once per type from
// data/missiles/<id>/missile.json by MissileCatalog and shared, read-only,
// by every missile of that type (MissileComponent). A scenario names the
// type in missile_doctrine and may override individual values for that
// scenario; how the missile is employed (nav gain, range, error models)
// stays in the doctrine. A new missile is a new directory, not new code.
//
// English units throughout, like every other flight-physics quantity here.

// Three-stage boost/sustain/coast motor. See REFERENCE.md - Three-stage
// propulsion.
struct MissilePropulsion {
    double boostDurationS = 0.0;
    double boostThrustLbf = 0.0;
    double sustainDurationS = 0.0;
    double sustainThrustLbf = 0.0;
};

// Propellant burns off linearly over boost + sustain. Together with the
// motor's total impulse this sets the specific impulse.
struct MissileMass {
    double dryMassSlug = 0.0;
    double propellantMassSlug = 0.0;
};

struct MissileAerodynamics {
    double referenceAreaFt2 = 0.0;  // drag reference area (body cross-section), not radar cross-section
    DragTable drag;                 // zero-lift Cd against Mach, burning or coasting

    // Zero-lift Cd: one table for the whole flight. See REFERENCE.md 6.4.
    double ZeroLiftCd(double mach) const { return drag.Lookup(mach); }

    // CN(alpha, Mach) up to alpha_max, for the turn ceiling and the drag due
    // to lift. Shared so each missile's MissileGLimitComponent can hold it.
    // See REFERENCE.md - Missile normal force and turn ceiling.
    std::shared_ptr<const MissileNormalForce> normalForce;
    double structuralGLimit = 0.0;
};

// Seeker hardware: which guidance it supports, how far it can look off
// boresight, how long it coasts on memory, and the active radar's link
// budget. See REFERENCE.md - Seeker Modeling.
struct MissileSeeker {
    SeekerMode mode = SeekerMode::ARH;
    double fovHalfAngleDeg = 0.0;
    double memoryCoastTimeS = 0.0;
    // The active radar's receiver budget (see ReceiverBudget.h) and its
    // detection threshold.
    double averagePowerW = 0.0;
    double antennaGain = 0.0;  // linear
    double frequencyHz = 0.0;
    double noiseFigureDb = 0.0;
    double integrationTimeS = 0.0;  // coherent integration per sweep
    double systemLossDb = 0.0;      // fitted to the cited acquisition range
    // That acquisition range (fit/radar_fit.json range_nmi, against its 5
    // m2 reference): the seeker reference-range knob's baseline. 0 when the
    // type has no fit file.
    double referenceRangeNmi = 0.0;
    double pfa = 0.0;          // false-alarm probability per look
    double thresholdDb = 0.0;  // square-law detection threshold, dB over noise: from pfa
    double sweepPeriodS = 0.0;
    // M-of-N acquisition: confirmHits detections in the last confirmWindow
    // sweeps. See REFERENCE.md - The active seeker.
    int confirmHits = 0;
    int confirmWindow = 0;
    // The Doppler notch, the aircraft radar's rule seen from the missile:
    // the beam (beamwidthFactor x lambda / antennaDiameterM either side of
    // the line of sight) touches the ground and the target's ground-relative
    // radial velocity is under the threshold. See REFERENCE.md - The active
    // seeker.
    double antennaDiameterM = 0.0;
    double beamwidthFactor = 0.0;
    double notchVelocityThresholdMps = 0.0;
};

// What the autopilot does beyond flying the guidance command. See
// REFERENCE.md 6.8 - Gravity and gravity bias.
struct MissileAutopilot {
    // The guidance response time: one first-order lag between the demanded
    // and the delivered lateral acceleration, standing for the seeker, fins
    // and airframe together. See REFERENCE.md - Guidance response time.
    double responseTimeS = 0.0;
    // After launch, while the target is more than this far off the
    // missile's velocity vector, the autopilot turns toward the line of
    // sight at the G it has instead of flying the guidance law; the first
    // time the angle falls inside, the guidance law takes over for the rest
    // of the flight. 0 = no launch turn. See REFERENCE.md 6.9 - Launch turn.
    double launchTurnExitDeg = 0.0;
};

// The loft: on a long shot, a missile still steering on the datalink climbs
// into thinner air at a fixed flight-path angle, then pushes over onto the
// target. The angle comes from the launch range: none below startRangeNmi,
// rising linearly to maxClimbDeg at fullRangeNmi and held beyond. See
// REFERENCE.md - Loft.
struct MissileLoft {
    double startRangeNmi = 0.0;
    double fullRangeNmi = 0.0;
    double maxClimbDeg = 0.0;
    // How fast the flight-path angle follows the commanded climb: a first
    // order lag on gamma, so the pull-up asks for V (gamma_L - gamma) / tau.
    double climbTimeConstantS = 0.0;
    // Once burnt out, the climb ends when the G the airframe can still pull
    // (the turn ceiling, which falls with the air's density) drops under
    // this: thin air leaves no steering for the endgame.
    double minAvailableG = 0.0;

    double ClimbDeg(double launchRangeNmi) const {
        if (maxClimbDeg <= 0.0 || launchRangeNmi <= startRangeNmi) return 0.0;
        if (launchRangeNmi >= fullRangeNmi) return maxClimbDeg;
        return maxClimbDeg * (launchRangeNmi - startRangeNmi) / (fullRangeNmi - startRangeNmi);
    }
};

struct MissileFuze {
    double armingRangeFt = 0.0;  // straight-line distance from launch before a kill can register
    double falloffWidthFt = 0.0;  // width of the Pk-vs-miss-distance sigmoid around the kill radius
};

// When the missile destroys itself rather than fly on. The real end of its
// useful flight is aerodynamic: once burnt out, the G it can still pull
// falls with speed, and below the floor it can no longer steer. The time
// of flight is a backstop. See REFERENCE.md 12.7 - Self-destruct.
struct MissileSelfDestruct {
    double minAvailableG = 0.0;     // checked only after motor burnout
    double maxTimeOfFlightS = 0.0;  // since launch, any phase
};

struct MissileType {
    std::string id;           // the directory name, and what scenarios name
    std::string displayName;  // for the GUI and reports

    MissilePropulsion propulsion;
    MissileMass mass;
    MissileAerodynamics aero;
    MissileSeeker seeker;
    MissileAutopilot autopilot;
    MissileLoft loft;
    MissileFuze fuze;
    MissileSelfDestruct selfDestruct;

    // Maximum launch range by launch conditions, from fit/launch_zone.json
    // (tools/drivers/launch_zone.cpp). Null until that file has been generated. A
    // table generated from a different missile.json still loads, with a
    // warning: rerun the tool.
    std::shared_ptr<const LaunchZone> launchZone;
    bool launchZoneStale = false;

    // This type's own directory: data/missiles/<id>.
    std::filesystem::path directory;
};
