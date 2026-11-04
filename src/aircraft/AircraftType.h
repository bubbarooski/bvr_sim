#pragma once

#include "components/RadarCrossSection.h"

#include <filesystem>
#include <string>

#include "sim/AircraftBounds.h"

// Everything that is true of every aircraft of one type, as opposed to one
// aircraft in one scenario: which flight model flies it, how big it is,
// what its structure takes, and how its flight control system reads the
// stick and throttle. Loaded once per type from
// data/aircraft/<id>/aircraft.json by AircraftCatalog and shared, read-only,
// by every entity of that type (AircraftComponent). A new aircraft is a
// new directory there, not new code.
//
// English units throughout, like every other flight-physics quantity here.

// The V-n envelope's type-level inputs. Each aircraft's GLimitComponent
// starts from these and then carries its own state (damage), which is why
// the component, not this, is what systems read in flight.
struct AircraftEnvelope {
    double clMax = 0.0;  // lift at the AoA limit in 1 g flight, from the flight model's own tables
    // Lift the FCS gives up per G above 1: its AoA limit falls with load
    // factor (the F-16's G/AoA blend), so CL available at n G is
    // clMax - clMaxLossPerG (n - 1).
    double clMaxLossPerG = 0.0;
    double structuralGLimitPos = 0.0;
    double structuralGLimitNeg = 0.0;
    double damageAccumulationRate = 0.0;  // damage per g-second over the limit
};

// How the type's flight control system reads the pilot's inputs, which
// differs between aircraft and is what ControlSystem drives.
struct AircraftControlInterface {
    // Load factor commanded per unit of aft stick, on top of 1 G: the
    // stick is a G command on a fly-by-wire fighter.
    double gPerUnitStick = 0.0;
    // Throttle commands (fcs/throttle-cmd-norm) for military power and for
    // the most the engine gives (full afterburner where it has one).
    double militaryThrottleCmd = 0.0;
    double maxThrottleCmd = 0.0;
};

// The type's fire-control radar: its receiver budget, its detection
// threshold, its frames and its gates. SI units, like every radar quantity. An aircraft
// carries it only when its scenario entry has an "airborne_radar" block,
// which may override any of these for that one aircraft. See
// REFERENCE.md - Radar.
struct AirborneRadarType {
    bool present = false;  // the type file has a "radar" block

    double frameTimeS = 0.0;  // one search / TWS frame: every target in the cone gets one look
    double sttFrameTimeS = 0.0;  // one single-target-track frame: one look at the locked target
    double averagePowerW = 0.0;
    double peakPowerW = 0.0;  // what a radar warning receiver hears
    double antennaGain = 0.0;  // linear
    double frequencyHz = 0.0;
    double noiseFigureDb = 0.0;
    double timeOnTargetS = 0.0;
    double systemLossDb = 0.0;
    double pfa = 0.0;  // false-alarm probability the search threshold is set for
    double searchThresholdDb = 0.0;
    // Track confirmation: confirmFrames detections in the last
    // confirmWindowFrames frames; a confirmed track drops after dropFrames
    // frames in a row without one.
    int confirmFrames = 0;
    int confirmWindowFrames = 0;
    int dropFrames = 0;
    int sttDropFrames = 0;  // a lock breaks after this many STT frames without a detection
    // How long the launch gate must stay open before the missile fires.
    double fireControlDelayS = 0.0;
    double fovHalfAngleDeg = 0.0;
    double notchVelocityThresholdMps = 0.0;
    double antennaHeightM = 0.0;
    double minRangeM = 0.0;
    double antennaWidthM = 0.0;
    double beamwidthFactor = 0.0;
    double monopulseSlope = 0.0;
    double pulseWidthS = 0.0;
    double trackManeuverG = 0.0;
    // The search range the system loss was fitted to (fit/radar_fit.json
    // range_nmi, against its 5 m2 reference): the radar reference-range
    // knob's baseline. 0 when the type has no fit file.
    double referenceRangeNmi = 0.0;
};

// The type's radar warning receiver: what it can hear. An aircraft carries
// it only when its scenario entry has an "evasion.rwr" block, which may
// override either field. See REFERENCE.md - Radar Warning Receiver.
struct AircraftRwrType {
    bool present = false;  // the type file has an "rwr" block
    double sensitivityDbm = 0.0;  // weakest pulse it registers, at the antenna port
    double antennaGain = 0.0;     // linear
    double signalJitterDb = 0.0;  // Gaussian jitter on the received power, per sweep
    double sweepPeriodS = 0.0;
};

// The type's radar signature: what any radar sees of it. Every aircraft of
// a type that has a "signature" block is radar-visible; a scenario's
// "radar_cross_section" block may override the nose-on RCS (rescaling the
// shape) or the fluctuation for one aircraft. See REFERENCE.md - Radar
// cross-section.
struct AircraftSignatureType {
    bool present = false;       // the type file has a "signature" block
    double staticRcsM2 = 0.0;  // nose-on mean RCS, derived from the shape at load
    FluctuationModel fluctuation = FluctuationModel::SWERLING_1;
    std::array<RcsEllipsoid, kMaxRcsEllipsoids> shape{};  // "rcs_shape"
    int shapeCount = 0;
};

// "swerling_1" or "steady" -> the model; anything else names `where`.
FluctuationModel ParseFluctuationModel(const std::string& name, const std::string& where);

struct AircraftType {
    std::string id;           // the directory name, and what scenarios name
    std::string displayName;  // for the GUI and reports
    std::string jsbsimModel;  // data/jsbsim_data/aircraft/<model>/<model>.xml

    double wingAreaFt2 = 0.0;
    AircraftBounds::HitBox hitBox;
    AircraftEnvelope envelope;
    AircraftControlInterface controls;
    AirborneRadarType radar;
    AircraftRwrType rwr;
    AircraftSignatureType signature;

    // Fuel tanks whose JSBSim name starts with this are internal; the rest
    // (drop tanks) are left as the model file declares them.
    std::string internalTankPrefix;

    // This type's own directory: data/aircraft/<id>. Reference data for
    // validation and model-fitting tools lives under it.
    std::filesystem::path directory;
};
