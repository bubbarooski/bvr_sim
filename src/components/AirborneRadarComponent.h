#pragma once

#include "ecs/World.h"
#include "sim/ReceiverBudget.h"
#include "util/Numeric.h"

// Forward-facing airborne radar carried by an aircraft. Starts as its
// aircraft type's radar (AircraftEntity::MakeAirborneRadar); a scenario may
// override any field for one aircraft. Detection is SNR against the
// receiver's noise floor, through the same RCS component every sensor
// reads -- only the FOV gating, the frames and the track states are
// particular to it. See REFERENCE.md - Radar.
//
// Every number below is zero here, not a default radar: a radar is built
// from its type (AircraftEntity::MakeAirborneRadar).
struct AirborneRadarComponent {
    // One search / TWS frame: every target in the cone gets one look, on
    // the same tick, once per frame. See REFERENCE.md - Frames and track
    // confirmation.
    double frameTimeS = 0.0;
    // Single-target track (STT): while sttTarget is set, that one target is
    // looked at once per sttFrameTimeS instead, and every other target gets
    // no looks at all -- its track coasts on the scan frames and drops.
    // Fire control (LaunchSystem) sets it; the radar clears it when the
    // lock's track drops. See REFERENCE.md - Single-target track.
    double sttFrameTimeS = 0.0;
    EntityID sttTarget = kNoEntity;

    // The receiver budget (see ReceiverBudget.h), plus the peak power a
    // radar warning receiver hears.
    double averagePowerW = 0.0;
    double peakPowerW = 0.0;
    double antennaGain = 0.0;
    double frequencyHz = 0.0;
    double noiseFigureDb = 0.0;
    double timeOnTargetS = 0.0;
    double systemLossDb = 0.0;

    // The teaching knob: where this radar detects the 5 m2 reference
    // target, against the type's fitted range. The receiver gains
    // ReferenceRangeGainDb of the two; equal (the default) changes nothing.
    double referenceRangeNmi = 0.0;
    double typeReferenceRangeNmi = 0.0;

    double fovHalfAngleDeg = 0.0;  // symmetric cone half-angle around the nose
    double searchThresholdDb = 0.0;  // the one detection threshold, ln(1/pfa) over the noise

    // A track confirms at confirmFrames detections in the last
    // confirmWindowFrames frames, and a confirmed track drops after
    // dropFrames frames in a row without one -- sttDropFrames STT frames
    // while it is locked.
    int confirmFrames = 0;
    int confirmWindowFrames = 0;
    int dropFrames = 0;
    int sttDropFrames = 0;
    // The launch gate must stay open this long before the missile fires.
    double fireControlDelayS = 0.0;

    // Pulse-Doppler clutter notch: ground-relative radial velocity below
    // this is a forced miss regardless of SNR. It filters a near-zero
    // closing RATE (beam aspect), not slow targets in general -- and only
    // while the beam, one elevation beamwidth either side of the line of
    // sight, touches the ground: with sky behind the whole beam there is no
    // main-beam clutter to hide in.
    double notchVelocityThresholdMps = 0.0;
    double antennaHeightM = 0.0;  // aperture height, sets the beam's vertical size
    // Blind range: a pulse radar cannot listen while transmitting, so
    // detection fails below this regardless of SNR. A threshold gate.
    double minRangeM = 0.0;

    // Measurement and tracking (REFERENCE.md - Track measurement and
    // filtering): each detection measures range and two angles with errors
    // ~ resolution / sqrt(SNR), filtered into a constant-velocity track.
    double antennaWidthM = 0.0;    // aperture width; with antennaHeightM, the beam
    double beamwidthFactor = 0.0;  // half-power beamwidth = this x lambda / aperture
    double monopulseSlope = 0.0;   // angle sigma = beamwidth / (this x sqrt(2 SNR))
    double pulseWidthS = 0.0;      // range sigma = c x this / (2 sqrt(2 SNR))
    double trackManeuverG = 0.0;   // tracker's 1-sigma target acceleration, g

    double lastFrameTime = Numeric::kLongAgoS;
    double lastSttFrameTime = Numeric::kLongAgoS;  // reset on each lock
};

inline ReceiverBudget ReceiverOf(const AirborneRadarComponent& radar) {
    return ReceiverBudget{radar.averagePowerW, radar.antennaGain,   WavelengthM(radar.frequencyHz),
                          radar.noiseFigureDb, radar.timeOnTargetS,
                          radar.systemLossDb - ReferenceRangeGainDb(radar.referenceRangeNmi,
                                                                    radar.typeReferenceRangeNmi)};
}
