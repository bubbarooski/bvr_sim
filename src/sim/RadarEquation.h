#pragma once

#include "components/RadarCrossSection.h"
#include "sim/ReceiverBudget.h"
#include "sim/SimContext.h"
#include "util/Vec3.h"

// Radar physics shared by every sensor in the sim -- the airborne radar, the
// missile's active seeker, the RWR, and the renderer's range rings. Systems decide when to sweep and what a return means; this
// is only the physics. See REFERENCE.md - Radar.
namespace RadarEquation {

// The detection threshold, in dB over the receiver's noise power, that a
// square-law detector sets for false-alarm probability `pfa` on one look:
// noise alone exceeds T = ln(1/pfa) times its mean power with probability
// exactly pfa.
double DetectionThresholdDb(double pfa);

// One look's square-law detector output, in dB over the receiver's noise
// power: the target's return at the mean SNR of `rcs` (aspect already
// applied) at rangeM through `budget`, in free space, fluctuating as
// rcs.fluctuation says, plus complex Gaussian receiver
// noise. A look detects when the output reaches the tier's threshold.
// Draws four Gaussian samples (Swerling 1: the return's two quadratures,
// then the noise's) or two (steady: the noise's) -- callers must call it
// at most once per (radar, target) pair per sweep to keep the RNG stream
// deterministic. rangeM is floored at 1 m. See REFERENCE.md - Probability
// of detection.
double DetectorOutputDb(SimContext& simContext, const ReceiverBudget& budget, const RadarCrossSection& rcs,
                        double rangeM);

// A Swerling 1 target's single-look probability of detection at mean SNR
// meanSnrDb against thresholdDb: exp(-T / (1 + S)), exact for an
// exponentially distributed return in complex Gaussian noise.
double SwerlingOneDetectionProbability(double meanSnrDb, double thresholdDb);

// Its inverse: the mean SNR in dB at which a Swerling 1 look against
// thresholdDb detects with probability pd.
double SwerlingOneRequiredSnrDb(double thresholdDb, double pd);

// The mean SNR in dB for a target of rcsM2 at rangeM -- no fluctuation, no
// noise draw.
double SnrDb(const ReceiverBudget& budget, double rcsM2, double rangeM);

// The range, in meters, at which a target of rcsM2 gives exactly the mean
// SNR snrDb in free space -- the SNR form solved for range. 0 for a
// degenerate budget.
double ReferenceRangeM(const ReceiverBudget& budget, double rcsM2, double snrDb);

// One-way Friis link budget in dB, no RNG draw -- what an RWR receives from
// an illuminating radar (R^2 falloff, not the round trip's R^4), in free
// space. rangeM is floored at 1 m.
double FriisReceivedPowerDb(double txPowerW, double txAntennaGain, double rxAntennaGain, double wavelengthM,
                            double rangeM);

// Line-of-sight range over a curved Earth (4/3-radius refraction model), in
// meters, given both altitudes in meters.
double HorizonRangeM(double radarAltitudeM, double targetAltitudeM);

// Distance between two feet-space positions, converted to meters.
double RangeM(const Vec3& a, const Vec3& b);

// Nose direction from Transform.euler {roll, pitch, yaw} in degrees. Roll
// drops out: rotating about the forward axis doesn't move the nose.
Vec3 ForwardVector(const Vec3& eulerDeg);

// Projected area in m^2 of `count` body-axis ellipsoids along the unit
// body-frame direction bodyUnit (AircraftBounds axes: +X starboard, +Y up,
// +Z nose): pi sqrt((h l n_x)^2 + (s l n_y)^2 + (s h n_z)^2) summed over
// the parts, overlap ignored. See REFERENCE.md - Radar cross-section.
double ShapeProjectedAreaM2(const RcsEllipsoid* parts, int count, const Vec3& bodyUnit);

// The target's mean RCS in m^2 seen along targetToSensorUnit (world space),
// the target at attitude targetEulerDeg (Transform.euler): a shaped
// target's nose-on staticRcsM2 scaled by its projected area toward the
// sensor over its nose-on area -- so heading, pitch and bank all count --
// or an unshaped target's staticRcsM2 from every direction. An ESTIMATE
// from geometry, not RCS data.
double MeanRcsM2(const RadarCrossSection& rcs, const Vec3& targetEulerDeg, const Vec3& targetToSensorUnit);

// Pulse-Doppler clutter notch: true if the target's GROUND-relative radial
// velocity along `losUnit` is under the notch threshold, which hides it in
// the ground-clutter filter. See REFERENCE.md - Pulse-Doppler clutter notch.
bool InDopplerNotch(const Vec3& targetVelFtPerSec, const Vec3& losUnit, double notchThresholdMps);

// True if a beam pointed along losUnit from radarAltitudeM, reaching
// beamwidthRad either side of boresight, touches the 4/3-Earth surface --
// i.e. the beam sees ground behind the target, main-beam clutter for it to
// hide in. Its lower edge must dip below the horizon (the same Earth model
// as HorizonRangeM). See REFERENCE.md - Pulse-Doppler clutter notch.
bool BeamOnGround(double radarAltitudeM, const Vec3& losUnit, double beamwidthRad);

}  // namespace RadarEquation
