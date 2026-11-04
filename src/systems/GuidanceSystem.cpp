#include "GuidanceSystem.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/DatalinkTrackComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "missile/MissileEntity.h"
#include "missile/MissileType.h"
#include "sim/Atmosphere.h"
#include "sim/Environment.h"
#include "sim/FlightEnvelope.h"
#include "sim/HitWindow.h"
#include "sim/RadarEquation.h"
#include "sim/RadarTrackFilter.h"
#include "sim/ResponseLag.h"
#include "util/EntityVelocity.h"
#include "util/Numeric.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace {

using Units::kFeetPerMeter;
using Units::kGravityFtPerSec2;
using Units::kRadToDeg;

// Fallback clamp for missiles with no MissileGLimitComponent.
constexpr double kMaxAccelG = 30.0;
constexpr double kMaxAccelFtPerSec2 = kMaxAccelG * kGravityFtPerSec2;

// Below this the velocity has no usable direction: no drag to apply, no
// axis to thrust along, and no body frame for the seeker cone.
constexpr double kMinSpeedFtPerSec = 0.001;

// |up x axis| below this: flying straight up or down, with no "sideways".
constexpr double kVerticalSideNorm = 1e-6;

// Flat-Earth gravity, world Y up. Acts on the missile in every branch,
// guided or not.
const Eigen::Vector3d kGravityAccel(0.0, -kGravityFtPerSec2, 0.0);

// The lift that cancels gravity's component across the body axis (the air
// velocity). Only that component: the part along the axis is not
// something lift can act on, so a climbing missile still decelerates by
// g sin(gamma). See REFERENCE.md 6.8 - Gravity and gravity bias.
Eigen::Vector3d GravityBias(const Eigen::Vector3d& airVel, double airSpeed) {
    if (airSpeed <= kMinSpeedFtPerSec) return Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = airVel / airSpeed;
    Eigen::Vector3d gravityAcross = kGravityAccel - kGravityAccel.dot(axis) * axis;
    return -gravityAcross;
}

// Turns a steering command into one the fins can fly. Lift acts only across
// the body axis (the air velocity), but true PN commands along the LOS-rate
// direction, which tilts off that plane whenever the missile leads its
// target -- and the part along the axis was free thrust or free braking.
// Rotated into the perpendicular plane keeping its magnitude, so the
// autopilot still pulls the G guidance asked for. A command lying along the
// axis has no perpendicular direction to rotate into, and lift cannot
// supply it, so it is dropped. See REFERENCE.md - Steering across the body
// axis.
Eigen::Vector3d AcrossBodyAxis(const Eigen::Vector3d& command, const Eigen::Vector3d& airVel, double airSpeed) {
    if (airSpeed <= kMinSpeedFtPerSec) return Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = airVel / airSpeed;
    Eigen::Vector3d across = command - command.dot(axis) * axis;
    double acrossMag = across.norm();
    double commandMag = command.norm();
    if (acrossMag <= Numeric::kNegligibleFraction * commandMag) return Eigen::Vector3d::Zero();
    return across * (commandMag / acrossMag);
}

// True PN's steering command, before gravity bias and the G clamp.
// `losRate` is the rate of change of the LOS unit vector (what the seeker
// reports). See REFERENCE.md - Steering across the body axis.
Eigen::Vector3d GuidanceCommand(const GuidanceComponent& guidance, double vClosing, const Eigen::Vector3d& losRate,
                                const Eigen::Vector3d& airVel, double airSpeed) {
    return AcrossBodyAxis(guidance.navGain * vClosing * losRate, airVel, airSpeed);
}

// The loft's steering command: across the body axis, the sideways part of
// PN's command (horizontal, so the missile still makes good its bearing to
// the target) plus a vertical part that brings the air flight-path angle to
// the climb angle as a first-order lag, V (gamma_L - gamma) / tau. Flying
// straight up there is no "sideways", so PN's command is left alone. See
// REFERENCE.md - Loft.
Eigen::Vector3d LoftCommand(const Eigen::Vector3d& pnCommand, double climbDeg, double climbTimeConstantS,
                            const Eigen::Vector3d& airVel, double airSpeed) {
    if (airSpeed <= kMinSpeedFtPerSec) return Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = airVel / airSpeed;
    Eigen::Vector3d side = Eigen::Vector3d::UnitY().cross(axis);
    double sideNorm = side.norm();
    if (sideNorm <= kVerticalSideNorm) return pnCommand;
    side /= sideNorm;
    Eigen::Vector3d normal = axis.cross(side);  // in the vertical plane, upward
    double gammaRad = std::asin(std::clamp(axis.y(), -1.0, 1.0));
    double climbRad = climbDeg / kRadToDeg;
    return pnCommand.dot(side) * side + (airSpeed * (climbRad - gammaRad) / climbTimeConstantS) * normal;
}

// The active seeker's Doppler notch: the aircraft radar's rule, seen from
// the missile. The seeker is a pulse-Doppler radar too, so the missile's own
// speed cancels just as the launcher's does -- what counts is the target's
// GROUND-relative radial velocity, and only while the seeker's beam (wide,
// from its small dish) touches the ground. See REFERENCE.md - The active
// seeker.
bool SeekerNotched(const GuidanceComponent& guidance, const Vec3& missilePosFt, const Vec3& targetPosFt,
                   const Vec3& targetVelFtPerSec) {
    if (guidance.seekerNotchVelocityThresholdMps <= 0.0) return false;
    Vec3 missileToTarget = targetPosFt - missilePosFt;
    double rangeFt = missileToTarget.norm();
    if (rangeFt <= 0.0) return false;
    Vec3 losUnit = missileToTarget * (1.0 / rangeFt);
    double beamwidthRad = RadarTrackFilter::BeamwidthRad(WavelengthM(guidance.seekerFrequencyHz),
                                                         guidance.seekerAntennaDiameterM, guidance.seekerBeamwidthFactor);
    return RadarEquation::BeamOnGround(missilePosFt.y() / kFeetPerMeter, losUnit, beamwidthRad) &&
           RadarEquation::InDopplerNotch(targetVelFtPerSec, losUnit, guidance.seekerNotchVelocityThresholdMps);
}

// One sweep of the missile's own active radar seeker. Returns true only on
// CONFIRMED acquisition (M-of-N), not on a single strong return. A notched
// sweep is a miss; the blind-range gate is deliberately not applied -- see
// REFERENCE.md - The active seeker.
bool ActiveSeekerSweep(World& world, SimContext& simContext, GuidanceComponent& guidance, EntityID missile,
                        double simTime) {
    if (!Numeric::PeriodElapsed(simTime, guidance.seekerLastSweepTime, guidance.seekerSweepPeriodS)) return false;
    guidance.seekerLastSweepTime = simTime;

    // Every early-out below records a MISS sweep rather than leaving the
    // window untouched, so stale hits cannot sit in it indefinitely.
    auto recordSweep = [&guidance](bool detected) {
        // M-of-N acquisition, the type's. See REFERENCE.md - The active seeker.
        HitWindow::Push(guidance.seekerSweepHistoryBits, detected, guidance.seekerConfirmWindow);
        return HitWindow::Count(guidance.seekerSweepHistoryBits) >= guidance.seekerConfirmHits;
    };

    ComponentStore<Transform>& transforms = world.Components<Transform>();
    // A target with no RadarCrossSection is invisible to radar.
    const RadarCrossSection* targetRcs = world.Components<RadarCrossSection>().TryGet(guidance.target);
    const Transform* targetTransformPtr = transforms.TryGet(guidance.target);
    if (targetRcs == nullptr || targetTransformPtr == nullptr) return recordSweep(false);

    const Transform& missileTransform = transforms.Get(missile);
    const Transform& targetTransform = *targetTransformPtr;

    Vec3 missileToTarget = targetTransform.pos - missileTransform.pos;
    double rangeFt = missileToTarget.norm();
    if (rangeFt <= 0.0) return recordSweep(true);  // co-located: trivially acquired
    double rangeM = rangeFt / kFeetPerMeter;

    // The RCS the target's attitude shows the seeker, on a local copy, same
    // as the aircraft radar.
    Vec3 targetToMissileUnit = missileToTarget * (-1.0 / rangeFt);
    RadarCrossSection adjustedRcs = *targetRcs;
    adjustedRcs.staticRcsM2 = RadarEquation::MeanRcsM2(*targetRcs, targetTransform.euler, targetToMissileUnit);

    const ReceiverBudget seeker = SeekerReceiverOf(guidance);
    double snrDb = RadarEquation::DetectorOutputDb(simContext, seeker, adjustedRcs, rangeM);

    double horizonRangeM =
        RadarEquation::HorizonRangeM(missileTransform.pos.y() / kFeetPerMeter, targetTransform.pos.y() / kFeetPerMeter);
    if (rangeM > horizonRangeM) return recordSweep(false);
    // After the SNR draw, like the horizon, so the RNG stream does not
    // depend on geometry.
    if (SeekerNotched(guidance, missileTransform.pos, targetTransform.pos, EntityVelocity::Get(world, guidance.target)))
        return recordSweep(false);

    return recordSweep(snrDb >= guidance.seekerThresholdDb);
}

// MIDCOURSE_ARH's own-seeker acquisition just landed this tick: latches
// seekerActive/navHandoffPending, logs SEEKER_ACTIVE, and records the
// midcourse error (the datalinked track's) at the moment of handoff --
// only the first such moment per engagement.
void HandleSeekerAcquisition(SimContext& simContext, GuidanceComponent& guidance, EntityID missile, double simTime,
                              const Eigen::Vector3d& missileTruePos, const Eigen::Vector3d& targetTruePos,
                              ComponentStore<DatalinkTrackComponent>& datalinks) {
    guidance.seekerActive = true;
    guidance.navHandoffPending = true;

    // The midcourse error: how wrong the position the missile was steering
    // to actually was. It sets where the seeker was looking, and so whether
    // it acquired at all.
    double datalinkErrorFt = -1.0;
    double datalinkStalenessS = -1.0;
    const DatalinkTrackComponent* datalink = datalinks.TryGet(missile);
    if (datalink != nullptr && datalink->hasTrack) {
        datalinkErrorFt = (datalink->estimatedPos - targetTruePos).norm();
        datalinkStalenessS = datalink->stalenessS;
    }

    // True geometric range, not the datalinked `los`: this reports where
    // the seeker physically acquired.
    simContext.eventLog.Record(SimEvent{simTime, "SEEKER_ACTIVE", missile,
                                          "target=" + std::to_string(guidance.target) +
                                              " range_ft=" + std::to_string((targetTruePos - missileTruePos).norm()) +
                                              " datalink_error_ft=" + std::to_string(datalinkErrorFt)});
    if (simContext.outcome.seekerActiveSimTime < 0.0) {
        simContext.outcome.seekerActiveSimTime = simTime;
        simContext.outcome.datalinkErrorAtHandoffFt = datalinkErrorFt;
        simContext.outcome.datalinkStalenessAtHandoffS = datalinkStalenessS;
    }
}

// Atmospheric drag, opposing the velocity through the AIR (`vel` is ground
// velocity minus the wind, `speed` its magnitude). See REFERENCE.md -
// Aerodynamic drag. Shared by the guided and ballistic-coast branches below.
// `aero` is the missile type's; a missile with no type (a synthetic test
// missile) is a drag-free point mass.
Eigen::Vector3d ComputeDrag(const MissileAerodynamics* aero, const Eigen::Vector3d& vel, double speed,
                             double altitudeFt, double referenceAreaFt2, double massSlug, double temperatureOffsetC) {
    if (aero == nullptr || speed <= kMinSpeedFtPerSec) return Eigen::Vector3d::Zero();
    Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(altitudeFt, temperatureOffsetC);
    double mach = speed / atmosphere.speedOfSoundFtPerSec;
    double cd = aero->ZeroLiftCd(mach);
    double dynamicPressure = 0.5 * atmosphere.densitySlugPerFt3 * speed * speed;
    double dragForceLbf = dynamicPressure * cd * referenceAreaFt2;
    double dragAccelMag = dragForceLbf / massSlug;
    return -(dragAccelMag / speed) * vel;
}

// The motor's thrust along the body axis (the air velocity): boost, then
// sustain, then nothing. Whether or not the missile is still guiding -- a
// shot that resolves as a miss mid-burn keeps burning.
Eigen::Vector3d ThrustAccel(const GuidanceComponent& guidance, double timeSinceLaunch, const Eigen::Vector3d& airVel,
                            double airSpeed, double massSlug) {
    double thrustLbf = 0.0;
    if (timeSinceLaunch < guidance.boostDurationS) {
        thrustLbf = guidance.boostThrustLbf;
    } else if (timeSinceLaunch < guidance.boostDurationS + guidance.sustainDurationS) {
        thrustLbf = guidance.sustainThrustLbf;
    }
    if (thrustLbf <= 0.0 || airSpeed <= kMinSpeedFtPerSec) return Eigen::Vector3d::Zero();
    return (airVel / airSpeed) * (thrustLbf / massSlug);
}

// Drag due to lift: the normal force that delivers `liftAccel` acts along
// the body's normal, which an angle of attack alpha tilts back from the
// flight path, so N sin(alpha) of it opposes the motion. alpha comes from
// inverting CN = m |a_n| / (q S_ref); mass cancels in the result, |a_n|
// sin(alpha). Zero with no lift or no normal-force model. Untrimmed, and
// the change of axial force with alpha is ignored. See REFERENCE.md - Drag
// due to lift.
Eigen::Vector3d DragDueToLift(const MissileNormalForce* normalForce, const Eigen::Vector3d& liftAccel,
                              const Eigen::Vector3d& airVel, double airSpeed, double altitudeFt, double massSlug,
                              double temperatureOffsetC) {
    if (normalForce == nullptr || airSpeed <= kMinSpeedFtPerSec) return Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = airVel / airSpeed;
    double normalAccel = (liftAccel - liftAccel.dot(axis) * axis).norm();
    if (normalAccel <= 0.0) return Eigen::Vector3d::Zero();
    Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(altitudeFt, temperatureOffsetC);
    double dynamicPressure = 0.5 * atmosphere.densitySlugPerFt3 * airSpeed * airSpeed;
    double cn = massSlug * normalAccel / (dynamicPressure * normalForce->ReferenceAreaFt2());
    double alphaRad = normalForce->AlphaForCn(cn, airSpeed / atmosphere.speedOfSoundFtPerSec);
    return -normalAccel * std::sin(alphaRad) * axis;
}

}  // namespace

namespace GuidanceSystem {

// Recomputed each tick rather than stored -- it is fully determined by
// elapsed time.
double CurrentMassSlug(const GuidanceComponent& guidance, double timeSinceLaunch) {
    double burnDurationS = guidance.boostDurationS + guidance.sustainDurationS;
    double burnFractionConsumed = burnDurationS > 0.0 ? std::clamp(timeSinceLaunch / burnDurationS, 0.0, 1.0) : 1.0;
    return guidance.dryMassSlug + guidance.propellantMassSlug * (1.0 - burnFractionConsumed);
}

void Update(World& world, SimContext& simContext, double dt, double simTime) {
    if (dt <= 0.0) return;

    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    ComponentStore<KinematicsComponent>& kinematics = world.Components<KinematicsComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    ComponentStore<MissileGLimitComponent>& missileGLimits = world.Components<MissileGLimitComponent>();
    ComponentStore<DatalinkTrackComponent>& datalinks = world.Components<DatalinkTrackComponent>();
    ComponentStore<AirborneRadarComponent>& radars = world.Components<AirborneRadarComponent>();
    const double temperatureOffsetC = simContext.environment.temperatureOffsetC;
    // The missile is a point mass that always points into its relative
    // wind, so its velocity through the air stands in for its body axis:
    // that is what drag, Mach, the G envelope, thrust, the seeker's
    // boresight and lift all act along. Guidance geometry (LOS,
    // closing speed) stays in the ground frame the target is tracked in.
    const Environment& environment = simContext.environment;

    for (auto& [entity, guidance] : guidances) {
        KinematicsComponent* missileKinematics = kinematics.TryGet(entity);
        const Transform* missileTransform = transforms.TryGet(entity);
        if (missileKinematics == nullptr || missileTransform == nullptr) continue;
        // Destroyed means merged with the target; nothing left to fly.
        if (destroyed.Has(entity)) continue;
        // The steady wind, the same at every altitude.
        const Eigen::Vector3d windFps = WindFps(environment);
        const MissileType* type = MissileEntity::TypeOf(world, entity);
        const MissileAerodynamics* aero = type != nullptr ? &type->aero : nullptr;

        if (!guidance.active) {
            // Resolved without a hit: no more steering, but the missile
            // flies on under its motor (if still burning), gravity and drag
            // rather than vanishing, until MissileSelfDestructSystem removes
            // it.
            Eigen::Vector3d airVel = missileKinematics->vel - windFps;
            double airSpeed = airVel.norm();
            double coastTimeS = simTime - guidance.launchSimTime;
            double coastMassSlug = CurrentMassSlug(guidance, coastTimeS);
            Eigen::Vector3d dragAccel = ComputeDrag(aero, airVel, airSpeed, missileTransform->pos.y(),
                                                     guidance.referenceAreaFt2, coastMassSlug, temperatureOffsetC);
            missileKinematics->accel =
                kGravityAccel + dragAccel + ThrustAccel(guidance, coastTimeS, airVel, airSpeed, coastMassSlug);
            continue;
        }

        const Transform* targetTransform = transforms.TryGet(guidance.target);
        if (targetTransform == nullptr) continue;

        // The missile knows its own position: its midcourse error is the
        // relayed radar track's, below. See REFERENCE.md - Datalink target
        // state.
        Eigen::Vector3d missileTruePos = missileTransform->pos;

        // MIDCOURSE_ARH before acquisition steers on the launcher's
        // datalinked track; once its own seeker is measuring (and ARH and
        // SARH always) it steers on the target itself.
        bool navigatingInertially = NavigatesInertially(guidance);
        Eigen::Vector3d missileVel = missileKinematics->vel;

        // The target's TRUE state, which the missile only has access to
        // once its own seeker is measuring. Kept separately because the
        // seeker-acquisition test and the SEEKER_ACTIVE range report are
        // physical facts about where things are, not about what the
        // missile believes.
        Eigen::Vector3d targetTruePos = targetTransform->pos;
        Eigen::Vector3d targetTrueVel = EntityVelocity::Get(world, guidance.target);

        // Midcourse steers to what the launcher radioed over, not to
        // truth. See REFERENCE.md - Datalink target state.
        Eigen::Vector3d targetPos = targetTruePos;
        Eigen::Vector3d targetVel = targetTrueVel;
        const DatalinkTrackComponent* datalink = navigatingInertially ? datalinks.TryGet(entity) : nullptr;
        if (datalink != nullptr && datalink->hasTrack) {
            targetPos = datalink->estimatedPos;
            targetVel = datalink->estimatedVel;
        }

        Eigen::Vector3d los = targetPos - missileTruePos;
        Eigen::Vector3d losUnit = los.normalized();

        // Positive when range is shrinking. No extra negation -- see
        // REFERENCE.md - Closing velocity for the derivation and for what
        // a sign flip here does to the loop.
        double vClosing = losUnit.dot(missileVel - targetVel);

        // Finite-difference LOS rate. prevLosUnit is seeded at launch;
        // navHandoffPending covers the jump from the datalinked track to
        // the seeker's own measurement, which would otherwise read as real
        // angular motion.
        Eigen::Vector3d losRate = guidance.navHandoffPending
                                       ? Eigen::Vector3d(Eigen::Vector3d::Zero())
                                       : Eigen::Vector3d((losUnit - guidance.prevLosUnit) / dt);
        guidance.navHandoffPending = false;
        // Updated every tick regardless of lock state, so reacquiring
        // after a coast doesn't see one huge multi-tick jump.
        guidance.prevLosUnit = losUnit;

        Eigen::Vector3d airVel = missileVel - windFps;
        double airSpeed = airVel.norm();
        double timeSinceLaunch = simTime - guidance.launchSimTime;
        double currentMassSlug = CurrentMassSlug(guidance, timeSinceLaunch);

        // Seeker gimbal cone, memory coast, and (for SARH/MIDCOURSE_ARH)
        // the launcher-illumination requirement all share one lock state
        // machine. See REFERENCE.md - Seeker Modeling.
        Eigen::Vector3d effectiveLosRate = losRate;
        double boresightErrorDeg = 0.0;
        if (airSpeed > kMinSpeedFtPerSec) {
            double cosBoresight = (airVel / airSpeed).dot(losUnit);
            cosBoresight = std::clamp(cosBoresight, -1.0, 1.0);
            boresightErrorDeg = std::acos(cosBoresight) * kRadToDeg;
            bool inCone = boresightErrorDeg <= guidance.seekerFovHalfAngleDeg;

            // MIDCOURSE_ARH tries its own seeker every sweep until it
            // acquires, then latches autonomous for the rest of the
            // flight. Gated on inCone -- the antenna is gimbal-limited.
            if (guidance.seekerMode == SeekerMode::MIDCOURSE_ARH && !guidance.seekerActive && inCone &&
                ActiveSeekerSweep(world, simContext, guidance, entity, simTime)) {
                HandleSeekerAcquisition(simContext, guidance, entity, simTime, missileTruePos, targetTruePos,
                                         datalinks);
            }

            // SARH homes on the launcher's single-target-track lock for its
            // whole flight. MIDCOURSE_ARH, until its own seeker goes active,
            // needs the launcher's confirmed track (the datalink's source) or
            // a datalink estimate already received: when the link is lost it
            // flies on to the last estimate, extrapolated (DatalinkSystem),
            // until its seeker acquires. See REFERENCE.md - Datalink target
            // state.
            bool illuminated = true;
            if (guidance.seekerMode == SeekerMode::SARH) {
                const AirborneRadarComponent* launcherRadar = radars.TryGet(guidance.launcher);
                illuminated = launcherRadar != nullptr && launcherRadar->sttTarget == guidance.target;
            } else if (guidance.seekerMode == SeekerMode::MIDCOURSE_ARH && !guidance.seekerActive) {
                const AirborneRadarTrack* track =
                    simContext.airborneRadarTracks.Find(guidance.launcher, guidance.target);
                illuminated = (track != nullptr && track->state == RadarTrackState::CONFIRMED) ||
                              (datalink != nullptr && datalink->hasTrack);
            }
            // The gimbal only limits a seeker that is measuring. Before its
            // own seeker goes active, MIDCOURSE_ARH steers on the datalinked
            // track from its inertial position, so a target outside the
            // seeker's cone (a high off-boresight launch) is still guided
            // on -- only acquisition, above, waits for the cone.
            bool seekerMeasures = !navigatingInertially;
            // A measuring active seeker loses the target in its Doppler
            // notch; memory coast rides through a brief one. SARH's notch is
            // the launcher's: its STT lock breaks, and with it illumination.
            bool notched = seekerMeasures && guidance.seekerMode != SeekerMode::SARH &&
                           SeekerNotched(guidance, missileTransform->pos, targetTransform->pos, targetTrueVel);
            bool hasValidLock = (inCone || !seekerMeasures) && illuminated && !notched;

            if (hasValidLock) {
                guidance.hasLock = true;
                guidance.timeSinceLockLostS = 0.0;
                guidance.lastLosRate = losRate;
            } else if (guidance.timeSinceLockLostS < guidance.memoryCoastTimeS) {
                // Lock lost but still inside the coast window: steer on
                // the frozen last real LOS rate.
                effectiveLosRate = guidance.lastLosRate;
                guidance.timeSinceLockLostS += dt;
            } else {
                // Datalink fallback: a MIDCOURSE_ARH missile whose own seeker
                // has lost the target, while the launcher still tracks it,
                // hands back to datalink midcourse and must acquire again
                // rather than going ballistic. See REFERENCE.md 8.2.
                const AirborneRadarTrack* launcherTrack =
                    simContext.airborneRadarTracks.Find(guidance.launcher, guidance.target);
                bool fallBack = guidance.seekerMode == SeekerMode::MIDCOURSE_ARH && guidance.seekerActive &&
                                launcherTrack != nullptr && launcherTrack->state == RadarTrackState::CONFIRMED &&
                                launcherTrack->estimate.valid;
                if (guidance.hasLock) {
                    std::string reason;
                    if (!inCone && seekerMeasures) {
                        reason = "boresight_error_deg=" + std::to_string(boresightErrorDeg);
                    } else if (notched) {
                        reason = "doppler_notch";
                    } else if (guidance.seekerMode == SeekerMode::MIDCOURSE_ARH) {
                        reason = "midcourse_track_lost_before_seeker_active";
                    } else {
                        reason = "sarh_illumination_lost";
                    }
                    if (fallBack) reason += " fallback=datalink";
                    simContext.eventLog.Record(SimEvent{simTime, "SEEKER_LOST", entity, reason});
                    if (simContext.outcome.seekerLostSimTime < 0.0) simContext.outcome.seekerLostSimTime = simTime;
                }
                if (fallBack) {
                    guidance.seekerActive = false;
                    guidance.seekerSweepHistoryBits = 0;  // a fresh M-of-N acquisition
                    // Steering moves from the seeker's relative measurement
                    // back to the inertial position: the LOS jumps by the
                    // navigation error, which is not angular motion.
                    guidance.navHandoffPending = true;
                    guidance.hasLock = true;
                    guidance.timeSinceLockLostS = 0.0;
                } else {
                    guidance.hasLock = false;
                }
                effectiveLosRate = Eigen::Vector3d::Zero();  // ballistic, or one tick of handback
            }
        }

        // The loft ends for good once the target lies the climb angle below
        // the missile's horizon, once the missile stops steering on the
        // datalink, or once -- burnt out -- the air is too thin to leave it
        // the G it needs to steer in the endgame. The launch turn re-arms to
        // push it over onto the target, well before its seeker needs the
        // target inside the cone. See REFERENCE.md - Loft.
        if (guidance.lofting) {
            double depressionDeg = -std::asin(std::clamp(losUnit.y(), -1.0, 1.0)) * kRadToDeg;
            bool burntOut = timeSinceLaunch >= guidance.boostDurationS + guidance.sustainDurationS;
            const MissileGLimitComponent* gLimit = missileGLimits.TryGet(entity);
            bool airTooThin = burntOut && gLimit != nullptr &&
                              FlightEnvelope::GetMissileMaxAvailableG(airSpeed, missileTruePos.y(), currentMassSlug,
                                                                      *gLimit, temperatureOffsetC) <
                                  guidance.loftMinAvailableG;
            if (depressionDeg >= guidance.loftClimbDeg || !navigatingInertially || airTooThin) {
                guidance.lofting = false;
                guidance.launchTurnDone = false;
                simContext.eventLog.Record(SimEvent{simTime, "LOFT_END", entity,
                                                      "altitude_ft=" + std::to_string(missileTruePos.y()) +
                                                          " range_ft=" + std::to_string(los.norm())});
            }
        }

        // The launch turn: until the target first comes within the exit
        // angle of the velocity vector, turn toward it at the G the airframe
        // has instead of flying the guidance law. Ends for good the first
        // time it is inside. See REFERENCE.md 6.9 - Launch turn.
        bool launchTurning = false;
        if (!guidance.launchTurnDone) {
            if (guidance.launchTurnExitDeg <= 0.0 || boresightErrorDeg <= guidance.launchTurnExitDeg) {
                guidance.launchTurnDone = true;
            } else {
                launchTurning = guidance.hasLock && airSpeed > kMinSpeedFtPerSec;
            }
        }

        // PN command plus the autopilot's gravity bias, clamped together
        // to the missile's own V-n envelope: the lift holding the airframe
        // up is lift the fins cannot also spend turning. Missiles without
        // the component fall back to the flat structural clamp. The bias
        // stays on after a lost lock -- the autopilot keeps flying the
        // airframe, only the PN steering stops. See REFERENCE.md 6.8 -
        // Gravity and gravity bias.
        double maxAccelFtPerSec2 = kMaxAccelFtPerSec2;
        double maxG = maxAccelFtPerSec2 / kGravityFtPerSec2;
        const MissileGLimitComponent* missileGLimit = missileGLimits.TryGet(entity);
        if (missileGLimit != nullptr) {
            maxG = FlightEnvelope::GetMissileMaxAvailableG(airSpeed, missileTruePos.y(), currentMassSlug,
                                                           *missileGLimit, temperatureOffsetC);
            maxAccelFtPerSec2 = maxG * kGravityFtPerSec2;
        }
        // The launch turn asks for everything the airframe has, toward the
        // line of sight -- on purpose, so it is not reported as guidance
        // overdemanding the airframe.
        // The loft climbs after any launch turn, and only while the
        // datalinked track is held: without it there is nothing to steer on.
        bool loftClimbing = guidance.lofting && !launchTurning && guidance.hasLock;
        Eigen::Vector3d accelCmd =
            launchTurning ? Eigen::Vector3d(AcrossBodyAxis(losUnit, airVel, airSpeed) * maxAccelFtPerSec2)
                          : GuidanceCommand(guidance, vClosing, effectiveLosRate, airVel, airSpeed);
        if (loftClimbing) {
            accelCmd =
                LoftCommand(accelCmd, guidance.loftClimbDeg, guidance.loftClimbTimeConstantS, airVel, airSpeed);
        }
        accelCmd += GravityBias(airVel, airSpeed);
        double accelMag = accelCmd.norm();
        // Like the launch turn, the loft's pull-up is the autopilot's own
        // demand, not guidance overdemanding the airframe.
        if (missileGLimit != nullptr && !launchTurning && !loftClimbing) {
            if (accelMag > maxAccelFtPerSec2) {
                simContext.eventLog.Record(
                    SimEvent{simTime, "MISSILE_G_LIMIT_EXCEEDED", entity,
                             "commanded_g=" + std::to_string(accelMag / kGravityFtPerSec2) +
                                 " available_g=" + std::to_string(maxG)});
            }
        }
        if (accelMag > maxAccelFtPerSec2) {
            accelCmd *= maxAccelFtPerSec2 / accelMag;
        }

        // What the airframe actually delivers, which is not quite what was
        // just commanded. Deliberately downstream of the clamp above: that
        // clamp is the autopilot limiting its own demand, so
        // MISSILE_G_LIMIT_EXCEEDED keeps meaning "PN asked for more than
        // the airframe has". The guidance response time is seeker, fins and
        // airframe as one lag (REFERENCE.md - Guidance response time). It
        // runs even when effectiveLosRate was zeroed by a lost seeker: the
        // fins still take time to come back.
        accelCmd = ResponseLag::Step(guidance.achievedAccel, accelCmd, airVel, guidance.responseTimeS, dt);

        // Re-clamped silently. The lift held from earlier ticks was legal
        // then, but the ceiling falls as the missile slows, and no lag makes
        // a missile exceed its own aerodynamic limit.
        double achievedMag = accelCmd.norm();
        if (achievedMag > maxAccelFtPerSec2) {
            accelCmd *= maxAccelFtPerSec2 / achievedMag;
        }

        // Axial thrust along the body axis (the air velocity). Motor thrust and
        // airframe G-capability are separate constraints, so this is not
        // subject to the clamp above -- it adds on top.
        Eigen::Vector3d thrustAccel = ThrustAccel(guidance, timeSinceLaunch, airVel, airSpeed, currentMassSlug);

        Eigen::Vector3d dragAccel = ComputeDrag(aero, airVel, airSpeed, missileTruePos.y(),
                                                 guidance.referenceAreaFt2, currentMassSlug, temperatureOffsetC);
        // From the lift the airframe actually delivers this tick, after the
        // lag and both clamps, gravity bias included.
        dragAccel += DragDueToLift(missileGLimit != nullptr ? missileGLimit->normalForce.get() : nullptr, accelCmd,
                                   airVel, airSpeed, missileTruePos.y(), currentMassSlug, temperatureOffsetC);

        missileKinematics->accel = accelCmd + thrustAccel + dragAccel + kGravityAccel;
    }
}

}  // namespace GuidanceSystem
