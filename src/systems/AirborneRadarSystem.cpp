#include "AirborneRadarSystem.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/JSBSimComponent.h"
#include "components/KinematicsComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Transform.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "sim/HitWindow.h"
#include "sim/RadarEquation.h"
#include "sim/RadarTrackFilter.h"
#include "util/EntityVelocity.h"
#include "util/Units.h"
#include "util/Vec3.h"
#include "util/Numeric.h"

namespace {

using Units::kFeetPerMeter;
using Units::kGravityFtPerSec2;
using Units::kRadToDeg;

// No return at all this frame (outside the cone, gated, or a dead target):
// below the detection threshold.
constexpr double kNoReturnDb = -std::numeric_limits<double>::infinity();
// A new track's velocity is unknown: 1-sigma per axis, about Mach 2, so the
// first few detections set it rather than this guess.
constexpr double kInitialVelSigmaFtPerSec = 2000.0;

// Whether `owner`'s radar looking at `target` is the look the outcome's
// first-detection time belongs to: a launcher watching the target of its own
// missile. Another aircraft's radar on the same scene is not. A scene with no
// missile has no launcher to prefer, so any radar's first look counts.
bool IsLaunchersLookAtItsTarget(World& world, EntityID owner, EntityID target) {
    const ComponentStore<GuidanceComponent>& missiles = world.Components<GuidanceComponent>();
    if (missiles.Size() == 0) return true;
    for (const auto& [missile, guidance] : missiles) {
        if (guidance.launcher == owner && guidance.target == target) return true;
    }
    return false;
}

}  // namespace

namespace AirborneRadarSystem {

void Update(World& world, SimContext& simContext, double simTime) {
    ComponentStore<AirborneRadarComponent>& owners = world.Components<AirborneRadarComponent>();
    ComponentStore<RadarCrossSection>& targets = world.Components<RadarCrossSection>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    ComponentStore<JSBSimComponent>& jsbsimComponents = world.Components<JSBSimComponent>();
    ComponentStore<KinematicsComponent>& kinematics = world.Components<KinematicsComponent>();
    ComponentStore<Velocity>& velocities = world.Components<Velocity>();

    // Per-target quantities independent of which radar is looking -- world
    // position, attitude (RCS shape), ground velocity (Doppler notch), and
    // alive state -- computed once per target rather than once per (owner,
    // target) pair. Built lazily on the first owner that actually sweeps (a
    // tick with no sweep pays nothing) and in targets-store order, which is
    // the order the per-pair RNG draws must happen in.
    struct TargetInfo {
        EntityID entity;
        Vec3 pos;
        Vec3 eulerDeg;
        Vec3 velocity;
        bool alive;
        RadarCrossSection rcs;
    };
    // thread_local only to reuse the buffer's capacity across ticks; the
    // contents are rebuilt every sweeping tick, so nothing carries over
    // between ticks, runs or batch worker threads.
    static thread_local std::vector<TargetInfo> targetInfos;
    targetInfos.clear();
    bool targetsCached = false;
    auto ensureTargets = [&]() {
        if (targetsCached) return;
        targetsCached = true;
        targetInfos.reserve(targets.Size());
        for (auto& [target, rcs] : targets) {
            const Transform* targetTransform = transforms.TryGet(target);
            if (targetTransform == nullptr) continue;
            targetInfos.push_back(TargetInfo{target, targetTransform->pos, targetTransform->euler,
                                             EntityVelocity::GetWithVelocityFallback(jsbsimComponents, kinematics,
                                                                                     velocities, target),
                                             !destroyed.Has(target), rcs});
        }
    };

    // One look per target per frame, every target on the same tick: a
    // frame is the time a +/-60 deg, 2-bar raster takes, but which bar the
    // beam is on is not modelled (REFERENCE.md - Frames and track
    // confirmation). In single-target track the locked target gets its own,
    // faster frame, and the scan frames still tick for every other target
    // but find nothing: the antenna is not looking there (REFERENCE.md -
    // Single-target track).
    for (auto& [owner, radar] : owners) {
        bool inStt = radar.sttTarget != kNoEntity;
        bool scanFrame = Numeric::PeriodElapsed(simTime, radar.lastFrameTime, radar.frameTimeS);
        bool sttFrame = inStt && Numeric::PeriodElapsed(simTime, radar.lastSttFrameTime, radar.sttFrameTimeS);
        if (!scanFrame && !sttFrame) continue;
        if (scanFrame) radar.lastFrameTime = simTime;
        if (sttFrame) radar.lastSttFrameTime = simTime;
        const Transform* ownerTransformPtr = transforms.TryGet(owner);
        if (ownerTransformPtr == nullptr) continue;
        const Transform& ownerTransform = *ownerTransformPtr;
        const bool ownerAlive = !destroyed.Has(owner);
        Vec3 forward = RadarEquation::ForwardVector(ownerTransform.euler);
        const ReceiverBudget receiver = ReceiverOf(radar);
        ensureTargets();

        bool lockedTargetListed = false;
        for (const TargetInfo& info : targetInfos) {
            if (info.entity == owner) continue;
            if (inStt && info.entity == radar.sttTarget) lockedTargetListed = true;
            bool locked = inStt && info.entity == radar.sttTarget;
            if (locked ? !sttFrame : !scanFrame) continue;
            const Vec3& targetPos = info.pos;

            // FOV gate, evaluated before the radar equation: outside the
            // cone is an automatic miss with no equation call and so no
            // RNG draw. A single symmetric cone, not independent
            // azimuth/elevation limits.
            Vec3 toTarget = targetPos - ownerTransform.pos;
            double toTargetLen = toTarget.norm();
            // A destroyed aircraft stops returning echoes, and a destroyed
            // owner stops transmitting: both fold into the same no-return
            // path an out-of-cone target takes, so a track drops (and a
            // lock breaks) by the same rule rather than freezing.
            // Under a lock on another target the antenna never points here:
            // no look, and so no RNG draw.
            double snrDb = kNoReturnDb;
            bool blind = inStt && !locked;
            if (!blind && toTargetLen > 0.0 && info.alive && ownerAlive) {
                Vec3 toTargetUnit = toTarget.normalized();
                double cosAngle = std::clamp(forward.dot(toTargetUnit), -1.0, 1.0);
                double angleDeg = std::acos(cosAngle) * kRadToDeg;
                if (angleDeg <= radar.fovHalfAngleDeg) {
                    double rangeM = RadarEquation::RangeM(ownerTransform.pos, targetPos);

                    // The RCS the target's attitude shows the radar.
                    // toTargetUnit points owner->target, so the view from
                    // the target is its negation.
                    RadarCrossSection adjustedRcs = info.rcs;
                    adjustedRcs.staticRcsM2 =
                        RadarEquation::MeanRcsM2(info.rcs, info.eulerDeg, -toTargetUnit);
                    snrDb = RadarEquation::DetectorOutputDb(simContext, receiver, adjustedRcs, rangeM);

                    // The notch is gated on the target's GROUND-relative
                    // radial velocity, NOT owner-relative closure: the
                    // owner's own motion shifts the clutter return exactly as
                    // much as the target's, so it cancels (relative closure
                    // would invert which maneuver defeats the radar). Only
                    // while the beam touches the ground -- with sky behind
                    // the whole beam there is no clutter to hide in. The
                    // notch, blind range and horizon are independent gates.
                    double elevationBeamwidthRad = RadarTrackFilter::BeamwidthRad(
                        receiver.wavelengthM, radar.antennaHeightM, radar.beamwidthFactor);
                    if (RadarEquation::BeamOnGround(ownerTransform.pos.y() / kFeetPerMeter, toTargetUnit,
                                                    elevationBeamwidthRad) &&
                        RadarEquation::InDopplerNotch(info.velocity, toTargetUnit, radar.notchVelocityThresholdMps))
                        snrDb = kNoReturnDb;
                    if (rangeM < radar.minRangeM) snrDb = kNoReturnDb;
                    double horizonRangeM = RadarEquation::HorizonRangeM(ownerTransform.pos.y() / kFeetPerMeter,
                                                                        targetPos.y() / kFeetPerMeter);
                    if (rangeM > horizonRangeM) snrDb = kNoReturnDb;
                }
            }

            AirborneRadarTrack& track = simContext.airborneRadarTracks.FindOrCreate(owner, info.entity);
            track.owner = owner;  // idempotent; sets the key fields on first creation
            track.target = info.entity;
            bool detected = snrDb >= radar.searchThresholdDb;
            HitWindow::Push(track.frameHits, detected, radar.confirmWindowFrames);
            int frameCount = HitWindow::Count(track.frameHits);
            track.missedFrames = detected ? 0 : track.missedFrames + 1;

            // The tracker: every frame moves the estimate to now; a
            // detection then measures the target -- range and two angles,
            // each with an error ~ resolution / sqrt(SNR) at this look's own
            // SNR -- and folds the measurement in. A missed frame coasts.
            // See REFERENCE.md - Track measurement and filtering.
            RadarTrackFilter::Predict(track.estimate, simTime, radar.trackManeuverG * kGravityFtPerSec2);
            if (detected) {
                Vec3 los = targetPos - ownerTransform.pos;
                double rangeFt = los.norm();
                Vec3 losUnit = los * (1.0 / rangeFt);
                // The detector output is signal plus noise, in units of
                // the noise: less one, the signal-to-noise ratio of this look.
                // Floored where the angle error reaches a full beamwidth --
                // a detection is at least somewhere in the beam (only a
                // threshold set below the noise can get here).
                double snrLinear = std::max(std::pow(10.0, snrDb / 10.0) - 1.0,
                                            1.0 / (2.0 * radar.monopulseSlope * radar.monopulseSlope));
                RadarTrackFilter::LookSigmas sigmas = RadarTrackFilter::MeasurementSigmas(
                    snrLinear, RadarTrackFilter::BeamwidthRad(receiver.wavelengthM, radar.antennaWidthM,
                                                              radar.beamwidthFactor),
                    RadarTrackFilter::BeamwidthRad(receiver.wavelengthM, radar.antennaHeightM, radar.beamwidthFactor),
                    radar.monopulseSlope, radar.pulseWidthS);
                Vec3 azimuthUnit, elevationUnit;
                RadarTrackFilter::MeasurementAxes(losUnit, azimuthUnit, elevationUnit);
                Vec3 measured = targetPos;
                measured += losUnit * simContext.rng.NextGaussian(0.0, sigmas.rangeM * kFeetPerMeter);
                measured += azimuthUnit * simContext.rng.NextGaussian(0.0, sigmas.azimuthRad * rangeFt);
                measured += elevationUnit * simContext.rng.NextGaussian(0.0, sigmas.elevationRad * rangeFt);
                RadarTrackFilter::Update(track.estimate, simTime, measured, losUnit, rangeFt, sigmas,
                                         kInitialVelSigmaFtPerSec);
                track.lastSeenTime = simTime;
            }

            RadarTrackState oldState = track.state;
            // Built only when an event fires: this runs for every pair every frame.
            auto eventData = [&] { return "owner=" + std::to_string(owner) + " target=" + std::to_string(info.entity); };

            // A first detection opens a tentative track; confirmFrames of the
            // last confirmWindowFrames confirm it ("three or more scans",
            // REFERENCE.md - Frames and track confirmation).
            if (track.state == RadarTrackState::NONE && detected) track.state = RadarTrackState::TENTATIVE;
            if (track.state == RadarTrackState::TENTATIVE && frameCount >= radar.confirmFrames)
                track.state = RadarTrackState::CONFIRMED;

            // Drops: a confirmed track after dropFrames frames in a row with
            // no detection (it coasts on the filter until then), or
            // sttDropFrames STT frames while locked; a tentative one once
            // its window holds no detection at all. The window clears with
            // the track, so a reacquired target has to confirm afresh.
            int dropAfter = locked ? radar.sttDropFrames : radar.dropFrames;
            bool dropped = (track.state == RadarTrackState::CONFIRMED && track.missedFrames >= dropAfter) ||
                           (track.state == RadarTrackState::TENTATIVE && frameCount == 0);
            if (dropped) {
                track.state = RadarTrackState::NONE;
                track.frameHits = 0;
                track.estimate.valid = false;
            }

            if (oldState == RadarTrackState::NONE && track.state != RadarTrackState::NONE) {
                simContext.eventLog.Record(SimEvent{simTime, "DETECTION", info.entity, eventData()});
                if (simContext.outcome.firstDetectionSimTime < 0.0 &&
                    IsLaunchersLookAtItsTarget(world, owner, info.entity))
                    simContext.outcome.firstDetectionSimTime = simTime;
            }
            if (oldState != RadarTrackState::CONFIRMED && track.state == RadarTrackState::CONFIRMED) {
                simContext.eventLog.Record(SimEvent{simTime, "RADAR_TRACK_ESTABLISHED", info.entity, eventData()});
            }
            if (oldState == RadarTrackState::CONFIRMED && dropped) {
                simContext.eventLog.Record(SimEvent{simTime, "RADAR_TRACK_DROPPED", info.entity, eventData()});
            }
            // A lock is a confirmed track in STT, so the drop above breaks
            // it -- counted in STT frames, about a second. Back to
            // scanning; it has to confirm again before fire control can
            // lock it again.
            if (locked && dropped) {
                radar.sttTarget = kNoEntity;
                simContext.eventLog.Record(SimEvent{simTime, "RADAR_LOCK_LOST", info.entity, eventData()});
            }
        }

        // A lock on something that is not a radar target at all (no cross
        // section, no transform) is never looked at, so no missed frames
        // would ever break it, and until it did the radar would be blind to
        // everything else.
        if (inStt && !lockedTargetListed) {
            simContext.eventLog.Record(SimEvent{simTime, "RADAR_LOCK_LOST", radar.sttTarget,
                                                "owner=" + std::to_string(owner) +
                                                    " target=" + std::to_string(radar.sttTarget)});
            radar.sttTarget = kNoEntity;
        }
    }
}

}  // namespace AirborneRadarSystem
