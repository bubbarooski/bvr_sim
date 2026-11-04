#include "LaunchSystem.h"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "aircraft/AircraftEntity.h"
#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/MissileComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/DatalinkTrackComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "missile/LaunchZone.h"
#include "missile/MissileEntity.h"
#include "sim/AircraftBounds.h"
#include "sim/RadarTrackFilter.h"
#include "util/EntityLookup.h"
#include "util/EntityVelocity.h"
#include "util/Numeric.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace LaunchSystem {

namespace {

// Absorbs rounding in the STT-to-scan frame ratio, so an exact multiple
// does not round up a frame.
constexpr double kFrameCountSlack = 1e-9;

// The scenario's doctrine values (the type's with the scenario's overrides
// applied, ScenarioData::SetMissileType, then the doctrine): what a
// "(next launch)" edit changes.
void CopyDoctrine(GuidanceComponent& guidance, const ScenarioData& scenario) {
    guidance.boostDurationS = scenario.missileBoostDurationS;
    guidance.boostThrustLbf = scenario.missileBoostThrustLbf;
    guidance.sustainDurationS = scenario.missileSustainDurationS;
    guidance.sustainThrustLbf = scenario.missileSustainThrustLbf;
    guidance.dryMassSlug = scenario.missileDryMassSlug;
    guidance.propellantMassSlug = scenario.missilePropellantMassSlug;
    guidance.referenceAreaFt2 = scenario.missileReferenceAreaFt2;
    guidance.seekerFovHalfAngleDeg = scenario.missileSeekerFovHalfAngleDeg;
    guidance.memoryCoastTimeS = scenario.missileMemoryCoastTimeS;
    guidance.armingRangeFt = scenario.missileArmingRangeFt;
    guidance.selfDestructMinAvailableG = scenario.missileSelfDestructMinAvailableG;
    guidance.selfDestructMaxTimeOfFlightS = scenario.missileSelfDestructMaxTimeOfFlightS;
    guidance.seekerMode = scenario.missileSeekerMode;
    guidance.launchMode = scenario.missileLaunchMode;
    // The missile's own active seeker, consulted only by MIDCOURSE_ARH.
    guidance.seekerAveragePowerW = scenario.missileSeekerAveragePowerW;
    guidance.seekerAntennaGain = scenario.missileSeekerAntennaGain;
    guidance.seekerFrequencyHz = scenario.missileSeekerFrequencyHz;
    guidance.seekerNoiseFigureDb = scenario.missileSeekerNoiseFigureDb;
    guidance.seekerIntegrationTimeS = scenario.missileSeekerIntegrationTimeS;
    guidance.seekerSystemLossDb = scenario.missileSeekerSystemLossDb;
    guidance.seekerReferenceRangeNmi = scenario.missileSeekerReferenceRangeNmi;
    guidance.seekerThresholdDb = scenario.missileSeekerThresholdDb;
    guidance.seekerSweepPeriodS = scenario.missileSeekerSweepPeriodS;
    guidance.launchTurnExitDeg = scenario.missileLaunchTurnExitDeg;
}

}  // namespace

void Init(World& world, EntityID pursuer, EntityID target, const ScenarioData& scenario) {
    if (scenario.missileType == nullptr) {
        throw std::logic_error("LaunchSystem::Init: the scenario has no missile type");
    }
    const MissileType& type = *scenario.missileType;

    EntityID missile = world.CreateEntity();
    world.Components<Tag>().Add(missile, Tag{"Missile"});
    world.Components<MissileComponent>().Add(missile, MissileComponent{scenario.missileType});

    // No Transform or Kinematics yet: rendering and trails both iterate
    // the Transform store, so an unlaunched missile draws nothing while
    // still being inspectable.
    GuidanceComponent guidance;
    guidance.target = target;
    guidance.launcher = pursuer;  // whose radar gates the launch, and whose illumination the missile depends on
    CopyDoctrine(guidance, scenario);
    guidance.seekerTypeReferenceRangeNmi = type.seeker.referenceRangeNmi;
    // Seeker signal processing (M-of-N, the notch) and the response time:
    // the type's, not a doctrine choice.
    guidance.seekerConfirmHits = type.seeker.confirmHits;
    guidance.seekerConfirmWindow = type.seeker.confirmWindow;
    guidance.seekerAntennaDiameterM = type.seeker.antennaDiameterM;
    guidance.seekerBeamwidthFactor = type.seeker.beamwidthFactor;
    guidance.seekerNotchVelocityThresholdMps = type.seeker.notchVelocityThresholdMps;
    guidance.responseTimeS = type.autopilot.responseTimeS;
    world.Components<GuidanceComponent>().Add(missile, guidance);

    // Real missiles only. Synthetic test missiles without one fall back to
    // the flat structural clamp in GuidanceSystem.
    world.Components<MissileGLimitComponent>().Add(missile, MissileEntity::MakeGLimit(type));

    // Where the missile thinks the target is during midcourse. Populated
    // by DatalinkSystem once the missile is actually in the air. The
    // missile knows its own position: its midcourse error is the relayed
    // radar track's (REFERENCE.md - Datalink target state).
    world.Components<DatalinkTrackComponent>().Add(missile, scenario.missileDatalink);
}

void RefreshUnlaunched(World& world, const ScenarioData& scenario) {
    ComponentStore<DatalinkTrackComponent>& datalinks = world.Components<DatalinkTrackComponent>();
    for (auto& [missile, guidance] : world.Components<GuidanceComponent>()) {
        if (guidance.launched) continue;
        CopyDoctrine(guidance, scenario);
        if (DatalinkTrackComponent* datalink = datalinks.TryGet(missile))
            datalink->updateIntervalS = scenario.missileDatalink.updateIntervalS;
    }
}

bool InitFromTags(World& world, const ScenarioData& scenario) {
    std::optional<EntityID> pursuer = EntityLookup::FindByTagName(world, "Pursuer");
    std::optional<EntityID> target = EntityLookup::FindByTagName(world, "Target");
    if (!pursuer.has_value() || !target.has_value() || scenario.missileType == nullptr) return false;
    Init(world, *pursuer, *target, scenario);
    return true;
}

namespace {

// The launch gate: the launcher's radar holds a confirmed track on the
// target, and the track's estimated range is inside the missile's maximum
// launch range (its type's launch zone, evaluated on the estimate, not
// truth) times the doctrine's launch range fraction. A missile type with
// no launch zone is gated on the confirmed track alone.
bool LaunchGateOpen(World& world, const SimContext& simContext, const ScenarioData& scenario,
                    const GuidanceComponent& guidance, const Transform& launcherTransform, double& estimatedRangeFt,
                    double& rmaxFt) {
    const AirborneRadarTrack* track = simContext.airborneRadarTracks.Find(guidance.launcher, guidance.target);
    if (track == nullptr || track->state != RadarTrackState::CONFIRMED || !track->estimate.valid) return false;
    Vec3 estimatedPos = RadarTrackFilter::Position(track->estimate);
    Vec3 estimatedVel = RadarTrackFilter::Velocity(track->estimate);
    estimatedRangeFt = (estimatedPos - launcherTransform.pos).norm();
    const LaunchZone* zone = scenario.missileType != nullptr ? scenario.missileType->launchZone.get() : nullptr;
    if (zone == nullptr) {
        rmaxFt = -1.0;
        return true;
    }
    LaunchGeometry geometry = LaunchZone::Geometry(launcherTransform.pos, EntityVelocity::Get(world, guidance.launcher),
                                                   estimatedPos, estimatedVel,
                                                   simContext.environment.temperatureOffsetC);
    rmaxFt = zone->RmaxFt(geometry);
    return estimatedRangeFt <= rmaxFt * scenario.launchRangeFraction;
}

std::string LockEventData(const GuidanceComponent& guidance) {
    return "owner=" + std::to_string(guidance.launcher) + " target=" + std::to_string(guidance.target);
}

// Fire control's side of a single-target-track lock: lock the missile's
// target while the shot needs it and the launcher holds a confirmed track,
// release it once it no longer does. The radar itself breaks a lock whose
// track drops (AirborneRadarSystem); fire control then locks again only
// after the track confirms afresh. A radar already locked on some other
// target is left alone, and so is a lock another launched missile of the
// same launcher on the same target still needs.
void HoldLock(SimContext& simContext, const ComponentStore<GuidanceComponent>& guidances, EntityID missile,
              const GuidanceComponent& guidance, AirborneRadarComponent& radar, double simTime) {
    if (!NeedsLauncherLock(guidance)) {
        if (radar.sttTarget != guidance.target) return;
        for (const auto& [other, otherGuidance] : guidances) {
            if (other != missile && otherGuidance.launched && otherGuidance.launcher == guidance.launcher &&
                otherGuidance.target == guidance.target && NeedsLauncherLock(otherGuidance))
                return;
        }
        radar.sttTarget = kNoEntity;
        // The track's missed frames were counted in STT frames; back on
        // the scan they count toward the scan-frame drop rule, so convert
        // them by time.
        if (simContext.airborneRadarTracks.Find(guidance.launcher, guidance.target) != nullptr) {
            AirborneRadarTrack& track = simContext.airborneRadarTracks.FindOrCreate(guidance.launcher, guidance.target);
            track.missedFrames =
                static_cast<int>(std::ceil(track.missedFrames * radar.sttFrameTimeS / radar.frameTimeS - kFrameCountSlack));
        }
        simContext.eventLog.Record(SimEvent{simTime, "RADAR_LOCK_RELEASED", guidance.target, LockEventData(guidance)});
        return;
    }
    if (radar.sttTarget != kNoEntity) return;
    const AirborneRadarTrack* track = simContext.airborneRadarTracks.Find(guidance.launcher, guidance.target);
    if (track == nullptr || track->state != RadarTrackState::CONFIRMED) return;
    // The track already met confirmation, so the lock holds at once; its
    // first STT look comes on the radar's next update.
    radar.sttTarget = guidance.target;
    radar.lastSttFrameTime = Numeric::kLongAgoS;
    simContext.eventLog.Record(SimEvent{simTime, "RADAR_LOCK", guidance.target, LockEventData(guidance)});
}

}  // namespace

void Update(World& world, SimContext& simContext, const ScenarioData& scenario, double simTime) {
    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<AirborneRadarComponent>& radars = world.Components<AirborneRadarComponent>();

    for (auto& [missile, guidance] : guidances) {
        AirborneRadarComponent* launcherRadar = radars.TryGet(guidance.launcher);
        if (guidance.launched) {  // one shot per missile; after it, only the lock
            if (launcherRadar != nullptr) HoldLock(simContext, guidances, missile, guidance, *launcherRadar, simTime);
            continue;
        }
        const Transform* pursuerTransformPtr = transforms.TryGet(guidance.launcher);
        const Transform* targetTransformPtr = transforms.TryGet(guidance.target);
        if (pursuerTransformPtr == nullptr || targetTransformPtr == nullptr) continue;
        // A destroyed launcher fires nothing. Its radar's confirmed track coasts for
        // dropFrames frames, longer than the fire-control delay, so the gate alone
        // would still open for a wreck.
        if (world.Components<DestroyedComponent>().Has(guidance.launcher)) {
            guidance.launchGateOpenTime = -1.0;
            continue;
        }
        const Transform& pursuerTransform = *pursuerTransformPtr;
        const Transform& targetTransform = *targetTransformPtr;

        // A launcher with no airborne radar never fires: it never holds a
        // track. Once the gate opens the fire-control system takes the
        // launcher radar's delay to fire, and a gate that shuts meanwhile
        // starts it again.
        // An STT shot locks once the TWS gate opens and needs the lock held
        // too: the fire-control delay runs again from the start after a
        // broken lock.
        double estimatedRangeFt = 0.0, rmaxFt = 0.0;
        bool gateOpen =
            LaunchGateOpen(world, simContext, scenario, guidance, pursuerTransform, estimatedRangeFt, rmaxFt);
        if (gateOpen && UsesStt(guidance)) {
            if (launcherRadar != nullptr) HoldLock(simContext, guidances, missile, guidance, *launcherRadar, simTime);
            gateOpen = launcherRadar != nullptr && launcherRadar->sttTarget == guidance.target;
        }
        if (!gateOpen) {
            guidance.launchGateOpenTime = -1.0;
            continue;
        }
        if (guidance.launchGateOpenTime < 0.0) {
            guidance.launchGateOpenTime = simTime;
            simContext.eventLog.Record(SimEvent{simTime, "LAUNCH_ZONE_ENTERED", missile,
                                                  "target=" + std::to_string(guidance.target) +
                                                      " est_range_ft=" + std::to_string(estimatedRangeFt) +
                                                      " rmax_ft=" + std::to_string(rmaxFt)});
            if (simContext.outcome.firingSolutionSimTime < 0.0) simContext.outcome.firingSolutionSimTime = simTime;
        }
        double fireControlDelayS = launcherRadar != nullptr ? launcherRadar->fireControlDelayS : 0.0;
        if (!Numeric::PeriodElapsed(simTime, guidance.launchGateOpenTime, fireControlDelayS)) continue;

        double range = (pursuerTransform.pos - targetTransform.pos).norm();

        Transform missileTransform;
        missileTransform.pos = pursuerTransform.pos;
        missileTransform.euler = pursuerTransform.euler;
        transforms.Add(missile, missileTransform);

        Vec3 launchVel = EntityVelocity::Get(world, guidance.launcher);
        KinematicsComponent kinematics;
        kinematics.vel = launchVel;
        kinematics.speed = launchVel.norm();
        world.Components<KinematicsComponent>().Add(missile, kinematics);

        guidance.navGain = scenario.missileNavGain;
        guidance.prevLosUnit = (targetTransform.pos - missileTransform.pos).normalized();
        guidance.active = true;
        guidance.launched = true;
        // Seeded with the same surface-distance metric InterceptSystem uses
        // every tick, not the center-to-center `range` logged below, so its
        // first comparison is against a like-for-like value. The impact
        // point and the distance come from one body-frame transform -- the
        // two are read as one snapshot.
        guidance.prevImpactBodyFt =
            AircraftBounds::ToBodyFrameFt(missileTransform.pos, targetTransform.pos, targetTransform.euler);
        guidance.prevRangeFt = AircraftBounds::DistanceToSurfaceFromBodyFt(
            AircraftEntity::HitBoxOf(world, guidance.target), guidance.prevImpactBodyFt);
        // Seeded to the same values so the sub-tick CPA fit's three-sample
        // window is well-formed from the first tick. An all-equal triple is
        // harmless: it is not convex, so the fit falls back to the sample.
        guidance.prevPrevRangeFt = guidance.prevRangeFt;
        guidance.prevPrevImpactBodyFt = guidance.prevImpactBodyFt;
        guidance.launchSimTime = simTime;
        guidance.launchPos = missileTransform.pos;  // arming-range reference

        // The loft angle from what fire control knows, the estimated range.
        // Only a missile that starts on the datalink lofts: a seeker
        // measuring from the rail would climb its target out of the cone.
        if (scenario.missileLoft && scenario.missileType != nullptr && NavigatesInertially(guidance)) {
            const MissileLoft& loft = scenario.missileType->loft;
            guidance.loftClimbDeg = loft.ClimbDeg(estimatedRangeFt / Units::kFeetPerNmi);
            guidance.loftClimbTimeConstantS = loft.climbTimeConstantS;
            guidance.loftMinAvailableG = loft.minAvailableG;
            guidance.lofting = guidance.loftClimbDeg > 0.0;
        }

        simContext.eventLog.Record(SimEvent{simTime, "LAUNCH", missile,
                                              "target=" + std::to_string(guidance.target) +
                                                  " gate=" + (UsesStt(guidance) ? "stt" : "tws") +
                                                  " range_ft=" + std::to_string(range)});

        simContext.outcome.launched = true;
        simContext.outcome.launchSimTime = simTime;
        simContext.outcome.launchRangeFt = range;

        // An ARH shot from STT releases the lock at once.
        if (launcherRadar != nullptr) HoldLock(simContext, guidances, missile, guidance, *launcherRadar, simTime);
    }
}

}  // namespace LaunchSystem
