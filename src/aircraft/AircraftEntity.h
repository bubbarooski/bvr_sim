#pragma once

#include "aircraft/AircraftType.h"
#include "components/AircraftComponent.h"
#include "components/AirborneRadarComponent.h"
#include "components/GLimitComponent.h"
#include "components/RadarCrossSection.h"
#include "components/RwrComponent.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "sim/AircraftBounds.h"

// The joins between an entity and its aircraft type, so systems ask for
// what they need ("this target's airframe") rather than each reaching for
// the type and deciding what an entity without one means.
namespace AircraftEntity {

// The entity's aircraft type, or nullptr if it is not an aircraft.
inline const AircraftType* TypeOf(World& world, EntityID entity) {
    const AircraftComponent* aircraft = world.Components<AircraftComponent>().TryGet(entity);
    return aircraft != nullptr ? aircraft->type.get() : nullptr;
}

// The airframe a missile's miss distance is measured against: the type's
// hit box, or a point for an entity with no aircraft type.
inline AircraftBounds::HitBox HitBoxOf(World& world, EntityID entity) {
    const AircraftType* type = TypeOf(world, entity);
    return type != nullptr ? type->hitBox : AircraftBounds::HitBox{};
}

// A new aircraft's envelope: the type's limits, no damage yet. A scenario
// may then override individual fields of it.
inline GLimitComponent MakeGLimit(const AircraftType& type) {
    GLimitComponent gLimit;
    gLimit.clMax = type.envelope.clMax;
    gLimit.clMaxLossPerG = type.envelope.clMaxLossPerG;
    gLimit.wingAreaFt2 = type.wingAreaFt2;
    gLimit.structuralGLimitPos = type.envelope.structuralGLimitPos;
    gLimit.structuralGLimitNeg = type.envelope.structuralGLimitNeg;
    gLimit.damageAccumulationRate = type.envelope.damageAccumulationRate;
    return gLimit;
}

// The type's radar as a new aircraft's component. A scenario may then
// override individual fields of it.
inline AirborneRadarComponent MakeAirborneRadar(const AircraftType& type) {
    const AirborneRadarType& r = type.radar;
    AirborneRadarComponent radar;
    radar.frameTimeS = r.frameTimeS;
    radar.sttFrameTimeS = r.sttFrameTimeS;
    radar.averagePowerW = r.averagePowerW;
    radar.peakPowerW = r.peakPowerW;
    radar.antennaGain = r.antennaGain;
    radar.frequencyHz = r.frequencyHz;
    radar.noiseFigureDb = r.noiseFigureDb;
    radar.timeOnTargetS = r.timeOnTargetS;
    radar.systemLossDb = r.systemLossDb;
    radar.referenceRangeNmi = r.referenceRangeNmi;
    radar.typeReferenceRangeNmi = r.referenceRangeNmi;
    radar.fovHalfAngleDeg = r.fovHalfAngleDeg;
    radar.searchThresholdDb = r.searchThresholdDb;
    radar.confirmFrames = r.confirmFrames;
    radar.confirmWindowFrames = r.confirmWindowFrames;
    radar.dropFrames = r.dropFrames;
    radar.sttDropFrames = r.sttDropFrames;
    radar.fireControlDelayS = r.fireControlDelayS;
    radar.notchVelocityThresholdMps = r.notchVelocityThresholdMps;
    radar.antennaHeightM = r.antennaHeightM;
    radar.antennaWidthM = r.antennaWidthM;
    radar.beamwidthFactor = r.beamwidthFactor;
    radar.monopulseSlope = r.monopulseSlope;
    radar.pulseWidthS = r.pulseWidthS;
    radar.trackManeuverG = r.trackManeuverG;
    radar.minRangeM = r.minRangeM;
    return radar;
}

// The type's radar warning receiver as a new aircraft's component.
inline RwrComponent MakeRwr(const AircraftType& type) {
    RwrComponent rwr;
    rwr.sensitivityDbm = type.rwr.sensitivityDbm;
    rwr.antennaGain = type.rwr.antennaGain;
    rwr.signalJitterDb = type.rwr.signalJitterDb;
    rwr.sweepPeriodS = type.rwr.sweepPeriodS;
    return rwr;
}

// The type's radar signature as a new aircraft's component. A scenario's
// radar_cross_section block may then override the nose-on RCS (rescaling
// the shape) or the fluctuation.
inline RadarCrossSection MakeRadarCrossSection(const AircraftType& type) {
    RadarCrossSection rcs;
    rcs.staticRcsM2 = type.signature.staticRcsM2;
    rcs.fluctuation = type.signature.fluctuation;
    rcs.shape = type.signature.shape;
    rcs.shapeCount = type.signature.shapeCount;
    return rcs;
}

}  // namespace AircraftEntity
