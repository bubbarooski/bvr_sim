#pragma once

#include "components/GuidanceComponent.h"
#include "components/MissileComponent.h"
#include "components/MissileGLimitComponent.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileType.h"

// The joins between an entity and its missile type, mirroring
// AircraftEntity.h.
namespace MissileEntity {

// The entity's missile type, or nullptr if it has none.
inline const MissileType* TypeOf(World& world, EntityID entity) {
    const MissileComponent* missile = world.Components<MissileComponent>().TryGet(entity);
    return missile != nullptr ? missile->type.get() : nullptr;
}

// A new missile's turn ceiling: the type's.
inline MissileGLimitComponent MakeGLimit(const MissileType& type) {
    MissileGLimitComponent gLimit;
    gLimit.normalForce = type.aero.normalForce;
    gLimit.structuralGLimit = type.aero.structuralGLimit;
    return gLimit;
}

// A missile's guidance with the type's motor, mass, aerodynamics, seeker
// and fuze. Guidance state (target, launcher, lock) is left at its
// defaults for the caller.
inline GuidanceComponent MakeGuidance(const MissileType& type) {
    GuidanceComponent guidance;
    guidance.boostDurationS = type.propulsion.boostDurationS;
    guidance.boostThrustLbf = type.propulsion.boostThrustLbf;
    guidance.sustainDurationS = type.propulsion.sustainDurationS;
    guidance.sustainThrustLbf = type.propulsion.sustainThrustLbf;
    guidance.dryMassSlug = type.mass.dryMassSlug;
    guidance.propellantMassSlug = type.mass.propellantMassSlug;
    guidance.referenceAreaFt2 = type.aero.referenceAreaFt2;
    guidance.seekerMode = type.seeker.mode;
    guidance.seekerFovHalfAngleDeg = type.seeker.fovHalfAngleDeg;
    guidance.memoryCoastTimeS = type.seeker.memoryCoastTimeS;
    guidance.seekerAveragePowerW = type.seeker.averagePowerW;
    guidance.seekerAntennaGain = type.seeker.antennaGain;
    guidance.seekerFrequencyHz = type.seeker.frequencyHz;
    guidance.seekerNoiseFigureDb = type.seeker.noiseFigureDb;
    guidance.seekerIntegrationTimeS = type.seeker.integrationTimeS;
    guidance.seekerSystemLossDb = type.seeker.systemLossDb;
    guidance.seekerReferenceRangeNmi = type.seeker.referenceRangeNmi;
    guidance.seekerTypeReferenceRangeNmi = type.seeker.referenceRangeNmi;
    guidance.seekerThresholdDb = type.seeker.thresholdDb;
    guidance.seekerSweepPeriodS = type.seeker.sweepPeriodS;
    guidance.seekerConfirmHits = type.seeker.confirmHits;
    guidance.seekerConfirmWindow = type.seeker.confirmWindow;
    guidance.seekerAntennaDiameterM = type.seeker.antennaDiameterM;
    guidance.seekerBeamwidthFactor = type.seeker.beamwidthFactor;
    guidance.seekerNotchVelocityThresholdMps = type.seeker.notchVelocityThresholdMps;
    guidance.responseTimeS = type.autopilot.responseTimeS;
    guidance.launchTurnExitDeg = type.autopilot.launchTurnExitDeg;
    guidance.armingRangeFt = type.fuze.armingRangeFt;
    guidance.selfDestructMinAvailableG = type.selfDestruct.minAvailableG;
    guidance.selfDestructMaxTimeOfFlightS = type.selfDestruct.maxTimeOfFlightS;
    return guidance;
}

}  // namespace MissileEntity
