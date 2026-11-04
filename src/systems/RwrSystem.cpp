#include "RwrSystem.h"

#include <cmath>
#include <limits>
#include <string>

#include "components/AirborneRadarComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/RwrComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "sim/RadarEquation.h"
#include "util/Units.h"
#include "util/Numeric.h"

namespace RwrSystem {

using Units::kFeetPerMeter;

void Update(World& world, SimContext& simContext, double simTime) {
    ComponentStore<RwrComponent>& rwrs = world.Components<RwrComponent>();
    ComponentStore<AirborneRadarComponent>& radars = world.Components<AirborneRadarComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();

    for (auto& [owner, rwr] : rwrs) {
        // Sweep gate, placed before the transform check so a sweep is
        // consumed on the same schedule regardless of the rest of the tick.
        if (!Numeric::PeriodElapsed(simTime, rwr.lastSweepTime, rwr.sweepPeriodS)) continue;
        rwr.lastSweepTime = simTime;
        const Transform* ownerTransform = transforms.TryGet(owner);
        if (ownerTransform == nullptr) continue;
        const Vec3& ownerPos = ownerTransform->pos;

        // A destroyed aircraft's RWR goes dark: its state is cleared, not
        // frozen, and nothing is logged -- a warning "clearing" after the
        // kill would read as something the dead aircraft heard.
        if (destroyed.Has(owner)) {
            rwr.currentlyIlluminated = false;
            rwr.perceivedMode = RwrMode::NONE;
            rwr.missileWarning = false;
            continue;
        }

        // Evaluated against every illuminator in the scene; the strongest
        // received signal wins, as a real RWR prioritizes the loudest
        // emitter.
        double bestPrDbw = -std::numeric_limits<double>::infinity();
        EntityID bestIlluminator = kNoEntity;
        bool heardLock = false;
        for (auto& [illuminator, radar] : radars) {
            if (illuminator == owner) continue;
            const Transform* illuminatorTransform = transforms.TryGet(illuminator);
            if (illuminatorTransform == nullptr) continue;
            // A destroyed aircraft is not transmitting either.
            if (destroyed.Has(illuminator)) continue;
            double rangeM = RadarEquation::RangeM(ownerPos, illuminatorTransform->pos);

            // One-way Friis link: R^2 falloff, not the round-trip R^4 the
            // illuminator's own detection sees. See REFERENCE.md - The
            // R^2 / R^4 asymmetry.
            // Peak power: an RWR detects the pulses themselves, not the
            // radar's integrated return.
            double wavelengthM = WavelengthM(radar.frequencyHz);
            double prDb = RadarEquation::FriisReceivedPowerDb(radar.peakPowerW, radar.antennaGain, rwr.antennaGain,
                                                              wavelengthM, rangeM);
            prDb += simContext.rng.NextGaussian(0.0, rwr.signalJitterDb);
            // Below the horizon the Earth is in the way: the same 4/3-Earth
            // line of sight the radar uses. Gated after the jitter draw so
            // the stream does not depend on geometry. See REFERENCE.md -
            // The Friis one-way link.
            double horizonRangeM = RadarEquation::HorizonRangeM(ownerPos.y() / kFeetPerMeter,
                                                                illuminatorTransform->pos.y() / kFeetPerMeter);
            if (rangeM > horizonRangeM) prDb = -std::numeric_limits<double>::infinity();
            if (radar.sttTarget == owner && prDb >= rwr.sensitivityDbm - Units::kDbmPerDbw) heardLock = true;

            if (prDb > bestPrDbw) {
                bestPrDbw = prDb;
                bestIlluminator = illuminator;
            }
        }

        // A missile seeker transmitting at this aircraft, heard while it is
        // above the horizon. No link budget and no jitter draw: the seeker
        // radiates its main lobe here only inside its own ~10 nmi
        // acquisition range, where the one-way link closes with margin for
        // any pulsed duty cycle. Its sidelobes, at anyone else, are not
        // modelled. See REFERENCE.md - Missile warning.
        bool heardMissile = false;
        for (auto& [missile, guidance] : guidances) {
            if (guidance.target != owner || !SeekerRadiating(guidance) || destroyed.Has(missile)) continue;
            const Transform* missileTransform = transforms.TryGet(missile);
            if (missileTransform == nullptr) continue;
            if (RadarEquation::RangeM(ownerPos, missileTransform->pos) >
                RadarEquation::HorizonRangeM(ownerPos.y() / kFeetPerMeter, missileTransform->pos.y() / kFeetPerMeter))
                continue;
            heardMissile = true;
            break;
        }

        RwrMode oldMode = rwr.perceivedMode;
        rwr.currentlyIlluminated = bestIlluminator != kNoEntity && bestPrDbw >= rwr.sensitivityDbm - Units::kDbmPerDbw;

        // A radar in search or track-while-scan sounds the same to an RWR:
        // its beam keeps scanning, whatever it holds a track on. So
        // illumination reads as SEARCH; only a radar heard holding a
        // single-target-track lock on this aircraft reads as LOCK. See
        // REFERENCE.md - Perceived emitter mode.
        rwr.perceivedMode = heardLock                  ? RwrMode::LOCK
                            : rwr.currentlyIlluminated ? RwrMode::SEARCH
                                                       : RwrMode::NONE;

        if (oldMode == RwrMode::NONE && rwr.perceivedMode != RwrMode::NONE) {
            simContext.eventLog.Record(
                SimEvent{simTime, "RWR_ILLUMINATED", owner, "illuminator=" + std::to_string(bestIlluminator)});
        } else if (oldMode != RwrMode::NONE && rwr.perceivedMode == RwrMode::NONE) {
            simContext.eventLog.Record(SimEvent{simTime, "RWR_CLEAR", owner, ""});
        }
        if (oldMode != RwrMode::LOCK && rwr.perceivedMode == RwrMode::LOCK) {
            simContext.eventLog.Record(SimEvent{simTime, "RWR_LOCK", owner, ""});
        } else if (oldMode == RwrMode::LOCK && rwr.perceivedMode != RwrMode::LOCK) {
            simContext.eventLog.Record(SimEvent{simTime, "RWR_LOCK_CLEAR", owner, ""});
        }
        if (!rwr.missileWarning && heardMissile) {
            simContext.eventLog.Record(SimEvent{simTime, "RWR_MISSILE", owner, ""});
        } else if (rwr.missileWarning && !heardMissile) {
            simContext.eventLog.Record(SimEvent{simTime, "RWR_MISSILE_CLEAR", owner, ""});
        }
        rwr.missileWarning = heardMissile;
    }
}

}  // namespace RwrSystem
