#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "AircraftRig.h"
#include "LevelTurnController.h"
#include "aircraft/AircraftType.h"
#include "sim/Atmosphere.h"
#include "systems/FlightDynamicsSystem.h"

namespace testrig {

// Drag and lift coefficients in a hard level turn, and the full-power
// thrust the aircraft has there: everything a sustained turn's energy
// balance needs. Shared by tools/drivers/turn_sample (which fits drag due to lift to
// a flight manual) and real_world_validation (which checks the result), so
// the two measure the same way.
struct TurnDragSample {
    double mach = 0.0;       // during the measurement (the pull bleeds a little)
    double thrustLbf = 0.0;  // full power at the point's Mach, 1 g
    double cd = 0.0;
    double cl = 0.0;
    double alphaDeg = 0.0;
    double nz = 0.0;
    bool stickAtStop = false;
};

// Trims the type at (altitude, Mach) and the gross weight, fuel frozen;
// reads full-power thrust at 1 g; then pulls (1 s ramp, 1 s to settle) to
// `loadFactor` and averages the coefficients over half a second. The speed
// moves a little during the pull; coefficients barely change with it.
// nullopt if the point will not trim.
inline std::optional<TurnDragSample> SampleTurnDrag(const AircraftType& type,
                                                    const FlightDynamicsSystem::FuelCapacity& fuel,
                                                    double altitudeFt, double mach, double grossLbf,
                                                    double loadFactor) {
    constexpr double kKtsToFtPerSec = 1.68781;
    double kts = mach * Atmosphere::Lookup(altitudeFt).speedOfSoundFtPerSec / kKtsToFtPerSec;
    std::optional<AircraftRig> trimmed;
    try {
        trimmed.emplace(type.id.c_str(), altitudeFt, kts, 0.0);
    } catch (const std::exception&) {
        return std::nullopt;
    }
    AircraftRig& rig = *trimmed;
    FlightDynamicsSystem::SetGrossWeight(rig.Fdm(), type, fuel, grossLbf);
    rig.Fdm().SetPropertyValue("propulsion/fuel_freeze", 1.0);
    rig.Fly(3.0);  // settle on the new weight

    LevelTurnController turn(altitudeFt, 1.0, type.controls.gPerUnitStick);
    const double dt = AircraftRig::kDt;
    TurnDragSample sample;
    const int quarter = static_cast<int>(0.25 / dt);
    for (int i = 0; i < quarter; ++i) {
        turn.Step(rig, 1.0, type.controls.maxThrottleCmd);
        sample.thrustLbf += rig.Prop("propulsion/engine[0]/thrust-lbs");
    }
    sample.thrustLbf /= quarter;

    for (int i = 0; i < static_cast<int>(2.0 / dt); ++i) {
        double n = 1.0 + (loadFactor - 1.0) * std::min(1.0, i * dt / 1.0);
        turn.Step(rig, n, type.controls.maxThrottleCmd);
    }
    double wingArea = rig.Prop("metrics/Sw-sqft");
    const int ticks = static_cast<int>(0.5 / dt);
    for (int i = 0; i < ticks; ++i) {
        turn.Step(rig, loadFactor, type.controls.maxThrottleCmd);
        double qs = rig.Prop("aero/qbar-psf") * wingArea;
        sample.cd += rig.Prop("forces/fwx-aero-lbs") / qs;
        sample.cl += rig.Prop("forces/fwz-aero-lbs") / qs;
        sample.alphaDeg += rig.Prop("aero/alpha-deg");
        sample.nz += rig.Prop("accelerations/Nz");
        sample.mach += rig.Prop("velocities/mach");
    }
    sample.cd /= ticks;
    sample.cl /= ticks;
    sample.alphaDeg /= ticks;
    sample.nz /= ticks;
    sample.mach /= ticks;
    sample.stickAtStop = turn.StickAtStop();
    return sample;
}

}  // namespace testrig
