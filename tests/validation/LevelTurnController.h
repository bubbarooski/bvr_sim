#pragma once

#include <algorithm>

#include "AircraftRig.h"
#include "components/JSBSimComponent.h"
#include "systems/FlightDynamicsSystem.h"
#include "util/Units.h"

namespace testrig {

// Flies a level turn the way a steep turn is actually flown, for measuring
// what the aircraft can do rather than what the scenario autopilot asks:
// stick holds load factor (PI on Nz through the FCS's G command), bank
// holds altitude (PI on the vertical error), and the caller sets the
// throttle. ControlSystem is deliberately not in the loop -- it clamps its
// demand to the G envelope, which would measure the envelope instead.
//
// Commanding bank as acos(1/n) under-banks at turn AoA (Euler roll is not
// wind-axis bank), so bank is found by the altitude loop instead.
class LevelTurnController {
public:
    // `gPerUnitStick` > 0 adds the FCS's own G command as feed-forward
    // (the type's AircraftControlInterface value), so the load factor is
    // reached in about a second rather than over the PI's tens of seconds.
    LevelTurnController(double altitudeFt, double initialLoadFactor, double gPerUnitStick = 0.0)
        : altitudeFt_(altitudeFt),
          bankBaseDeg_(std::acos(1.0 / initialLoadFactor) * Units::kRadToDeg),
          gPerUnitStick_(gPerUnitStick) {}

    // One tick: sets aileron and elevator for `loadFactorCmd` and the given
    // throttle command, then steps the flight model (not ControlSystem).
    void Step(AircraftRig& rig, double loadFactorCmd, double throttleCmd) {
        const JSBSimPropertyNodes& p = rig.world.Components<JSBSimComponent>().Get(rig.id).props;
        double nz = p.nz->getDoubleValue();
        double altErrorFt = altitudeFt_ - p.hSlFt->getDoubleValue();
        double climbRateFtPerSec = -p.vDownFps->getDoubleValue();
        double qbarPsf = rig.Prop("aero/qbar-psf");

        // Negative elevator is nose-up (back stick).
        double gErrorG = loadFactorCmd - nz;
        elevatorIntegral_ = std::clamp(elevatorIntegral_ + 0.1 * gErrorG * AircraftRig::kDt, -1.0, 1.0);
        double feedForward = gPerUnitStick_ > 0.0 ? (loadFactorCmd - 1.0) / gPerUnitStick_ : 0.0;
        elevator_ = std::clamp(-(feedForward + 0.05 * gErrorG + elevatorIntegral_), -1.0, 1.0);

        // Below the target altitude or sinking: shallower bank puts more of
        // the lift vector vertical.
        double verticalError = 0.05 * altErrorFt - climbRateFtPerSec;
        bankBaseDeg_ = std::clamp(bankBaseDeg_ - 0.05 * verticalError * AircraftRig::kDt, 0.0, 88.0);
        double bankCmdDeg = std::clamp(bankBaseDeg_ - 0.2 * verticalError, 0.0, 88.0);
        double aileron = std::clamp((0.04 * (bankCmdDeg - p.phiDeg->getDoubleValue()) -
                                     0.01 * p.pRadSec->getDoubleValue() * Units::kRadToDeg) *
                                        std::min(1.0, 250.0 / std::max(qbarPsf, 1.0)),
                                    -1.0, 1.0);

        p.aileronCmd->setDoubleValue(aileron);
        p.elevatorCmd->setDoubleValue(elevator_);
        p.throttleCmd->setDoubleValue(throttleCmd);
        FlightDynamicsSystem::Update(rig.world, AircraftRig::kDt);
    }

    // The stick at its aft stop: the FCS will not give more G.
    bool StickAtStop() const { return elevator_ <= -0.999; }

private:
    double altitudeFt_;
    double bankBaseDeg_;
    double gPerUnitStick_;
    double elevatorIntegral_ = 0.0;
    double elevator_ = 0.0;
};

}  // namespace testrig
