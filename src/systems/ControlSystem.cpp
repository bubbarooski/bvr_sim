#include "ControlSystem.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "aircraft/AircraftEntity.h"
#include "components/AutopilotComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GLimitComponent.h"
#include "components/JSBSimComponent.h"
#include "ecs/ComponentStore.h"
#include "sim/Atmosphere.h"
#include "sim/FlightEnvelope.h"
#include "util/Units.h"

namespace {

// Wraps a heading error into (-180, 180] so a target of 10 deg and a
// current heading of 350 deg gives a 20 deg error, not -340.
double HeadingErrorDeg(double targetDeg, double currentDeg) {
    double error = std::fmod(targetDeg - currentDeg, 360.0);
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

// Cascaded P(+D) autopilot: heading/altitude error drive a target attitude
// angle, and an inner loop holds that attitude via a control surface.
// See REFERENCE.md - The cascaded autopilot for the gains' derivation.
// How the aircraft's FCS reads stick and throttle comes from its type
// (AircraftControlInterface); the loop gains below are tuned on the F-16
// and belong in the type once an airframe with different dynamics needs
// its own.
constexpr double kHeadingToBankGainPerDeg = 4.0;  // deg bank per deg heading error

// Turn-direction latch for near-reversal commands -- see
// AutopilotComponent::turnDirection. Both thresholds sit far above the
// ~21deg error at which the bank command saturates, so ordinary turns
// never reach them.
constexpr double kTurnLatchEngageDeg = 175.0;
constexpr double kTurnLatchReleaseDeg = 150.0;
// A heading command that moves this far in one tick is a new command, not
// the slow drift of a bearing to a waypoint.
constexpr double kNewHeadingCommandDeg = 10.0;

constexpr double kMaxBankDeg = 25.0;           // fallback with no GLimitComponent
constexpr double kAbsoluteMaxBankDeg = 85.0;   // acos(1/n) diverges near vertical
constexpr double kBankGainPerDeg = 0.04;       // aileron per deg of bank error
// Light, because JSBSim's F-16 FCS already closes its own roll-rate loop;
// heavier damping stacked on it caps roll rate near 25 deg/s.
constexpr double kRollRateDampingGainPerDegPerSec = 0.01;

// Bank-loop integral: ~2 s to work off a standing error (Ki/Kp = 0.5/s),
// active within 10 deg of the target and bounded to a quarter of full
// aileron. See AutopilotComponent::bankErrorIntegralDegSec.
constexpr double kBankIntegralGainPerDegSec = 0.02;
constexpr double kBankIntegralWindowDeg = 10.0;
constexpr double kBankIntegralLimitDegSec = 0.25 / kBankIntegralGainPerDegSec;

// Roll-loop gains are tuned for the low-to-moderate dynamic-pressure regime.
// Aileron control power rises with dynamic pressure, so a fixed gain that is
// well-damped at cruise becomes an under-damped roll limit cycle at high speed.
// Above this reference qbar the roll command is scaled by qRef/qbar to hold the
// loop gain -- and so the damping ratio -- roughly constant; at or below it the
// scale is 1, leaving the tuned regime untouched. See REFERENCE.md - The
// cascaded autopilot.
constexpr double kRollGainRefQbarPsf = 250.0;

// Vertical channel: altitude error -> climb-rate command -> vertical
// acceleration command -> load factor. Together the two gains give
// h'' + Kv h' + Kv Kh h = 0: wn ~0.17 rad/s, damping ~0.87. The climb-rate
// loop's crossover (~Kv) sits well below the FCS's G-tracking bandwidth;
// at 0.5 the lag of the G response at high altitude makes it oscillate.
constexpr double kAltitudeToClimbRatePerSec = 0.1;       // ft/s of climb per ft of error
constexpr double kClimbRateToVerticalAccelPerSec = 0.3;  // ft/s^2 per ft/s of climb-rate error
constexpr double kMaxFlightPathDeg = 60.0;               // caps the climb-rate command
// Floor on cos(bank) in n = .../cos(bank), so a transient past vertical
// asks for the envelope's G rather than a sign-flipped or infinite one.
constexpr double kMinCosBank = 0.1;

constexpr double kThrottleGainPerKt = 0.04;

using Units::kRadToDeg;

// This tick's G exceedance (if any): logs G_LIMIT_EXCEEDED, accumulates
// structural damage on the stored component, and destroys the entity on
// total structural failure -- the only case that returns true, so the
// caller stops commanding it. The load-factor command is already clamped to
// the envelope, so an exceedance is the airframe overshooting it.
bool RecordOverstress(World& world, SimContext& simContext, EntityID entity, GLimitComponent& storedGLimit,
                      double actualNz, double maxG, double minG, double dt, double simTime) {
    double overG = 0.0;
    if (actualNz > maxG) {
        overG = actualNz - maxG;
    } else if (actualNz < minG) {
        overG = minG - actualNz;
    }
    if (overG <= 0.0) return false;

    simContext.eventLog.Record(SimEvent{simTime, "G_LIMIT_EXCEEDED", entity,
                                          "Nz=" + std::to_string(actualNz) + " max=" + std::to_string(maxG) +
                                              " min=" + std::to_string(minG)});

    // Persisted to the STORED component, not the local live-weight copy,
    // so it survives across ticks.
    storedGLimit.cumulativeDamage =
        std::min(1.0, storedGLimit.cumulativeDamage + overG * dt * storedGLimit.damageAccumulationRate);
    if (storedGLimit.cumulativeDamage >= 1.0) {
        simContext.eventLog.Record(SimEvent{simTime, "STRUCTURAL_FAILURE", entity,
                                              "cumulative_damage=" + std::to_string(storedGLimit.cumulativeDamage)});
        world.Components<DestroyedComponent>().Add(entity, DestroyedComponent{});
        return true;
    }
    return false;
}

}  // namespace

namespace ControlSystem {

void Update(World& world, SimContext& simContext, double dt, double simTime) {
    // The loops are P(+D) on instantaneous error, except the bank loop's
    // integral near its target (AutopilotComponent::bankErrorIntegralDegSec).
    // That and the structural damage accumulator below are the two
    // time-integrated quantities, and are why dt is needed.
    ComponentStore<AutopilotComponent>& autopilots = world.Components<AutopilotComponent>();
    ComponentStore<JSBSimComponent>& jsbsimComponents = world.Components<JSBSimComponent>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    ComponentStore<GLimitComponent>& gLimits = world.Components<GLimitComponent>();
    const double temperatureOffsetC = simContext.environment.temperatureOffsetC;

    for (auto& [entity, autopilot] : autopilots) {
        const JSBSimComponent* jsbsim = jsbsimComponents.TryGet(entity);
        if (jsbsim == nullptr || destroyed.Has(entity)) continue;
        // Every flying entity has a type (FlightDynamicsSystem::Init).
        const AircraftType* type = AircraftEntity::TypeOf(world, entity);
        if (type == nullptr) continue;
        const AircraftControlInterface& controls = type->controls;
        const JSBSimPropertyNodes& props = jsbsim->props;
        GLimitComponent* gLimit = gLimits.TryGet(entity);

        double headingErrorDeg = HeadingErrorDeg(autopilot.targetHeadingDeg, props.psiDeg->getDoubleValue());

        // Past kTurnLatchEngageDeg the wrapped error's sign is numerically
        // unstable, so commit to the current direction until the remaining
        // turn is unambiguous. The engage/release gap is hysteresis. The
        // commitment is to one target: a jump in the command (the next
        // waypoint) drops it, or the aircraft would take the long way round
        // to a heading that now lies the other way.
        if (autopilot.turnDirection != 0 &&
            std::fabs(HeadingErrorDeg(autopilot.targetHeadingDeg, autopilot.lastTargetHeadingDeg)) >
                kNewHeadingCommandDeg) {
            autopilot.turnDirection = 0;
        }
        autopilot.lastTargetHeadingDeg = autopilot.targetHeadingDeg;
        if (autopilot.turnDirection != 0) {
            if (std::fabs(headingErrorDeg) < kTurnLatchReleaseDeg) {
                autopilot.turnDirection = 0;
            } else {
                headingErrorDeg = autopilot.turnDirection * std::fabs(headingErrorDeg);
            }
        } else if (std::fabs(headingErrorDeg) > kTurnLatchEngageDeg) {
            autopilot.turnDirection = headingErrorDeg >= 0.0 ? 1 : -1;
            headingErrorDeg = autopilot.turnDirection * std::fabs(headingErrorDeg);
        }
        double altErrorFt = autopilot.targetAltitudeFt - props.hSlFt->getDoubleValue();
        // TRUE airspeed, as targetSpeedKts is (REFERENCE.md - The cascaded
        // autopilot).
        double speedErrorKts = autopilot.targetSpeedKts - props.vtKts->getDoubleValue();

        double currentBankDeg = props.phiDeg->getDoubleValue();
        double rollRateDegPerSec = props.pRadSec->getDoubleValue() * kRadToDeg;

        // Bank cap inverted from this tick's available G (a coordinated
        // level turn at bank b needs n = 1/cos(b)). availableG <= 1 cannot
        // hold level flight at all, let alone bank: wings level. Weight is
        // live, so available G rises as fuel burns off.
        double speedFtPerSec = props.vtFps->getDoubleValue();
        double altitudeFt = props.hSlFt->getDoubleValue();
        double maxBankDegForEntity = kMaxBankDeg;
        double availableG = 0.0;
        double availableNegativeG = 0.0;
        if (gLimit != nullptr) {
            GLimitComponent liveWeightGLimit = *gLimit;
            liveWeightGLimit.weightLbf = props.weightLbs->getDoubleValue();
            // Maximum lift follows Mach where the model says it does
            // (supersonic, the F-16's falls with its lift-curve slope).
            if (props.clMaxMachFactor != nullptr) {
                liveWeightGLimit.clMax *= props.clMaxMachFactor->getDoubleValue();
            }
            availableG = FlightEnvelope::GetMaxAvailableG(speedFtPerSec, altitudeFt, liveWeightGLimit,
                                                          temperatureOffsetC);
            availableNegativeG = FlightEnvelope::GetMaxAvailableNegativeG(speedFtPerSec, altitudeFt,
                                                                          liveWeightGLimit, temperatureOffsetC);
            maxBankDegForEntity =
                availableG > 1.0 ? std::min(std::acos(1.0 / availableG) * kRadToDeg, kAbsoluteMaxBankDeg) : 0.0;
        }

        // --- Vertical channel: altitude -> climb rate -> load factor -
        // Works on the flight path, not pitch attitude, so it means the
        // same thing at any bank: the lift vector's vertical share is
        // n cos(bank), and holding a climb rate needs n cos(bank) - cos(gamma)
        // = a_v / g. A level turn falls out as n = 1/cos(bank) with no
        // separate turn compensation. See REFERENCE.md - The cascaded
        // autopilot. v-down-fps is positive downward, hence the negation.
        double climbRateFtPerSec = -props.vDownFps->getDoubleValue();
        double maxClimbRateFtPerSec = speedFtPerSec * std::sin(kMaxFlightPathDeg / kRadToDeg);
        double targetClimbRateFtPerSec =
            std::clamp(altErrorFt * kAltitudeToClimbRatePerSec, -maxClimbRateFtPerSec, maxClimbRateFtPerSec);
        double verticalAccelG =
            (targetClimbRateFtPerSec - climbRateFtPerSec) * kClimbRateToVerticalAccelPerSec / Units::kGravityFtPerSec2;
        double sinGamma = std::clamp(climbRateFtPerSec / std::max(speedFtPerSec, 1.0), -1.0, 1.0);
        double cosGamma = std::sqrt(1.0 - sinGamma * sinGamma);
        // The vertical share of load factor the flight path needs.
        double verticalLoadFactor = cosGamma + verticalAccelG;

        // How much of a body normal load factor is vertical, and what the
        // rest of the airframe's forces add. The accelerometers' vertical
        // specific force is exact kinematics:
        //   F_up = Nx sin(theta) - Ny sin(phi) cos(theta) + Nz cos(phi) cos(theta)
        // At a bank near 85 deg and 15-18 deg AoA the thrust along the nose
        // (Nx sin(theta)) and the side force (Ny) are 0.15-0.2 g each against
        // the 0.6 g that Nz cos(bank) gives; without them a steep turn holds
        // ~350 ft high. Nx and Ny are sensed, so thrust and drag changes are
        // included as they happen.
        const double thetaRad = props.thetaDeg->getDoubleValue() / kRadToDeg;
        const double phiRad = currentBankDeg / kRadToDeg;
        const double otherVerticalG = props.nx->getDoubleValue() * std::sin(thetaRad) -
                                      props.ny->getDoubleValue() * std::sin(phiRad) * std::cos(thetaRad);
        const double normalVerticalShare = std::max(std::cos(phiRad) * std::cos(thetaRad), kMinCosBank);

        // --- Outer loop: heading error -> target bank angle ---
        double targetBankDeg =
            std::clamp(headingErrorDeg * kHeadingToBankGainPerDeg, -maxBankDegForEntity, maxBankDegForEntity);

        // A hard turn -- the bank command at the envelope's cap -- banks to
        // exactly where the flight path's vertical load factor leaves the
        // whole envelope for the turn: n cos(bank) = vertical share at
        // n = available G. The load-factor command below (vertical share /
        // cos(bank), clamped to the envelope) then reaches the available G
        // as the bank does, and never more than the current bank can hold
        // level, so the nose cannot rise while rolling in.
        bool hardTurn = gLimit != nullptr && availableG > 1.0 &&
                        std::fabs(headingErrorDeg * kHeadingToBankGainPerDeg) >= maxBankDegForEntity;
        if (hardTurn) {
            double bankDeg = std::acos(std::clamp((verticalLoadFactor - otherVerticalG) / (availableG * std::cos(thetaRad)),
                                                  -1.0, 1.0)) * kRadToDeg;
            targetBankDeg = std::copysign(std::min(bankDeg, kAbsoluteMaxBankDeg), headingErrorDeg);
        }

        // --- Inner loop: bank-angle error -> aileron -
        // Positive aileron rolls right, so a positive bank error takes a
        // positive gain. Positive roll rate is a rightward roll already in
        // progress, damped by subtracting it.
        double bankErrorDeg = targetBankDeg - currentBankDeg;
        double qbarPsf =
            0.5 * Atmosphere::Lookup(altitudeFt, temperatureOffsetC).densitySlugPerFt3 * speedFtPerSec * speedFtPerSec;
        double rollGainScale = std::min(1.0, kRollGainRefQbarPsf / std::max(qbarPsf, 1.0));
        // Integral action only near the target bank, where the
        // proportional loop's few-degree standing error lives; a large
        // error (rolling in, reversing) clears it, so it cannot wind up.
        if (std::fabs(bankErrorDeg) < kBankIntegralWindowDeg) {
            autopilot.bankErrorIntegralDegSec = std::clamp(autopilot.bankErrorIntegralDegSec + bankErrorDeg * dt,
                                                           -kBankIntegralLimitDegSec, kBankIntegralLimitDegSec);
        } else {
            autopilot.bankErrorIntegralDegSec = 0.0;
        }
        double aileron = std::clamp((bankErrorDeg * kBankGainPerDeg +
                                     autopilot.bankErrorIntegralDegSec * kBankIntegralGainPerDegSec -
                                     rollRateDegPerSec * kRollRateDampingGainPerDegPerSec) *
                                        rollGainScale,
                                    -1.0, 1.0);

        // Load factor: the vertical channel's, through the current bank.
        double loadFactorCmd = (verticalLoadFactor - otherVerticalG) / normalVerticalShare;
        if (gLimit != nullptr) loadFactorCmd = std::clamp(loadFactorCmd, availableNegativeG, availableG);

        // The FCS reads the stick as a G command; negative stick is aft.
        // It clips the stick to its own travel, so no envelope without a
        // GLimitComponent is needed here.
        double elevator = std::clamp(-(loadFactorCmd - 1.0) / controls.gPerUnitStick, -1.0, 1.0);

        // Overstress bookkeeping, for entities with a GLimitComponent only.
        // accelerations/Nz is the standard aviation sign convention (level
        // flight reads +1.0), NOT n-pilot-z-norm, which is flipped.
        if (gLimit != nullptr) {
            double actualNz = props.nz->getDoubleValue();
            if (RecordOverstress(world, simContext, entity, *gLimit, actualNz, availableG, availableNegativeG, dt,
                                  simTime)) {
                continue;
            }
        }

        // Trim throttle as feed-forward (JSBSimComponent::trimThrottle),
        // captured per aircraft.
        //
        // The top of the range is the type's: the command for the most
        // thrust its engine gives. The F-16's is 1.0 even though JSBSim's
        // augmented turbines take a 0-2 throttle POSITION, because its FCS
        // has a gain-of-2 stage between command and position.
        double throttle =
            std::clamp(jsbsim->trimThrottle + speedErrorKts * kThrottleGainPerKt, 0.0, controls.maxThrottleCmd);

        props.aileronCmd->setDoubleValue(aileron);
        props.elevatorCmd->setDoubleValue(elevator);
        props.throttleCmd->setDoubleValue(throttle);
    }
}

}  // namespace ControlSystem
