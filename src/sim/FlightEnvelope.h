#pragma once

#include <algorithm>
#include <stdexcept>

#include "components/GLimitComponent.h"
#include "components/MissileGLimitComponent.h"
#include "sim/Atmosphere.h"
#include "util/Units.h"

// Aircraft V-n envelope and the missile's analogous control-authority
// envelope. Available G rises with V^2 at low speed (aerodynamically
// limited) and flattens at the structural cap above corner velocity -- the
// classic V-n diagram shape. See REFERENCE.md - The Flight Envelope.
//
// Every function takes the scenario's ISA temperature offset, since the
// lift limit depends on density. It defaults to a standard day for test
// rigs; the systems pass SimContext::environment.
namespace FlightEnvelope {

// Lift at CLmax over weight: the load factor the wing can generate before
// any structural cap applies. `g.weightLbf` must hold the aircraft's live
// weight; an unset (zero) weight would divide by zero, so it is refused.
inline double AircraftAeroLimitG(double speedFtPerSec, double altitudeFt, const GLimitComponent& g,
                                 double temperatureOffsetC = 0.0) {
    if (!(g.weightLbf > 0.0)) {
        throw std::logic_error("FlightEnvelope: GLimitComponent::weightLbf must be set to the aircraft's weight");
    }
    double rho = Atmosphere::Lookup(altitudeFt, temperatureOffsetC).densitySlugPerFt3;
    return (0.5 * rho * speedFtPerSec * speedFtPerSec * g.clMax * g.wingAreaFt2) / g.weightLbf;
}

// The positive lift limit when the lift the FCS allows falls with load
// factor, CL(n) = clMax - L (n - 1): n = CL(n) Q with Q = q S / W solves to
// n = (clMax + L) Q / (1 + L Q). L = 0 is AircraftAeroLimitG.
inline double AircraftAeroLimitPositiveG(double speedFtPerSec, double altitudeFt, const GLimitComponent& g,
                                         double temperatureOffsetC = 0.0) {
    double q = AircraftAeroLimitG(speedFtPerSec, altitudeFt, g, temperatureOffsetC) / g.clMax;  // q S / W
    return (g.clMax + g.clMaxLossPerG) * q / (1.0 + g.clMaxLossPerG * q);
}

inline double GetMaxAvailableG(double speedFtPerSec, double altitudeFt, const GLimitComponent& g,
                               double temperatureOffsetC = 0.0) {
    return std::min(AircraftAeroLimitPositiveG(speedFtPerSec, altitudeFt, g, temperatureOffsetC),
                    g.structuralGLimitPos);
}

// The negative-G mirror: same aerodynamic lift-limit magnitude (no
// separate negative CLmax is modeled), capped by the negative structural
// limit. Returns a negative number.
inline double GetMaxAvailableNegativeG(double speedFtPerSec, double altitudeFt, const GLimitComponent& g,
                                       double temperatureOffsetC = 0.0) {
    return std::max(-AircraftAeroLimitG(speedFtPerSec, altitudeFt, g, temperatureOffsetC), g.structuralGLimitNeg);
}

// The missile's normal force at its largest angle of attack over weight,
// CN_max(M) q S_ref / W -- the aircraft's q CLmax S / W, for the missile:
// the load factor the airframe can generate before the structural cap.
// Takes current mass directly (mass depletes during the burn);
// currentMassSlug must be > 0, which dry mass alone guarantees. Zero with
// no normal-force model. See REFERENCE.md - Missile normal force and turn
// ceiling.
inline double MissileAeroLimitG(double speedFtPerSec, double altitudeFt, double currentMassSlug,
                                const MissileGLimitComponent& g, double temperatureOffsetC = 0.0) {
    if (g.normalForce == nullptr) return 0.0;
    const MissileNormalForce& normalForce = *g.normalForce;
    Atmosphere::AtmosphereState atmosphere = Atmosphere::Lookup(altitudeFt, temperatureOffsetC);
    double cnMax = normalForce.CnMax(speedFtPerSec / atmosphere.speedOfSoundFtPerSec);
    double dynamicPressure = 0.5 * atmosphere.densitySlugPerFt3 * speedFtPerSec * speedFtPerSec;
    return dynamicPressure * cnMax * normalForce.ReferenceAreaFt2() / (currentMassSlug * Units::kGravityFtPerSec2);
}

// Same V-n shape as the aircraft's: aerodynamically limited at low speed,
// flat at the structural cap above the corner. A component without a
// normal-force model is structurally limited only.
inline double GetMissileMaxAvailableG(double speedFtPerSec, double altitudeFt, double currentMassSlug,
                                       const MissileGLimitComponent& g, double temperatureOffsetC = 0.0) {
    if (g.normalForce == nullptr) return g.structuralGLimit;
    return std::min(MissileAeroLimitG(speedFtPerSec, altitudeFt, currentMassSlug, g, temperatureOffsetC),
                    g.structuralGLimit);
}

}  // namespace FlightEnvelope
