#pragma once

// One aircraft's speed-dependent V-n envelope, and the structural damage it
// has taken. English units, like every other flight-physics quantity here.
// See REFERENCE.md - The Flight Envelope.
//
// Holds no aircraft data of its own: an aircraft's starts as its type's
// (AircraftEntity::MakeGLimit), then a scenario may override fields, and
// overstress changes it in flight. A default-constructed one is no envelope
// at all (zero lift, zero structure) -- build it from a type.
struct GLimitComponent {
    double clMax = 0.0;          // in 1 g flight
    double clMaxLossPerG = 0.0;  // lift the FCS gives up per G above 1 (G/AoA blend)
    double wingAreaFt2 = 0.0;
    double weightLbf = 0.0;  // the aircraft's live weight: the caller sets it (ControlSystem reads JSBSim's); 0 is "unset" and FlightEnvelope refuses it
    double structuralGLimitPos = 0.0;
    double structuralGLimitNeg = 0.0;

    // Cumulative structural damage from overstress, accrued in proportion
    // to G over the limit times time. Reaching 1.0 is structural failure.
    double cumulativeDamage = 0.0;
    double damageAccumulationRate = 0.0;  // damage per g-second over the limit
};
