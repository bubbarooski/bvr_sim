// The aircraft V-n envelope against its closed form, and the weight it needs.

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "components/GLimitComponent.h"
#include "sim/Atmosphere.h"
#include "sim/FlightEnvelope.h"

namespace {

GLimitComponent Fighter(double weightLbf) {
    GLimitComponent g;
    g.clMax = 1.65;
    g.clMaxLossPerG = 0.049;
    g.wingAreaFt2 = 300.0;
    g.structuralGLimitPos = 9.0;
    g.structuralGLimitNeg = -3.0;
    g.weightLbf = weightLbf;
    return g;
}

}  // namespace

// n = q S CLmax / W, from the atmosphere's own density.
TEST(FlightEnvelope, AeroLimitIsLiftAtClMaxOverWeight) {
    const double speed = 600.0, altitude = 20000.0, weight = 22000.0;
    GLimitComponent g = Fighter(weight);
    double rho = Atmosphere::Lookup(altitude).densitySlugPerFt3;
    double expected = 0.5 * rho * speed * speed * g.clMax * g.wingAreaFt2 / weight;
    EXPECT_DOUBLE_EQ(FlightEnvelope::AircraftAeroLimitG(speed, altitude, g), expected);
    // Negative G mirrors the same lift limit.
    EXPECT_DOUBLE_EQ(FlightEnvelope::GetMaxAvailableNegativeG(speed, altitude, g),
                     std::max(-expected, g.structuralGLimitNeg));
}

// CL falls with load factor, CL(n) = clMax - L (n - 1), so n solves
// n = CL(n) Q: the returned limit must satisfy that equation.
TEST(FlightEnvelope, PositiveLimitSolvesTheLoadFactorDependentLiftEquation) {
    GLimitComponent g = Fighter(22000.0);
    double n = FlightEnvelope::AircraftAeroLimitPositiveG(400.0, 15000.0, g);
    double q = FlightEnvelope::AircraftAeroLimitG(400.0, 15000.0, g) / g.clMax;
    EXPECT_NEAR(n, (g.clMax - g.clMaxLossPerG * (n - 1.0)) * q, 1e-12);
}

// weightLbf is the aircraft's live weight, set by whoever calls the
// envelope. Left at zero it divides by zero and, for a type whose CL falls
// with G, turns inf / inf into NaN that `std::min` then passes through.
TEST(FlightEnvelope, AnUnsetWeightIsRefusedNotDividedBy) {
    GLimitComponent unset = Fighter(0.0);
    EXPECT_THROW(FlightEnvelope::AircraftAeroLimitG(400.0, 10000.0, unset), std::logic_error);
    EXPECT_THROW(FlightEnvelope::GetMaxAvailableG(400.0, 10000.0, unset), std::logic_error);
    EXPECT_THROW(FlightEnvelope::GetMaxAvailableNegativeG(400.0, 10000.0, unset), std::logic_error);
    GLimitComponent negative = Fighter(-1.0);
    EXPECT_THROW(FlightEnvelope::AircraftAeroLimitG(400.0, 10000.0, negative), std::logic_error);
}
