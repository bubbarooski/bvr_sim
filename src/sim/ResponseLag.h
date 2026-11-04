#pragma once

#include "util/Numeric.h"
#include "util/Vec3.h"

// The missile's one guidance response time: a first-order lag between the
// lateral acceleration the autopilot demands and the one the airframe
// delivers. It stands for the whole loop -- seeker tracking, fin servos and
// airframe response -- as a single time constant, the textbook single-lag
// guidance system (Palumbo et al. 2010, Eqs. 41-42). See REFERENCE.md -
// Guidance response time.
namespace ResponseLag {

constexpr double kMinAirspeedFtPerSec = 0.001;  // below this there is no airflow, so no lift

// Advances `achieved` one tick toward `demanded` and returns it. tau 0 is
// instantaneous: the demand comes straight back.
//
// The lagged acceleration is the fins' lift, fixed in the airframe, so it
// turns with the body: carried into the plane across the current air
// velocity, keeping its magnitude, before lagging toward the new demand.
// Held in world axes instead, it would gain a part along the flight path
// as the missile turned -- free thrust or braking that lift cannot make. A
// part lying wholly along the axis has nowhere to rotate to and is dropped.
inline Vec3 Step(Vec3& achieved, const Vec3& demanded, const Vec3& airVel, double tau, double dt) {
    if (tau <= 0.0) {
        achieved = demanded;
        return achieved;
    }
    double airspeed = airVel.norm();
    if (airspeed > kMinAirspeedFtPerSec) {
        Vec3 axis = airVel * (1.0 / airspeed);
        Vec3 across = achieved - axis * achieved.dot(axis);
        double heldMag = achieved.norm();
        double acrossMag = across.norm();
        achieved = acrossMag <= Numeric::kNegligibleFraction * heldMag ? Vec3{} : (across * (heldMag / acrossMag));
    }
    double alpha = dt / (tau + dt);
    achieved += (demanded - achieved) * alpha;
    return achieved;
}

}  // namespace ResponseLag
