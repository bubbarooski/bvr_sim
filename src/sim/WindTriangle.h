#pragma once

#include <cmath>

#include "util/Units.h"
#include "util/Vec3.h"

namespace WindTriangle {

// The compass heading that makes good `trackDeg` over the ground at true
// airspeed `trueAirspeedFps` through horizontal wind `windFps` (world frame:
// x east, z north; y ignored). The aircraft crabs into the crosswind just
// enough to cancel it: its air velocity is minus the crosswind plus whatever
// of the airspeed is left over, along the track. The along-track wind only
// changes the ground speed, so it plays no part in the heading.
//
// A crosswind at least as fast as the aircraft cannot be cancelled; the best
// it can do is point straight into it. With no wind the track is returned
// unchanged, bit for bit.
inline double HeadingForTrackDeg(double trackDeg, double trueAirspeedFps, const Vec3& windFps) {
    if (windFps.x() == 0.0 && windFps.z() == 0.0) return trackDeg;
    double trackRad = trackDeg * Units::kDegToRad;
    double trackEast = std::sin(trackRad);
    double trackNorth = std::cos(trackRad);

    double along = windFps.x() * trackEast + windFps.z() * trackNorth;
    double crossEast = windFps.x() - along * trackEast;
    double crossNorth = windFps.z() - along * trackNorth;
    double cross = std::hypot(crossEast, crossNorth);

    double alongAirspeed =
        cross < trueAirspeedFps ? std::sqrt(trueAirspeedFps * trueAirspeedFps - cross * cross) : 0.0;
    double airEast = -crossEast + alongAirspeed * trackEast;
    double airNorth = -crossNorth + alongAirspeed * trackNorth;

    double headingDeg = std::atan2(airEast, airNorth) * Units::kRadToDeg;
    return headingDeg < 0.0 ? headingDeg + 360.0 : headingDeg;
}

}  // namespace WindTriangle
