#include "sim/AircraftBounds.h"

#include <algorithm>
#include <cmath>

#include "util/Units.h"
#include "util/Vec3.h"

namespace AircraftBounds {

namespace {

using Units::kDegToRad;

// How far out, in half-extents, a point still gets an airframe region
// rather than a bare bearing: roughly where a warhead going off stops being
// "alongside the port wing" and is just a miss. Deliberately generous, so
// proximity kills keep a region name. A labelling choice, not a damage model.
constexpr double kNearFieldExtents = 3.0;

// Region boundaries, in half-extents (see ImpactRegionName).
constexpr double kWingFraction = 0.6;          // |x| beyond this is a wing
constexpr double kNoseFraction = 0.55;         // z beyond this is the nose, below minus it the tail
constexpr double kMidFuselageFraction = 0.15;  // |z| within this is mid fuselage
constexpr double kVerticalFraction = 0.6;      // |y| beyond this adds _upper / _lower

// A segment component smaller than this (ft) runs parallel to that slab.
constexpr double kParallelTolerance = 1e-12;

// World offset from the box center -> body frame: the inverse of
// RenderSystem's RotateBodyToWorld (yaw, then pitch, then roll undone, each
// by its transpose). Double precision, unlike the render-side float copy.
Vec3 WorldToBody(const Vec3& worldRelative, double rollDeg, double pitchDeg, double yawDeg) {
    double rollRad = rollDeg * kDegToRad;
    double pitchRad = pitchDeg * kDegToRad;
    double yawRad = yawDeg * kDegToRad;

    double cy = std::cos(yawRad), sy = std::sin(yawRad);
    double x2 = worldRelative.x() * cy - worldRelative.z() * sy;
    double z2 = worldRelative.x() * sy + worldRelative.z() * cy;
    double y2 = worldRelative.y();

    double cp = std::cos(pitchRad), sp = std::sin(pitchRad);
    double y1 = y2 * cp - z2 * sp;
    double z1 = y2 * sp + z2 * cp;
    double x1 = x2;

    double cr = std::cos(rollRad), sr = std::sin(rollRad);
    double x = x1 * cr - y1 * sr;
    double y = x1 * sr + y1 * cr;
    double z = z1;

    return Vec3{x, y, z};
}

}  // namespace

Vec3 ToBodyFrameFt(const Vec3& queryPos, const Vec3& boxCenter, const Vec3& boxEulerDeg) {
    Vec3 relative{queryPos.x() - boxCenter.x(), queryPos.y() - boxCenter.y(), queryPos.z() - boxCenter.z()};
    return WorldToBody(relative, boxEulerDeg.x(), boxEulerDeg.y(), boxEulerDeg.z());
}

Vec3 WorldToBodyDirection(const Vec3& worldDir, const Vec3& eulerDeg) {
    return WorldToBody(worldDir, eulerDeg.x(), eulerDeg.y(), eulerDeg.z());
}

// Regions are decided in half-extent-normalized coordinates: an airframe is
// far longer than it is wide or tall, so in raw feet almost everything would
// read as a nose or tail hit. Wings are tested first -- far out on a wing is
// a wing hit wherever it sits fore and aft. A telemetry label, not a damage
// model.
std::string ImpactRegionName(const HitBox& box, const Vec3& bodyPointFt) {
    // A point target has no airframe regions, only a bearing.
    if (box.halfSpanFt <= 0.0 || box.halfHeightFt <= 0.0 || box.halfLengthFt <= 0.0) {
        return "point_target";
    }
    double nx = bodyPointFt.x() / box.halfSpanFt;     // -1 port .. +1 starboard
    double ny = bodyPointFt.y() / box.halfHeightFt;   // -1 belly .. +1 upper
    double nz = bodyPointFt.z() / box.halfLengthFt;   // -1 tail  .. +1 nose

    // Beyond the near field a region name would make a miss by kilometres
    // read like a hit, so report a bearing instead.
    double magnitude = std::max(std::max(std::fabs(nx), std::fabs(ny)), std::fabs(nz));
    if (magnitude > kNearFieldExtents) {
        // Dominant axis only -- at this distance the other two are noise.
        if (std::fabs(nx) >= std::fabs(ny) && std::fabs(nx) >= std::fabs(nz)) {
            return nx > 0.0 ? "clear_starboard" : "clear_port";
        }
        if (std::fabs(ny) >= std::fabs(nz)) {
            return ny > 0.0 ? "clear_above" : "clear_below";
        }
        return nz > 0.0 ? "clear_ahead" : "clear_astern";
    }

    std::string region;
    if (nx > kWingFraction) {
        region = "starboard_wing";
    } else if (nx < -kWingFraction) {
        region = "port_wing";
    } else if (nz > kNoseFraction) {
        region = "nose";
    } else if (nz > kMidFuselageFraction) {
        region = "forward_fuselage";
    } else if (nz > -kMidFuselageFraction) {
        region = "mid_fuselage";
    } else if (nz > -kNoseFraction) {
        region = "aft_fuselage";
    } else {
        region = "tail";
    }

    if (ny > kVerticalFraction) {
        region += "_upper";
    } else if (ny < -kVerticalFraction) {
        region += "_lower";
    }

    // Outside the box but in the near field: went off alongside, not a strike.
    if (magnitude > 1.0) region = "near_" + region;
    return region;
}

bool SegmentEntryFromBodyFt(const HitBox& box, const Vec3& from, const Vec3& to, Vec3& entry) {
    const double lo[3] = {-box.halfSpanFt, -box.halfHeightFt, -box.halfLengthFt};
    const double hi[3] = {box.halfSpanFt, box.halfHeightFt, box.halfLengthFt};
    const double a[3] = {from.x(), from.y(), from.z()};
    const double d[3] = {to.x() - from.x(), to.y() - from.y(), to.z() - from.z()};
    double tEnter = 0.0, tExit = 1.0;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::fabs(d[axis]) < kParallelTolerance) {
            if (a[axis] < lo[axis] || a[axis] > hi[axis]) return false;  // parallel and outside this slab
            continue;
        }
        double t0 = (lo[axis] - a[axis]) / d[axis];
        double t1 = (hi[axis] - a[axis]) / d[axis];
        if (t0 > t1) std::swap(t0, t1);
        tEnter = std::max(tEnter, t0);
        tExit = std::min(tExit, t1);
        if (tEnter > tExit) return false;
    }
    entry = from + (to - from) * tEnter;
    return true;
}

double DistanceToSurfaceFromBodyFt(const HitBox& box, const Vec3& local) {
    // Closest point on the box, in its own frame; rotation preserves
    // distance, so no need to go back to world space.
    Vec3 clamped{std::clamp(local.x(), -box.halfSpanFt, box.halfSpanFt),
                 std::clamp(local.y(), -box.halfHeightFt, box.halfHeightFt),
                 std::clamp(local.z(), -box.halfLengthFt, box.halfLengthFt)};
    return (local - clamped).norm();
}

double DistanceToSurfaceFt(const HitBox& box, const Vec3& queryPos, const Vec3& boxCenter, const Vec3& boxEulerDeg) {
    return DistanceToSurfaceFromBodyFt(box, ToBodyFrameFt(queryPos, boxCenter, boxEulerDeg));
}

}  // namespace AircraftBounds
