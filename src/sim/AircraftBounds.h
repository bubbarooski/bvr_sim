#pragma once

#include <string>

#include "util/Vec3.h"

// An aircraft's airframe as a rectangular bounding box, so a hit is
// measured against the nearest point on the aircraft rather than its
// center point. Each aircraft type carries its own (AircraftType::hitBox);
// a target with no type is a point (all half-extents zero). Body axes are
// +X starboard, +Y up, +Z nose.
namespace AircraftBounds {

struct HitBox {
    double halfSpanFt = 0.0;
    double halfHeightFt = 0.0;
    double halfLengthFt = 0.0;
};

// Distance from queryPos to the nearest point on the oriented bounding
// box centered at boxCenter with attitude boxEulerDeg (roll, pitch, yaw
// degrees -- same convention as Transform.euler). 0.0 if queryPos is
// inside the box.
double DistanceToSurfaceFt(const HitBox& box, const Vec3& queryPos, const Vec3& boxCenter, const Vec3& boxEulerDeg);

// Same distance from a point already in the box's body frame (as
// ToBodyFrameFt returns), for callers that need that point too.
// DistanceToSurfaceFt is exactly this composed with ToBodyFrameFt.
double DistanceToSurfaceFromBodyFt(const HitBox& box, const Vec3& bodyPointFt);

// queryPos in the target's body frame relative to its center, ft: the
// transform DistanceToSurfaceFt uses, so the reported impact point comes
// from the same geometry as the hit decision.
Vec3 ToBodyFrameFt(const Vec3& queryPos, const Vec3& boxCenter, const Vec3& boxEulerDeg);

// A world-space direction in the body frame of an aircraft at attitude
// eulerDeg (roll, pitch, yaw degrees): the same rotation, no offset.
Vec3 WorldToBodyDirection(const Vec3& worldDir, const Vec3& eulerDeg);

// Human-readable airframe region for a body-frame point (as returned by
// ToBodyFrameFt). Works for points inside the box (a real strike) and
// outside it (a near miss -- tells you which side it went past), since
// both are just positions in the same frame.
std::string ImpactRegionName(const HitBox& box, const Vec3& bodyPointFt);

// Where the straight segment from body-frame point `from` to `to` first
// enters the box (slab method), into `entry`. False if it misses the box.
// A segment starting inside enters at `from`. A missile crossing the
// airframe within one tick never samples a point inside it, and once
// inside every point is at surface distance 0, so this is how the point of
// impact is found for a pass straight through.
bool SegmentEntryFromBodyFt(const HitBox& box, const Vec3& from, const Vec3& to, Vec3& entry);

}  // namespace AircraftBounds
