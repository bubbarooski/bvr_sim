#pragma once

#include <array>

// How a target's return fluctuates from one look to the next. See
// REFERENCE.md - Probability of detection.
enum class FluctuationModel {
    STEADY,      // Swerling 0 / Marcum: a constant return
    SWERLING_1,  // many comparable scatterers: exponential power, fresh each look
};

// One body-axis ellipsoid of a target's RCS shape, semi-axes in feet
// (span = starboard, height = up, length = nose). See REFERENCE.md - Radar
// cross-section.
struct RcsEllipsoid {
    double spanFt = 0.0;
    double heightFt = 0.0;
    double lengthFt = 0.0;
};

// More than any airframe's estimate needs (the F-16's is four parts); a
// fixed array keeps the per-look copy free of heap allocation.
inline constexpr int kMaxRcsEllipsoids = 8;

// A target's radar signature, a property of the target rather than of any
// sensor. Attach it to any entity that should be radar-detectable.
struct RadarCrossSection {
    // Mean RCS. With a shape: the nose-on value, which the shape scales by
    // its projected area toward the radar. Without one (shapeCount 0): the
    // same from every direction.
    double staticRcsM2 = 0.0;
    FluctuationModel fluctuation = FluctuationModel::SWERLING_1;
    std::array<RcsEllipsoid, kMaxRcsEllipsoids> shape{};
    int shapeCount = 0;
    // The teaching knob: multiplies the mean RCS every radar and seeker
    // sees, after aspect. Detection range goes as its fourth root.
    double rcsScale = 1.0;
};
