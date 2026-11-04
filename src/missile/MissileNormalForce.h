#pragma once

#include <utility>
#include <vector>

// A missile type's normal-force coefficient CN(alpha, Mach), on the drag
// reference area: a lift slope plus one term for the body's extra lift at
// high angle of attack, pinned so the curve reaches CN_max at alpha_max:
//
//   CN = a alpha + b alpha^2,  a = CN_alpha(M),  b = (CN_max(M) - a alpha_max) / alpha_max^2
//
// The slope sets what turning costs (drag due to lift); CN_max sets the
// turn ceiling. Both tables are over Mach; for the AIM-120 they are computed
// by tools/fit/fit_normal_force.py from a component build-up of published
// theory (REFERENCE.md - Missile normal force and turn ceiling).
//
// Immutable, owned by MissileType like the zero-lift DragTable. See
// REFERENCE.md - Missile normal force and turn ceiling.
class MissileNormalForce {
public:
    // [Mach, value] rows in strictly increasing Mach, linearly interpolated
    // and held at the ends.
    using Table = std::vector<std::pair<double, double>>;

    MissileNormalForce() = default;
    // The catalog checks the tables (positive, increasing Mach, CN_max at
    // least CN_alpha alpha_max at every Mach) before constructing.
    MissileNormalForce(Table cnAlphaPerDeg, Table cnMax, double alphaMaxDeg, double referenceAreaFt2)
        : cnAlphaPerDeg_(std::move(cnAlphaPerDeg)),
          cnMax_(std::move(cnMax)),
          alphaMaxDeg_(alphaMaxDeg),
          referenceAreaFt2_(referenceAreaFt2) {}

    // CN at |alphaRad|, clamped to alpha_max.
    double Cn(double alphaRad, double mach) const;
    // The angle of attack that produces `cn`: the quadratic solved in closed
    // form, clamped to [0, alpha_max].
    double AlphaForCn(double cn, double mach) const;
    // CN at alpha_max: the turn ceiling's coefficient.
    double CnMax(double mach) const;

    double AlphaMaxRad() const;
    double ReferenceAreaFt2() const { return referenceAreaFt2_; }

private:
    // a (per degree) and b (per degree squared) at `mach`.
    std::pair<double, double> Coefficients(double mach) const;

    Table cnAlphaPerDeg_;
    Table cnMax_;
    double alphaMaxDeg_ = 0.0;
    double referenceAreaFt2_ = 0.0;
};
