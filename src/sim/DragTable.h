#pragma once

#include <utility>
#include <vector>

// A missile body's zero-lift drag coefficient against Mach, linearly
// interpolated and held at the end values outside the table. The values
// belong to a missile type (MissileType::drag); this is only the lookup.
// See REFERENCE.md - Aerodynamic drag.
class DragTable {
public:
    struct Point {
        double mach;
        double cd;
    };

    DragTable() = default;
    // `points` must be non-empty and in strictly increasing Mach.
    explicit DragTable(std::vector<Point> points) : points_(std::move(points)) {}

    double Lookup(double mach) const;
    const std::vector<Point>& Points() const { return points_; }
    bool Empty() const { return points_.empty(); }

private:
    std::vector<Point> points_;
};
