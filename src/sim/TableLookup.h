#pragma once

#include <cstddef>

// Clamped piecewise-linear lookup over a small table (a std::array or
// std::vector of rows) sorted by ascending key, shared by the drag table
// and the missile normal-force table.
// Find locates the bracketing pair once; Interpolate then reads any number
// of columns from it:
//
//   TableLookup::Bracket b = TableLookup::Find(kTable, altitudeFt, &Point::altitudeFt);
//   double rho = TableLookup::Interpolate(kTable, b, &Point::densitySlugPerFt3);
//
// Outside the table the end row is returned unchanged -- no extrapolation.
namespace TableLookup {

// Rows lo and hi bracket the key, t in [0, 1] between them. At a clamp
// lo == hi and t == 0, which Interpolate returns exactly.
struct Bracket {
    std::size_t lo;
    std::size_t hi;
    double t;
};

// `table` must not be empty.
template <typename Table, typename Row>
Bracket Find(const Table& table, double key, double Row::*keyColumn) {
    const std::size_t n = table.size();
    if (key <= table.front().*keyColumn) return {0, 0, 0.0};
    if (key >= table.back().*keyColumn) return {n - 1, n - 1, 0.0};
    for (std::size_t i = 0; i + 1 < n; ++i) {
        double lo = table[i].*keyColumn;
        double hi = table[i + 1].*keyColumn;
        if (key <= hi) return {i, i + 1, (key - lo) / (hi - lo)};
    }
    return {n - 1, n - 1, 0.0};  // only reachable for a NaN key
}

template <typename Table, typename Row>
double Interpolate(const Table& table, const Bracket& bracket, double Row::*column) {
    double a = table[bracket.lo].*column;
    double b = table[bracket.hi].*column;
    return a + bracket.t * (b - a);
}

// The one-column case.
template <typename Table, typename Row>
double Lookup(const Table& table, double key, double Row::*keyColumn, double Row::*column) {
    return Interpolate(table, Find(table, key, keyColumn), column);
}

}  // namespace TableLookup
