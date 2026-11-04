#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "util/Vec3.h"

// The one seeded RNG stream for the whole sim (owned by SimContext). Every
// stochastic system draws from this -- no rand(), no per-system instances.
//
// The distributions are written out here rather than taken from <random>.
// std::mt19937_64 is fully specified by the standard, but
// std::normal_distribution and std::uniform_real_distribution are not:
// libstdc++ and MSVC's STL turn the same engine output into different
// numbers, so one seed would give one stream on Windows and another on Linux.
//
// Every operation below is one IEEE 754 requires to be correctly rounded
// (+ - * / sqrt) or exact (frexp, integer shifts), evaluated in a fixed
// order, so the stream is bit-identical on every conforming platform. That
// is also why the logarithm is our own: std::log's last bit differs between
// C runtimes. The build turns off floating-point contraction on GCC/Clang
// (-ffp-contract=off), which would otherwise fuse a*b+c into an FMA and
// round it differently on hardware that has one; MSVC does not contract
// under its default /fp:precise.
class Rng {
public:
    void Seed(uint64_t seed) { engine_.seed(seed); }

    // Marsaglia's polar method. Rejection is decided by arithmetic alone,
    // so every platform consumes the same number of engine draws. Only one
    // of the pair is returned, which keeps each call's consumption a pure
    // function of the stream rather than of a hidden cached value.
    double NextGaussian(double mean, double sigma) {
        // A zero-variance distribution is just its own mean, and "no
        // jitter" is a legitimate scenario choice that must not consume a
        // draw -- or turning one noise source off would shift every draw
        // after it.
        if (sigma <= 0.0) return mean;
        double x = 0.0;
        double s = 0.0;
        do {
            x = 2.0 * NextUnit() - 1.0;
            double y = 2.0 * NextUnit() - 1.0;
            s = x * x + y * y;
        } while (s >= 1.0 || s == 0.0);
        return mean + sigma * x * std::sqrt(-2.0 * PortableLog(s) / s);
    }

    // Natural log of a positive finite x, from basic arithmetic only.
    // x = m * 2^e with m in [sqrt(1/2), sqrt(2)), so log x = e*ln2 + log m,
    // and log m = 2*atanh(t) with t = (m-1)/(m+1), |t| <= 0.1716. The odd
    // atanh series then converges below double precision by t^25. Accurate
    // to a few ulp -- not correctly rounded, but identical everywhere,
    // which is the property the stream needs.
    static double PortableLog(double x) {
        int exponent = 0;
        double m = std::frexp(x, &exponent);  // exact; m in [0.5, 1)
        constexpr double kSqrtHalf = 0.70710678118654752440;
        if (m < kSqrtHalf) {
            m *= 2.0;  // exact
            --exponent;
        }
        double t = (m - 1.0) / (m + 1.0);
        double t2 = t * t;
        // Horner over 1/(2k+1), k = kLastTerm down to 0.
        constexpr int kLastTerm = 12;
        double series = 1.0 / (2.0 * kLastTerm + 1.0);
        for (int k = kLastTerm - 1; k >= 0; --k) series = series * t2 + 1.0 / (2.0 * k + 1.0);
        // ln 2 split so e*ln2Hi is exact for any double exponent.
        constexpr double kLn2Hi = 6.93147180369123816490e-01;
        constexpr double kLn2Lo = 1.90821492927058770002e-10;
        double e = static_cast<double>(exponent);
        return e * kLn2Hi + (e * kLn2Lo + 2.0 * t * series);
    }

    // Three independent zero-mean draws, taken in x, y, z order -- the
    // order is part of the stream, so it is spelled out rather than left to
    // an initializer list.
    Vec3 NextGaussianVec3(double sigma) {
        Vec3 v;
        v.x() = NextGaussian(0.0, sigma);
        v.y() = NextGaussian(0.0, sigma);
        v.z() = NextGaussian(0.0, sigma);
        return v;
    }

    // Uniform on [a, b).
    double NextUniform(double a, double b) { return a + (b - a) * NextUnit(); }

private:
    // Uniform on [0, 1): the top 53 bits of one engine draw, scaled by
    // 2^-53, so every representable step is equally likely and exact.
    double NextUnit() {
        constexpr int kMantissaBits = 53;
        return static_cast<double>(engine_() >> (64 - kMantissaBits)) * 0x1.0p-53;
    }

    std::mt19937_64 engine_;
};
