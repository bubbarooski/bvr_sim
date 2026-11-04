// The normal-force model CN = a alpha + b alpha^2: the AIM-120's curve is
// zero at zero alpha, rises monotonically and reaches its CN_max table at
// alpha_max; the closed-form alpha inversion used for drag due to lift
// round-trips, including a pure straight line (b = 0).

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "missile/MissileCatalog.h"
#include "util/Units.h"

namespace {

const MissileNormalForce& Aim120NormalForce() {
    static std::shared_ptr<const MissileType> type = MissileCatalog::Default().Get("aim120");
    return *type->aero.normalForce;
}

double Rad(double deg) { return deg * Units::kDegToRad; }

}  // namespace

// At a table row the interpolation is exact: CN(alpha_max) is that row's
// CN_max, and a small alpha sits close to the slope's straight line.
TEST(MissileNormalForce, ReachesCnMaxAtAlphaMax) {
    const MissileNormalForce& nf = Aim120NormalForce();
    EXPECT_DOUBLE_EQ(nf.AlphaMaxRad(), Rad(30.0));
    EXPECT_NEAR(nf.Cn(Rad(30.0), 2.0), 20.252, 1e-12);
    EXPECT_NEAR(nf.CnMax(2.0), 20.252, 1e-12);
    EXPECT_NEAR(nf.Cn(Rad(30.0), 0.8), 10.674, 1e-12);
    double b = (20.252 - 0.2068 * 30.0) / 900.0;
    EXPECT_NEAR(nf.Cn(Rad(5.0), 2.0), 0.2068 * 5.0 + b * 25.0, 1e-12);
    // Beyond alpha_max the coefficient is held.
    EXPECT_DOUBLE_EQ(nf.Cn(Rad(40.0), 2.0), nf.CnMax(2.0));
}

TEST(MissileNormalForce, ZeroAtZeroAlphaAndRisingToAlphaMax) {
    const MissileNormalForce& nf = Aim120NormalForce();
    for (double mach = 0.2; mach <= 4.5; mach += 0.05) {
        EXPECT_DOUBLE_EQ(nf.Cn(0.0, mach), 0.0);
        double previous = 0.0;
        for (double alphaDeg = 0.25; alphaDeg <= 30.0; alphaDeg += 0.25) {
            double cn = nf.Cn(Rad(alphaDeg), mach);
            EXPECT_GT(cn, previous) << "M" << mach << " alpha " << alphaDeg;
            previous = cn;
        }
    }
}

TEST(MissileNormalForce, AlphaForCnRoundTrips) {
    const MissileNormalForce& nf = Aim120NormalForce();
    for (double mach : {0.3, 0.6, 0.95, 1.3, 2.5, 4.0}) {
        for (double alphaDeg : {0.5, 3.0, 9.0, 17.0, 29.0}) {
            double cn = nf.Cn(Rad(alphaDeg), mach);
            EXPECT_NEAR(nf.AlphaForCn(cn, mach), Rad(alphaDeg), 1e-9);
        }
    }
    EXPECT_DOUBLE_EQ(nf.AlphaForCn(0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(nf.AlphaForCn(-1.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(nf.AlphaForCn(1.0e3, 1.0), nf.AlphaMaxRad());  // beyond CN_max: clamped
}

// CN_max exactly CN_alpha alpha_max makes b = 0: the curve is the slope's
// straight line and the inversion is CN / a.
TEST(MissileNormalForce, StraightLineWhenCnMaxIsOnTheSlope) {
    MissileNormalForce line({{0.5, 0.3}, {2.0, 0.2}}, {{0.5, 9.0}, {2.0, 6.0}}, 30.0, 0.25);
    EXPECT_NEAR(line.Cn(Rad(10.0), 1.25), 2.5, 1e-12);  // a = 0.25 at M1.25
    EXPECT_NEAR(line.AlphaForCn(2.5, 1.25), Rad(10.0), 1e-12);
    EXPECT_DOUBLE_EQ(line.ReferenceAreaFt2(), 0.25);
}
