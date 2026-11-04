// The scenario wind's direction convention and the wind triangle the
// waypoint follower steers by. The triangle is checked by what it must
// achieve -- air velocity plus wind lies along the track, at the stated
// airspeed -- rather than against its own formula.

#include <cmath>

#include <gtest/gtest.h>

#include "sim/Environment.h"
#include "sim/WindTriangle.h"
#include "util/Units.h"

namespace {

using Units::kDegToRad;
using Units::kFpsPerKt;

Environment Wind(double speedKts, double fromDeg) {
    Environment environment;
    environment.windSpeedKts = speedKts;
    environment.windFromDeg = fromDeg;
    return environment;
}

}  // namespace

TEST(Wind, BlowsFromTheStatedDirection) {
    // A westerly moves air east; a northerly moves it south.
    Vec3 westerly = WindFps(Wind(10.0, 270.0));
    EXPECT_NEAR(westerly.x(), 10.0 * kFpsPerKt, 1e-9);
    EXPECT_NEAR(westerly.z(), 0.0, 1e-9);
    Vec3 northerly = WindFps(Wind(10.0, 0.0));
    EXPECT_NEAR(northerly.x(), 0.0, 1e-9);
    EXPECT_NEAR(northerly.z(), -10.0 * kFpsPerKt, 1e-9);
    EXPECT_EQ(northerly.y(), 0.0);
}

TEST(Wind, CalmIsExactlyZero) {
    Vec3 calm = WindFps(Wind(0.0, 123.0));
    EXPECT_EQ(calm.x(), 0.0);
    EXPECT_EQ(calm.y(), 0.0);
    EXPECT_EQ(calm.z(), 0.0);
    EXPECT_FALSE(std::signbit(calm.x()));
    EXPECT_FALSE(std::signbit(calm.z()));
    // And the triangle hands the track straight back.
    EXPECT_EQ(WindTriangle::HeadingForTrackDeg(37.123, 700.0, calm), 37.123);
}

TEST(WindTriangle, HeadAndTailWindLeaveTheHeadingAlone) {
    EXPECT_NEAR(WindTriangle::HeadingForTrackDeg(90.0, 700.0, WindFps(Wind(80.0, 90.0))), 90.0, 1e-9);
    EXPECT_NEAR(WindTriangle::HeadingForTrackDeg(90.0, 700.0, WindFps(Wind(80.0, 270.0))), 90.0, 1e-9);
}

TEST(WindTriangle, CrabsIntoACrosswindByTheClassicAngle) {
    // Track north, wind from the east: crab right by asin(w / V).
    constexpr double kV = 700.0;
    Vec3 easterly = WindFps(Wind(100.0, 90.0));
    double expected = std::asin(100.0 * kFpsPerKt / kV) / kDegToRad;
    EXPECT_NEAR(WindTriangle::HeadingForTrackDeg(0.0, kV, easterly), expected, 1e-9);
    // Wind from the west on the same track: crab left, wrapped to [0, 360).
    Vec3 westerly = WindFps(Wind(100.0, 270.0));
    EXPECT_NEAR(WindTriangle::HeadingForTrackDeg(0.0, kV, westerly), 360.0 - expected, 1e-9);
}

TEST(WindTriangle, GroundVelocityLiesAlongTheTrack) {
    constexpr double kV = 650.0;
    const double tracks[] = {0.0, 45.0, 133.0, 222.5, 359.0};
    const double froms[] = {10.0, 100.0, 200.0, 300.0};
    for (double track : tracks) {
        for (double from : froms) {
            Vec3 wind = WindFps(Wind(120.0, from));
            double heading = WindTriangle::HeadingForTrackDeg(track, kV, wind) * kDegToRad;
            double groundEast = kV * std::sin(heading) + wind.x();
            double groundNorth = kV * std::cos(heading) + wind.z();
            double groundTrack = std::atan2(groundEast, groundNorth) / kDegToRad;
            double error = std::remainder(groundTrack - track, 360.0);
            EXPECT_NEAR(error, 0.0, 1e-9) << "track " << track << " wind from " << from;
        }
    }
}

TEST(WindTriangle, PointsIntoACrosswindItCannotBeat) {
    // 500 kt of wind from the east against a 400 ft/s aircraft tracking north.
    Vec3 gale = WindFps(Wind(500.0, 90.0));
    EXPECT_NEAR(WindTriangle::HeadingForTrackDeg(0.0, 400.0, gale), 90.0, 1e-9);
}
