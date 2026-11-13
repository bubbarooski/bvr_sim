// Which events tick a trail, and whose: a radar event names the target but
// belongs on the radar owner's path, a launch on the launcher's, an RWR event
// on the aircraft that heard it, a seeker event on the missile's.

#include <gtest/gtest.h>

#include <string>

#include "components/GuidanceComponent.h"
#include "ecs/ComponentStore.h"
#include "render/EventMarks.h"

namespace {

constexpr EntityID kShooter = 1;
constexpr EntityID kTarget = 2;

SimEvent Event(const std::string& type, EntityID entity, const std::string& data = "") {
    return SimEvent{10.0, type, entity, data};
}

}  // namespace

TEST(EventMarks, RadarEventsTickTheRadarOwnersPath) {
    World world;
    std::string data = "owner=" + std::to_string(kShooter) + " target=" + std::to_string(kTarget);
    auto track = EventMarks::Classify(world, Event("RADAR_TRACK_ESTABLISHED", kTarget, data));
    ASSERT_TRUE(track.has_value());
    EXPECT_EQ(track->path, kShooter);
    EXPECT_EQ(track->tier, RadarTrackMode::TRACK);
    auto lock = EventMarks::Classify(world, Event("RADAR_LOCK", kTarget, data));
    ASSERT_TRUE(lock.has_value());
    EXPECT_EQ(lock->path, kShooter);
    EXPECT_EQ(lock->tier, RadarTrackMode::FIRING_SOLUTION);
}

TEST(EventMarks, ALaunchTicksTheLaunchersPathAsAMilestone) {
    World world;
    EntityID missile = world.CreateEntity();
    GuidanceComponent guidance;
    guidance.launcher = kShooter;
    world.Components<GuidanceComponent>().Add(missile, guidance);
    auto launch = EventMarks::Classify(world, Event("LAUNCH", missile, "target=2 gate=tws"));
    ASSERT_TRUE(launch.has_value());
    EXPECT_EQ(launch->path, kShooter);
    EXPECT_FALSE(launch->tier.has_value());
}

TEST(EventMarks, RwrAndSeekerEventsTickTheirOwnEntity) {
    World world;
    auto rwr = EventMarks::Classify(world, Event("RWR_LOCK", kTarget));
    ASSERT_TRUE(rwr.has_value());
    EXPECT_EQ(rwr->path, kTarget);
    EXPECT_EQ(rwr->tier, RadarTrackMode::FIRING_SOLUTION);
    auto coasting = EventMarks::Classify(world, Event("SEEKER_LOST", 7, "reason=gimbal"));
    ASSERT_TRUE(coasting.has_value());
    EXPECT_EQ(coasting->tier, RadarTrackMode::TRACK);
    auto onDatalink = EventMarks::Classify(world, Event("SEEKER_LOST", 7, "reason=gimbal fallback=datalink"));
    ASSERT_TRUE(onDatalink.has_value());
    EXPECT_EQ(onDatalink->tier, RadarTrackMode::SEARCH);
}

TEST(EventMarks, NoisyAndUnownedEventsLeaveNoTick) {
    World world;
    EXPECT_FALSE(EventMarks::Classify(world, Event("DETECTION", kTarget, "owner=1 target=2")).has_value());
    EXPECT_FALSE(EventMarks::Classify(world, Event("G_LIMIT_EXCEEDED", kShooter)).has_value());
    // A radar event with no owner in its data has no path to tick.
    EXPECT_FALSE(EventMarks::Classify(world, Event("RADAR_LOCK", kTarget)).has_value());
}
