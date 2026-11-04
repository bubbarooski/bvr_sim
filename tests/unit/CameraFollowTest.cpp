// Which entity the camera follows when "Center Camera on Selection" is on.
// The follow starts when the selection changes or the option is switched on;
// a manual release (the F key, a drag, a double-click on empty space) then
// holds until the next change. Following the selection every frame instead
// re-followed the moment it was released.

#include <optional>

#include <gtest/gtest.h>

#include "render/CameraFollow.h"

namespace {

struct Camera {
    FollowSync sync;
    std::optional<EntityID> followed;
    void Frame(bool enabled, std::optional<EntityID> selection) { SyncFollow(enabled, selection, sync, followed); }
};

}  // namespace

TEST(CameraFollow, FollowsTheSelectionWhenItChanges) {
    Camera camera;
    camera.Frame(true, 2);
    EXPECT_EQ(camera.followed, 2u);
    camera.Frame(true, 3);
    EXPECT_EQ(camera.followed, 3u);
}

TEST(CameraFollow, AManualReleaseHoldsUntilTheSelectionChanges) {
    Camera camera;
    camera.Frame(true, 2);
    camera.followed.reset();  // the user releases it
    for (int frame = 0; frame < 5; ++frame) camera.Frame(true, 2);
    EXPECT_FALSE(camera.followed.has_value());
    camera.Frame(true, 3);  // a new selection is followed again
    EXPECT_EQ(camera.followed, 3u);
}

TEST(CameraFollow, SwitchingTheOptionOnFollowsTheCurrentSelection) {
    Camera camera;
    camera.Frame(false, 2);
    EXPECT_FALSE(camera.followed.has_value());
    camera.Frame(true, 2);
    EXPECT_EQ(camera.followed, 2u);
}

TEST(CameraFollow, LeavingAndReenteringTheModeFollowsAgain) {
    // Waypoint placement turns the follow off for as long as it lasts.
    Camera camera;
    camera.Frame(true, 2);
    camera.followed.reset();
    camera.Frame(false, 2);
    camera.Frame(true, 2);
    EXPECT_EQ(camera.followed, 2u);
}

TEST(CameraFollow, NothingSelectedMeansNothingFollowed) {
    Camera camera;
    camera.Frame(true, 2);
    camera.Frame(true, std::nullopt);
    EXPECT_FALSE(camera.followed.has_value());
    camera.Frame(true, 4);
    EXPECT_EQ(camera.followed, 4u);
}

TEST(CameraFollow, DoesNothingWhileDisabled) {
    Camera camera;
    camera.followed = 7;  // followed some other way (the F key)
    camera.Frame(false, 2);
    camera.Frame(false, 3);
    EXPECT_EQ(camera.followed, 7u);
}
