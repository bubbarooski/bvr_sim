// Drawing and click-picking both build their frustum from
// Camera::ProjectionMatrix * ViewMatrix; these check that frustum puts the
// orbit target where the lens shift says and that a pick through it lands
// back on the point that was drawn.

#include <gtest/gtest.h>

#include "render/Camera.h"
#include "render/Mat4.h"

namespace {

constexpr int kWidth = 1600;
constexpr int kHeight = 900;
constexpr float kAspect = static_cast<float>(kWidth) / kHeight;

Camera GroundCamera() {
    Camera camera;
    camera.target = {1000.0f, 0.0f, -2000.0f};
    camera.azimuthDeg = -110.0f;
    camera.elevationDeg = 30.0f;
    camera.distance = 20000.0f;
    return camera;
}

}  // namespace

TEST(Camera, EyeSitsOnTheOrbitSphere) {
    Camera camera = GroundCamera();
    Vec3f offset = camera.EyePosition() - camera.target;
    EXPECT_NEAR(offset.norm(), camera.distance, 0.01f);
    EXPECT_NEAR(offset.y(), camera.distance * 0.5f, 0.01f);  // sin 30 deg

    camera.azimuthDeg = 0.0f;
    camera.elevationDeg = 0.0f;
    offset = camera.EyePosition() - camera.target;
    EXPECT_NEAR(offset.x(), camera.distance, 0.01f);
    EXPECT_NEAR(offset.z(), 0.0f, 0.01f);
}

TEST(Camera, TargetProjectsToTheLensShiftedCentre) {
    Camera camera = GroundCamera();
    for (float shift : {0.0f, -0.25f}) {
        Mat4 viewProj = Camera::ProjectionMatrix(kAspect, shift) * camera.ViewMatrix();
        float x = 0.0f, y = 0.0f;
        ASSERT_TRUE(WorldToScreen(camera.target, viewProj, kWidth, kHeight, x, y));
        EXPECT_NEAR(x, (shift * 0.5f + 0.5f) * kWidth, 0.5f) << "shift " << shift;
        EXPECT_NEAR(y, kHeight * 0.5f, 0.5f) << "shift " << shift;
    }
}

TEST(Camera, PickingAGroundPointRecoversIt) {
    Camera camera = GroundCamera();
    Mat4 viewProj = Camera::ProjectionMatrix(kAspect, -0.25f) * camera.ViewMatrix();
    for (Vec3f ground : {Vec3f{1000.0f, 0.0f, -2000.0f}, Vec3f{4000.0f, 0.0f, 3000.0f}, Vec3f{-5000.0f, 0.0f, -6000.0f}}) {
        float x = 0.0f, y = 0.0f;
        ASSERT_TRUE(WorldToScreen(ground, viewProj, kWidth, kHeight, x, y));
        Vec3f hit;
        ASSERT_TRUE(RayHitPlaneY(ScreenToWorldRay(x, y, viewProj, kWidth, kHeight), 0.0f, hit));
        // Feet. Float unprojection over the 50 ft..6e6 ft frustum leaves a few
        // ft at 20 000 ft range, a fraction of a pixel.
        EXPECT_LT((hit - ground).norm(), 25.0f) << ground.transpose();
    }
}

TEST(Camera, PointBehindTheEyeIsNotOnScreen) {
    Camera camera = GroundCamera();
    Mat4 viewProj = Camera::ProjectionMatrix(kAspect) * camera.ViewMatrix();
    Vec3f behind = camera.EyePosition() + (camera.EyePosition() - camera.target);
    float x = 0.0f, y = 0.0f;
    EXPECT_FALSE(WorldToScreen(behind, viewProj, kWidth, kHeight, x, y));
}

// The world is (X east, Y up, Z north) -- a left-handed triple -- and the
// projection is right-handed, so the view must flip one axis or the whole
// picture is mirrored. From due south, looking north, east is on the right.
TEST(Camera, EastIsOnTheRightWhenLookingNorth) {
    Camera camera = GroundCamera();
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.azimuthDeg = -90.0f;  // eye due south of the target
    camera.elevationDeg = 20.0f;
    Mat4 viewProj = Camera::ProjectionMatrix(kAspect) * camera.ViewMatrix();

    float targetX = 0.0f, targetY = 0.0f, eastX = 0.0f, eastY = 0.0f, northX = 0.0f, northY = 0.0f, upX = 0.0f, upY = 0.0f;
    ASSERT_TRUE(WorldToScreen(camera.target, viewProj, kWidth, kHeight, targetX, targetY));
    ASSERT_TRUE(WorldToScreen({3000.0f, 0.0f, 0.0f}, viewProj, kWidth, kHeight, eastX, eastY));
    ASSERT_TRUE(WorldToScreen({0.0f, 0.0f, 3000.0f}, viewProj, kWidth, kHeight, northX, northY));
    ASSERT_TRUE(WorldToScreen({0.0f, 3000.0f, 0.0f}, viewProj, kWidth, kHeight, upX, upY));
    EXPECT_GT(eastX, targetX + 50.0f) << "east should be to the right";
    EXPECT_NEAR(northX, targetX, 1.0f);
    EXPECT_LT(northY, targetY) << "north, away from the camera, rises toward the horizon (smaller y)";
    EXPECT_LT(upY, targetY) << "up is up on screen (smaller y)";
}

TEST(Camera, EastIsOnTheRightFromTheDefaultViewpoint) {
    // The default eye is south-south-west, looking north-north-east.
    Camera camera;
    Mat4 viewProj = Camera::ProjectionMatrix(kAspect) * camera.ViewMatrix();
    float targetX = 0.0f, targetY = 0.0f, eastX = 0.0f, eastY = 0.0f;
    ASSERT_TRUE(WorldToScreen(camera.target, viewProj, kWidth, kHeight, targetX, targetY));
    ASSERT_TRUE(WorldToScreen(camera.target + Vec3f{3000.0f, 0.0f, 0.0f}, viewProj, kWidth, kHeight, eastX, eastY));
    EXPECT_GT(eastX, targetX);
}

// Pan moves the target along the screen's own axes, so it must read them from
// the view that is drawn: from due south, screen-right is east.
TEST(Camera, ScreenAxesAreTheOnesTheViewDraws) {
    Camera camera = GroundCamera();
    camera.azimuthDeg = -90.0f;
    camera.elevationDeg = 30.0f;
    Vec3f right = camera.ScreenRight();
    Vec3f up = camera.ScreenUp();
    EXPECT_NEAR(right.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(right.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(right.z(), 0.0f, 1e-5f);
    EXPECT_GT(up.y(), 0.8f);  // mostly world-up, tilted toward north by the 30 degree elevation
    EXPECT_GT(up.z(), 0.4f);
    EXPECT_NEAR(right.dot(up), 0.0f, 1e-5f);
}
