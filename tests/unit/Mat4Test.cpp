#include <gtest/gtest.h>

#include "render/Mat4.h"

TEST(Mat4, InverseUndoesLookAt) {
    Mat4 view = LookAt(Vec3f{0.0f, 0.0f, 5.0f}, Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{0.0f, 1.0f, 0.0f});
    EXPECT_TRUE((view * InverseOrIdentity(view)).isIdentity(1e-5f));
}

TEST(Mat4, LookAtMovesEyeToOrigin) {
    Mat4 view = LookAt(Vec3f{0.0f, 0.0f, 5.0f}, Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{0.0f, 1.0f, 0.0f});
    Eigen::Vector4f eye = view * Eigen::Vector4f(0.0f, 0.0f, 5.0f, 1.0f);
    EXPECT_TRUE(eye.isApprox(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f)));
}

TEST(Mat4, InverseOfSingularMatrixReturnsIdentity) {
    EXPECT_EQ(InverseOrIdentity(Mat4::Zero()), Mat4::Identity());
}

TEST(ScreenProjection, UnprojectPointWithIdentityIsPlainNdcConversion) {
    Vec3f world = UnprojectPoint(800.0f, 0.0f, 0.5f, Mat4::Identity(), 800, 600);
    EXPECT_FLOAT_EQ(world.x(), 1.0f);
    EXPECT_FLOAT_EQ(world.y(), 1.0f);
    EXPECT_FLOAT_EQ(world.z(), 0.5f);
}

TEST(ScreenProjection, WorldToScreenWithIdentityIsPlainNdcConversion) {
    float x = 0.0f, y = 0.0f;
    ASSERT_TRUE(WorldToScreen(Vec3f{1.0f, 1.0f, 0.5f}, Mat4::Identity(), 800, 600, x, y));
    EXPECT_FLOAT_EQ(x, 800.0f);
    EXPECT_FLOAT_EQ(y, 0.0f);
}

TEST(ScreenProjection, PointBehindCameraFailsToProject) {
    Mat4 viewProj = Perspective(0.785398f, 800.0f / 600.0f, 1.0f, 100000.0f) * LookAt(Vec3f{0.0f, 0.0f, 5.0f}, Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{0.0f, 1.0f, 0.0f});
    float x = 0.0f, y = 0.0f;
    EXPECT_FALSE(WorldToScreen(Vec3f{0.0f, 0.0f, 10.0f}, viewProj, 800, 600, x, y));
}

TEST(ScreenProjection, PointAtCameraCenterProjectsNearViewportCenter) {
    Mat4 viewProj = Perspective(0.785398f, 800.0f / 600.0f, 1.0f, 100000.0f) * LookAt(Vec3f{0.0f, 0.0f, 5.0f}, Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{0.0f, 1.0f, 0.0f});
    float x = 0.0f, y = 0.0f;
    ASSERT_TRUE(WorldToScreen(Vec3f{0.0f, 0.0f, 0.0f}, viewProj, 800, 600, x, y));
    EXPECT_NEAR(x, 400.0f, 0.5f);
    EXPECT_NEAR(y, 300.0f, 0.5f);
}
