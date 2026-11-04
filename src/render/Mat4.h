#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

#include "util/Vec3.h"

// Render-side matrix and vector math, single precision (OpenGL's float
// uniforms), shared by RenderSystem and Camera. Eigen's default storage is
// column-major, so data() feeds glUniformMatrix4fv with transpose = GL_FALSE.

using Mat4 = Eigen::Matrix4f;
using Vec3f = Eigen::Vector3f;

constexpr float kPiF = 3.14159265f;
constexpr float kDegToRadF = kPiF / 180.0f;

// Degenerate-geometry guards, in the units of the value tested.
constexpr float kSingularDeterminant = 1e-12f;
constexpr float kMinClipW = 1e-9f;      // keeps the perspective divide finite
constexpr float kParallelRayY = 1e-6f;  // ray direction y: parallel to a horizontal plane
constexpr float kBehindCameraW = 0.0001f;

inline Vec3f ToVec3f(const Vec3& v) { return v.cast<float>(); }

// gluPerspective's matrix.
inline Mat4 Perspective(float fovYRadians, float aspect, float nearZ, float farZ) {
    Mat4 m = Mat4::Zero();
    float f = 1.0f / std::tan(fovYRadians / 2.0f);
    m(0, 0) = f / aspect;
    m(1, 1) = f;
    m(2, 2) = (farZ + nearZ) / (nearZ - farZ);
    m(3, 2) = -1.0f;
    m(2, 3) = (2.0f * farZ * nearZ) / (nearZ - farZ);
    return m;
}

// gluLookAt's matrix.
inline Mat4 LookAt(const Vec3f& eye, const Vec3f& center, const Vec3f& up) {
    Vec3f f = (center - eye).normalized();
    Vec3f s = f.cross(up).normalized();
    Vec3f u = s.cross(f);

    Mat4 m = Mat4::Identity();
    m.block<1, 3>(0, 0) = s.transpose();
    m.block<1, 3>(1, 0) = u.transpose();
    m.block<1, 3>(2, 0) = -f.transpose();
    m(0, 3) = -s.dot(eye);
    m(1, 3) = -u.dot(eye);
    m(2, 3) = f.dot(eye);
    return m;
}

// Identity for a singular matrix: a degenerate frame is a bad frame to
// click in, not a crash.
inline Mat4 InverseOrIdentity(const Mat4& m) {
    if (std::fabs(m.determinant()) < kSingularDeterminant) return Mat4::Identity();
    return m.inverse();
}

// Unprojects a screen-space point (pixel coords, origin top-left -- see
// WorldToScreen) at NDC depth ndcZ (-1 = near plane, 1 = far plane) back to
// world space through invViewProj.
inline Vec3f UnprojectPoint(float screenX, float screenY, float ndcZ, const Mat4& invViewProj, int viewportWidth,
                            int viewportHeight) {
    float ndcX = (screenX / static_cast<float>(viewportWidth)) * 2.0f - 1.0f;
    float ndcY = 1.0f - (screenY / static_cast<float>(viewportHeight)) * 2.0f;
    Eigen::Vector4f clip = invViewProj * Eigen::Vector4f(ndcX, ndcY, ndcZ, 1.0f);
    if (std::fabs(clip.w()) < kMinClipW) clip.w() = kMinClipW;
    return clip.head<3>() / clip.w();
}

// The world-space ray under a screen pixel: from the near plane toward the
// far plane, `dir` unnormalized.
struct Ray {
    Vec3f origin;
    Vec3f dir;
};

inline Ray ScreenToWorldRay(float screenX, float screenY, const Mat4& viewProj, int viewportWidth, int viewportHeight) {
    Mat4 invViewProj = InverseOrIdentity(viewProj);
    Vec3f nearPt = UnprojectPoint(screenX, screenY, -1.0f, invViewProj, viewportWidth, viewportHeight);
    Vec3f farPt = UnprojectPoint(screenX, screenY, 1.0f, invViewProj, viewportWidth, viewportHeight);
    return Ray{nearPt, farPt - nearPt};
}

// Where `ray` crosses the horizontal plane at height planeY. False when
// the ray runs parallel to the plane or the plane is behind its origin.
inline bool RayHitPlaneY(const Ray& ray, float planeY, Vec3f& hit) {
    if (std::fabs(ray.dir.y()) <= kParallelRayY) return false;
    float t = (planeY - ray.origin.y()) / ray.dir.y();
    if (t <= 0.0f) return false;
    hit = ray.origin + ray.dir * t;
    return true;
}

// Projects a world-space point through viewProj to pixel coordinates
// (origin top-left, matching GLFW's cursor convention: NDC is Y-up, screen
// is Y-down). False for a point behind the camera (clip w near zero or
// negative), which a plain divide would put at a bogus on-screen position.
inline bool WorldToScreen(const Vec3f& worldPos, const Mat4& viewProj, int viewportWidth, int viewportHeight,
                          float& outX, float& outY) {
    Eigen::Vector4f clip = viewProj * worldPos.homogeneous();
    if (clip.w() <= kBehindCameraW) return false;

    float ndcX = clip.x() / clip.w();
    float ndcY = clip.y() / clip.w();
    outX = (ndcX * 0.5f + 0.5f) * static_cast<float>(viewportWidth);
    outY = (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(viewportHeight);
    return true;
}
