#include "Camera.h"

#include <algorithm>
#include <cmath>

#include <GLFW/glfw3.h>

#include "imgui.h"

namespace {

constexpr float kOrbitSpeedDegPerPixel = 0.25f;
constexpr float kPanSpeedPerPixel = 0.0015f;  // scaled by distance in Update()
// Zoom factor per mouse-wheel notch: about 10 notches from the default
// ~20000 ft down to kMinDistance (1.6^10 ~ 110x).
constexpr float kZoomFactor = 1.6f;
constexpr float kMinDistance = 200.0f;
constexpr float kMaxDistance = 500000.0f;
// Floored just above the horizon: an eye below the target's height looks
// up from underground once the target is near the ground, with no ground
// plane to reorient against.
constexpr float kMinElevationDeg = 2.0f;
constexpr float kMaxElevationDeg = 89.0f;

constexpr float kFovYDeg = 45.0f;
// A 24-bit depth buffer resolves about d^2 / (near * 2^24): 24 ft at the
// default 20 000 ft orbit with a 1 ft near plane, enough for a cone's fill to
// bleed through the marker slab at its apex; 0.5 ft with this. The orbit
// never comes closer than kMinDistance to its target.
constexpr float kNearZ = 50.0f;
// Past kMaxDistance plus the ground grid's fade-out radius (RenderSystem's
// kFadeEndRatio, relative to camera distance), so zooming out never clips
// the scene.
constexpr float kFarZ = 6000000.0f;

}  // namespace

Mat4 Camera::ProjectionMatrix(float aspect, float ndcShiftX) {
    Mat4 m = Perspective(kFovYDeg * kDegToRadF, aspect, kNearZ, kFarZ);
    // m(0, 2) is the x-from-z shear glFrustum gives an asymmetric frustum: a
    // uniform NDC x offset with no depth skew, hence the negation. It widens
    // the frustum on one side rather than cropping, so the world under the
    // sidebar comes into view.
    m(0, 2) = -ndcShiftX;
    return m;
}

Vec3f Camera::EyePosition() const {
    float azimuthRad = azimuthDeg * kDegToRadF;
    float elevationRad = elevationDeg * kDegToRadF;
    Vec3f offset = {
        distance * std::cos(elevationRad) * std::cos(azimuthRad),
        distance * std::sin(elevationRad),
        distance * std::cos(elevationRad) * std::sin(azimuthRad),
    };
    return target + offset;
}

// The world is (X east, Y up, Z north), a left-handed triple, and LookAt and
// the projection are right-handed: fed the world as it is, they draw the
// whole scene mirrored (east on the left looking north). North is flipped on
// the way in, view = LookAt(M eye, M target, up) * M with M = diag(1, 1, -1),
// so what is drawn is the world as seen from where the eye is. The picking
// ray inverts this same matrix and agrees by construction.
Mat4 Camera::ViewMatrix() const {
    auto flipNorth = [](Vec3f v) { return Vec3f{v.x(), v.y(), -v.z()}; };
    Mat4 flip = Mat4::Identity();
    flip(2, 2) = -1.0f;
    return LookAt(flipNorth(EyePosition()), flipNorth(target), Vec3f::UnitY()) * flip;
}

Vec3f Camera::ScreenRight() const { return ViewMatrix().block<1, 3>(0, 0).transpose(); }
Vec3f Camera::ScreenUp() const { return ViewMatrix().block<1, 3>(1, 0).transpose(); }

void Camera::Update(GLFWwindow* window, float scrollDeltaY) {
    double mouseX = 0.0, mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    double dx = hasLastMouse_ ? mouseX - lastMouseX_ : 0.0;
    double dy = hasLastMouse_ ? mouseY - lastMouseY_ : 0.0;
    lastMouseX_ = mouseX;
    lastMouseY_ = mouseY;
    hasLastMouse_ = true;

    // Not while an ImGui widget has the mouse (valid: this runs after
    // ImGui::NewFrame()).
    bool imguiWantsMouse = ImGui::GetIO().WantCaptureMouse;

    if (!imguiWantsMouse && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        azimuthDeg -= static_cast<float>(dx) * kOrbitSpeedDegPerPixel;
        elevationDeg = std::clamp(elevationDeg + static_cast<float>(dy) * kOrbitSpeedDegPerPixel, kMinElevationDeg,
                                   kMaxElevationDeg);
    } else if (!imguiWantsMouse && !panLocked && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        Vec3f right = ScreenRight();
        Vec3f up = ScreenUp();

        float panScale = distance * kPanSpeedPerPixel;
        target -= right * (static_cast<float>(dx) * panScale);
        target += up * (static_cast<float>(dy) * panScale);
        // Keeps the target above ground; the elevation clamp alone cannot stop a
        // pan from taking the view underground.
        target.y() = std::max(target.y(), 0.0f);
    }

    if (!imguiWantsMouse) {
        distance = std::clamp(distance * std::pow(kZoomFactor, -scrollDeltaY), kMinDistance, kMaxDistance);
    }
}
