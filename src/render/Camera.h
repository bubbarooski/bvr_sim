#pragma once

#include "render/Mat4.h"

struct GLFWwindow;

// Orbit camera: eye position is derived from a target point plus spherical
// coordinates (azimuth/elevation/distance) around it. Owns its own input
// handling (pan/orbit/zoom) so main.cpp just calls Update() once per frame
// and hands the result to RenderSystem::Draw().
//
// Controls: left-drag pans the target across the view plane, right-drag
// orbits (adjusts azimuth/elevation), scroll zooms (adjusts distance).
class Camera {
public:
    Vec3f target = {0.0f, 1500.0f, 0.0f};
    float azimuthDeg = -110.0f;
    float elevationDeg = 20.0f;
    float distance = 20000.0f;

    // True while the target follows an entity: left-drag pan is ignored, as
    // the follow would overwrite it. Orbit and zoom still work.
    bool panLocked = false;

    Vec3f EyePosition() const;
    Mat4 ViewMatrix() const;

    // The world-space directions that point to the right and up on screen,
    // read from the view matrix's own rows so they hold whatever its
    // handedness. Left-drag pan moves the target along them.
    Vec3f ScreenRight() const;
    Vec3f ScreenUp() const;

    // The one projection: rendering and click-picking must use the same
    // frustum, so the FOV and clip planes live only here.
    //
    // ndcShiftX slides the projected image sideways by that many NDC units
    // (-1..1 spans the window) without moving the camera -- an off-axis "lens
    // shift". main.cpp passes -sidebarWidth/windowWidth so `target` sits in the
    // middle of the visible 3D view, and must pass the same value everywhere a
    // frame's frustum is built, or picking disagrees with what was drawn.
    static Mat4 ProjectionMatrix(float aspect, float ndcShiftX = 0.0f);

    // Applies this frame's mouse input. scrollDeltaY is the scroll accumulated
    // by main.cpp's callback (GLFW has no scroll polling). Drag and scroll are
    // ignored while ImGui wants the mouse, but the cursor is still tracked, so
    // a drag starting just off a panel does not jump.
    void Update(GLFWwindow* window, float scrollDeltaY);

private:
    double lastMouseX_ = 0.0;
    double lastMouseY_ = 0.0;
    bool hasLastMouse_ = false;
};
