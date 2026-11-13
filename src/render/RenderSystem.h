#pragma once

#include <optional>
#include <string>

#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "ecs/World.h"
#include "render/Camera.h"
#include "render/Mat4.h"
#include "render/ViewSettings.h"
#include "sim/SimContext.h"

// The 3D view: a marker per entity with a Transform, sensor volumes,
// routes, trails, and a procedural ground grid. Core-profile GL, so a
// minimal shader + VBOs.
namespace RenderSystem {

// An aircraft marker's on-screen length; a missile's is 3/4 of it. Exposed
// so 3D-view picking can size its click radius to match.
inline constexpr float kMarkerScreenLengthPx = 30.0f;

// The tier an aircraft's radar cone shows: the most advanced track its
// radar holds on any target (STT lock, confirmed, anything else = scanning).
// Exposed so the Entity Inspector reads the same. Defined in
// RadarConeTier.cpp, which needs no GL.
RadarTrackMode RadarConeTier(const SimContext& simContext, EntityID owner, const AirborneRadarComponent& radar);

// The ground grid's line spacing (ft) at a camera orbit distance, snapped to
// a 1/2/5 x 10^n ladder so it reads as a round number. Exposed so the
// sidebar's scale legend shows exactly what is drawn.
float GridSpacingFt(float cameraDistance);

void Init();
void Shutdown();
// Clears every entity's trail history and event ticks. Call after a reload,
// whose new entities reuse the old IDs.
void ResetTrails();

// The labels ("t=41.2 s  Radar lock"), one per line, of the event ticks
// drawn last frame within hover reach of a cursor at (screenX, screenY);
// empty if none. Same window units and viewProj as entity picking.
std::string HoveredEventMarks(const Mat4& viewProj, int viewportWidth, int viewportHeight, float screenX,
                              float screenY);
// Per-frame display choices.
struct RenderOptions {
    // Freezes trail aging, so trails don't fade away purely from real time
    // spent paused.
    bool paused = false;

    // Places event ticks between frames (s).
    double simTime = 0.0;

    // What to draw (the viewer's Settings window).
    ViewSettings view;

    // An index into the selected entity's waypoint queue: draws a vertical
    // altitude-drag gizmo at that entry.
    std::optional<size_t> selectedWaypointIndex;

    // Camera::ProjectionMatrix's ndcShiftX (see there).
    float viewNdcShiftX = 0.0f;
};

// simContext is only read (the radar cones' tiers come from its tracks). A
// zero-sized viewport (a minimised window) draws nothing.
void Draw(World& world, const SimContext& simContext, int viewportWidth, int viewportHeight, const Camera& camera,
          const RenderOptions& options);

}  // namespace RenderSystem
