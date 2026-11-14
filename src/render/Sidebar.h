#pragma once

#include <optional>
#include <string>

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"

class TickRecorder;

// The app/session state the sidebar reads and writes each frame, as
// references into what main.cpp owns.
struct SidebarState {
    SimClock& simClock;
    ScenarioData& scenario;
    std::string& scenarioPath;
    double& simTime;
    std::string& groundImpactMessage;
    std::optional<EntityID>& followedEntity;
    // When on, a left click in the 3D view appends a waypoint to the selected
    // entity (handled in main.cpp) instead of selecting.
    bool& waypointPlacementMode;

    // The grabbed entry of the selected entity's waypoint queue, which
    // DrawWaypointEditPanel edits and its close button clears. Its screen
    // position comes from main.cpp, which has the projection in scope.
    std::optional<size_t>& selectedWaypointIndex;
    bool hasSelectedWaypointScreenPos = false;
    float selectedWaypointScreenX = 0.0f;
    float selectedWaypointScreenY = 0.0f;
    // When on, the camera follows the selected entity (main.cpp links it).
    bool& centerOnSelectedEntity;

    // For the grid scale legend (RenderSystem::GridSpacingFt).
    float cameraDistance;
    // The time panel's Record toggle opens and closes it; main.cpp samples
    // it after every tick. Files go in recordingDir.
    TickRecorder& recorder;
    const std::string& recordingDir;
};

// Fixed-width, right-docked control panel. Called once per frame, after
// the 3D scene draw, before ImGui::Render().
namespace Sidebar {

void Draw(World& world, SimContext& simContext, SidebarState& state);

// The sidebar's current (drag-resizable) width in ImGui display units, so
// main.cpp can centre the projection on the visible 3D view (see
// Camera::ProjectionMatrix). As of the last Draw(), so a splitter drag lags
// the view by one frame rather than drawing one frame at two widths.
float Width();

// The narrowest window that fits the sidebar at its minimum width beside
// the 3D view's time panel; main.cpp sets it as the window's size limit.
float MinWindowWidth();

}  // namespace Sidebar
