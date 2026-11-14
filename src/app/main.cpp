#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

#include "FGFDMExec.h"

// GLFW pulls in windows.h, whose min/max macros break std::min/max (as its
// ERROR macro would JSBSim's LogLevel::ERROR, hence FGFDMExec.h first).
// Must precede windows.h.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "components/JSBSimComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "components/WaypointComponent.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "render/Camera.h"
#include "render/CameraFollow.h"
#include "render/EntityInspector.h"
#include "render/RenderSystem.h"
#include "render/Sidebar.h"
#include "render/ViewSettings.h"
#include "render/style/LatticeStyle.h"
#include "scenario/ParamRegistry.h"
#include "scenario/ScenarioLoader.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "sim/TickRecorder.h"
#include "systems/LaunchSystem.h"
#include "systems/WaypointSystem.h"
#include "util/EntityLookup.h"
#include "util/Numeric.h"

namespace {

// Scroll for Camera's zoom: GLFW has only a callback. Registered before
// ImGui_ImplGlfw_InitForOpenGL so ImGui's callback chains to it; skipped
// while ImGui wants the mouse.
double gScrollDeltaY = 0.0;
void ScrollCallback(GLFWwindow*, double /*xoffset*/, double yoffset) {
    if (!ImGui::GetIO().WantCaptureMouse) {
        gScrollDeltaY += yoffset;
    }
}

// Presses since the view's click handlers last ran, per button. The
// handlers poll the button once per frame, so a press and release inside
// one frame (a short click or touchpad tap) would otherwise never be seen
// down. Indexed by GLFW_MOUSE_BUTTON_LEFT / _RIGHT.
bool gPressLatched[2] = {false, false};

// Latches presses, and tells ImGui where the cursor is on every press:
// after focus loss ImGui holds an unknown position until the cursor
// moves, so a click with no move first (into an unfocused window, as
// automated drivers send) would be dropped. Registered before ImGui's
// init, like the scroll callback.
void MouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/) {
    if (action != GLFW_PRESS) return;
    if (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT) gPressLatched[button] = true;
    double x = 0.0, y = 0.0;
    glfwGetCursorPos(window, &x, &y);
    ImGui::GetIO().AddMousePosEvent(static_cast<float>(x), static_cast<float>(y));
}

// Down this frame for the 3D view: held now, or pressed since the last
// frame even if already released (its release edge then lands next frame).
bool ViewButtonDown(GLFWwindow* window, int button) {
    return !ImGui::GetIO().WantCaptureMouse &&
           (glfwGetMouseButton(window, button) == GLFW_PRESS || gPressLatched[button]);
}

// Markers -- entities and waypoints alike -- are just points, not meshes,
// so picking is screen-space distance to each projected position rather
// than a ray/geometry test. A click selects the nearest marker within
// kPickRadiusPixels: a little past the tip of an aircraft icon.
constexpr float kPickRadiusPixels = RenderSystem::kMarkerScreenLengthPx * 0.6f;

// The nearest-so-far search both pickers share.
template <typename Key>
struct NearestPick {
    const Mat4& viewProj;
    int viewportWidth;
    int viewportHeight;
    float screenX;
    float screenY;
    std::optional<Key> best;
    float bestDistSq = kPickRadiusPixels * kPickRadiusPixels;

    NearestPick(const Mat4& viewProjIn, int width, int height, float x, float y)
        : viewProj(viewProjIn), viewportWidth(width), viewportHeight(height), screenX(x), screenY(y) {}

    void Consider(Key key, const Vec3& worldPos) {
        float screenPosX = 0.0f, screenPosY = 0.0f;
        if (!WorldToScreen(ToVec3f(worldPos), viewProj, viewportWidth, viewportHeight, screenPosX, screenPosY)) {
            return;
        }
        float dx = screenPosX - screenX;
        float dy = screenPosY - screenY;
        float distSq = dx * dx + dy * dy;
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            best = key;
        }
    }
};

// The entity whose marker is closest to (screenX, screenY) on screen.
std::optional<EntityID> PickEntity(World& world, const Mat4& viewProj, int viewportWidth, int viewportHeight,
                                    float screenX, float screenY) {
    NearestPick<EntityID> pick{viewProj, viewportWidth, viewportHeight, screenX, screenY};
    for (auto& [entity, transform] : world.Components<Transform>()) pick.Consider(entity, transform.pos);
    return pick.best;
}

// Tracks left-button press/release to distinguish a click (for picking)
// from a drag (for Camera's pan) by movement distance, and consecutive
// clicks close together in time/position to detect a double-click.
struct ClickTracker {
    bool wasLeftDown = false;
    double pressX = 0.0, pressY = 0.0;
    double lastClickTime = Numeric::kLongAgoS;
    double lastClickX = 0.0, lastClickY = 0.0;
};

constexpr double kClickMoveThresholdPixels = 5.0;
constexpr double kDoubleClickSeconds = 0.4;

// Click on an entity marker to select it -- the same unified selection a
// click in the inspector's list drives, so everything downstream reacts
// identically either way. This does not follow the camera on its own;
// that is gated by the "Center Camera on Selection" toggle. Double-click
// on empty space releases an active follow. A single click on empty space
// and a double-click on an entity are both deliberate no-ops.
void HandleEntityClick(GLFWwindow* window, ClickTracker& tracker, World& world, const Mat4& viewProj,
                        int viewportWidth, int viewportHeight, std::optional<EntityID>& followedEntity) {
    bool leftDown = ViewButtonDown(window, GLFW_MOUSE_BUTTON_LEFT);
    double mouseX = 0.0, mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if (leftDown && !tracker.wasLeftDown) {
        tracker.pressX = mouseX;
        tracker.pressY = mouseY;
    }

    // A drag past the click threshold releases the follow, so panning always
    // works in one gesture rather than silently doing nothing while a follow
    // holds.
    if (leftDown && tracker.wasLeftDown && followedEntity.has_value()) {
        double dragDist = std::hypot(mouseX - tracker.pressX, mouseY - tracker.pressY);
        if (dragDist >= kClickMoveThresholdPixels) followedEntity.reset();
    }

    if (!leftDown && tracker.wasLeftDown) {
        double moveDist = std::hypot(mouseX - tracker.pressX, mouseY - tracker.pressY);
        if (moveDist < kClickMoveThresholdPixels) {
            double now = glfwGetTime();
            bool isDoubleClick = (now - tracker.lastClickTime) < kDoubleClickSeconds &&
                                  std::hypot(mouseX - tracker.lastClickX, mouseY - tracker.lastClickY) <
                                      kClickMoveThresholdPixels;

            std::optional<EntityID> picked = PickEntity(world, viewProj, viewportWidth, viewportHeight,
                                                          static_cast<float>(mouseX), static_cast<float>(mouseY));

            if (isDoubleClick && !picked) {
                followedEntity.reset();
            } else if (!isDoubleClick && picked) {
                // Clicking the selected entity again deselects, as the inspector list does.
                if (EntityInspector::SelectedEntity() == picked) {
                    EntityInspector::ClearSelection();
                } else {
                    EntityInspector::SetSelectedEntity(*picked);
                }
            }

            tracker.lastClickTime = now;
            tracker.lastClickX = mouseX;
            tracker.lastClickY = mouseY;
        }
    }

    tracker.wasLeftDown = leftDown;
}

// Placement mode's own tracker: left button select-or-place then optional
// drag, right button delete; no double-click or follow.
struct PlacementClickTracker {
    bool wasLeftDown = false;
    double pressX = 0.0, pressY = 0.0;
    // The press started on an existing marker: a drag moves it, not a
    // placement.
    bool draggingAltitude = false;
    double lastDragY = 0.0;

    bool wasRightDown = false;
    double rightPressX = 0.0, rightPressY = 0.0;
};

// The queue entry closest to (screenX, screenY) on screen.
std::optional<size_t> PickWaypoint(const WaypointComponent& waypoint, const Mat4& viewProj, int viewportWidth,
                                    int viewportHeight, float screenX, float screenY) {
    NearestPick<size_t> pick{viewProj, viewportWidth, viewportHeight, screenX, screenY};
    for (size_t i = 0; i < waypoint.queue.size(); ++i) pick.Consider(i, waypoint.queue[i].pos);
    return pick.best;
}

// Altitude per pixel of vertical drag: a deliberate 150-300 px drag covers
// a few thousand feet. A flat rate, not zoom-scaled, so the gesture stays
// predictable.
constexpr double kAltitudeFtPerPixel = 10.0;

// Left button in placement mode: press on an existing marker grabs it for
// an altitude drag; a clean release on a different entity's marker
// switches selection to it; a clean release elsewhere places a new point
// (or clears the waypoint selection, if one was active). No entity
// selected is a no-op throughout.
void HandleWaypointPlacement(GLFWwindow* window, PlacementClickTracker& tracker, World& world, const Mat4& viewProj,
                              int viewportWidth, int viewportHeight, std::optional<size_t>& selectedWaypointIndex) {
    bool leftDown = ViewButtonDown(window, GLFW_MOUSE_BUTTON_LEFT);
    bool rightDown = ViewButtonDown(window, GLFW_MOUSE_BUTTON_RIGHT);
    double mouseX = 0.0, mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    ComponentStore<WaypointComponent>& waypointStore = world.Components<WaypointComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    WaypointComponent* waypoint = selected.has_value() ? waypointStore.TryGet(*selected) : nullptr;

    if (leftDown && !tracker.wasLeftDown) {
        tracker.pressX = mouseX;
        tracker.pressY = mouseY;
        tracker.lastDragY = mouseY;
        std::optional<size_t> picked =
            waypoint ? PickWaypoint(*waypoint, viewProj, viewportWidth, viewportHeight, static_cast<float>(mouseX),
                                     static_cast<float>(mouseY))
                     : std::nullopt;
        tracker.draggingAltitude = picked.has_value();
        if (picked.has_value()) selectedWaypointIndex = picked;
    }

    if (leftDown && tracker.draggingAltitude && waypoint && selectedWaypointIndex.has_value() &&
        *selectedWaypointIndex < waypoint->queue.size()) {
        // Screen Y grows downward, hence the subtraction. Held to the
        // Placement altitude range, as the waypoint editor is.
        double deltaYPixels = tracker.lastDragY - mouseY;
        double& altitude = waypoint->queue[*selectedWaypointIndex].pos.y();
        const ParamRegistry::ParamDef* altitudeParam = ParamRegistry::Find("/entities/{}/transform/pos/1");
        altitude = std::clamp(altitude + deltaYPixels * kAltitudeFtPerPixel, altitudeParam->minValue,
                              altitudeParam->maxValue);
        tracker.lastDragY = mouseY;
    }

    if (!leftDown && tracker.wasLeftDown) {
        double moveDist = std::hypot(mouseX - tracker.pressX, mouseY - tracker.pressY);
        if (moveDist < kClickMoveThresholdPixels && !tracker.draggingAltitude) {
            std::optional<EntityID> pickedEntity = PickEntity(
                world, viewProj, viewportWidth, viewportHeight, static_cast<float>(mouseX), static_cast<float>(mouseY));
            if (pickedEntity.has_value() && pickedEntity != selected) {
                // Switch selection instead of placing a point there.
                EntityInspector::SetSelectedEntity(*pickedEntity);
                selectedWaypointIndex.reset();
            } else if (selectedWaypointIndex.has_value()) {
                // A plain click while a point is selected dismisses the selection rather
                // than also placing.
                selectedWaypointIndex.reset();
            } else if (const Transform* selectedTransform = selected.has_value() ? transforms.TryGet(*selected)
                                                                                 : nullptr) {
                // Inherit the last queued point's altitude (the entity's own for an empty
                // queue), so a route stays level unless deliberately dragged; the
                // aircraft's live altitude would staircase a climbing route.
                float planeY = static_cast<float>(selectedTransform->pos.y());
                if (waypoint != nullptr && !waypoint->queue.empty()) {
                    planeY = static_cast<float>(waypoint->queue.back().pos.y());
                }

                // No sensible hit (looking level, or the plane behind the camera): no-op.
                Ray ray = ScreenToWorldRay(static_cast<float>(mouseX), static_cast<float>(mouseY), viewProj,
                                           viewportWidth, viewportHeight);
                Vec3f hit;
                if (RayHitPlaneY(ray, planeY, hit)) {
                    // No speed override on a click-placed point (WaypointComponent.h).
                    WaypointComponent& route =
                        waypoint != nullptr ? *waypoint : waypointStore.Add(*selected, WaypointComponent{});
                    WaypointSystem::AppendPoint(route, WaypointPoint{Vec3{hit.x(), planeY, hit.z()}, std::nullopt});
                }
            }
        }
        tracker.draggingAltitude = false;
    }

    // Right click on a waypoint marker deletes it and every point after it.
    if (rightDown && !tracker.wasRightDown) {
        tracker.rightPressX = mouseX;
        tracker.rightPressY = mouseY;
    }
    if (!rightDown && tracker.wasRightDown) {
        double moveDist = std::hypot(mouseX - tracker.rightPressX, mouseY - tracker.rightPressY);
        if (moveDist < kClickMoveThresholdPixels && waypoint) {
            std::optional<size_t> picked = PickWaypoint(*waypoint, viewProj, viewportWidth, viewportHeight,
                                                          static_cast<float>(mouseX), static_cast<float>(mouseY));
            if (picked.has_value()) {
                WaypointSystem::TruncateQueue(*waypoint, *picked);
                if (selectedWaypointIndex.has_value() && *selectedWaypointIndex >= *picked) {
                    selectedWaypointIndex.reset();
                }
            }
        }
    }

    tracker.wasLeftDown = leftDown;
    tracker.wasRightDown = rightDown;
}

// Whether any aircraft has reached the ground. A missile at y=0 is an
// impact or miss, not a crash.
std::optional<EntityID> FindGroundImpact(World& world) {
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    for (auto& [entity, component] : world.Components<JSBSimComponent>()) {
        (void)component;
        const Transform* transform = transforms.TryGet(entity);
        if (transform != nullptr && transform->pos.y() <= 0.0) return entity;
    }
    return std::nullopt;
}

// The viewer's own display and editing state -- nothing here is simulated
// or saved. Sidebar edits it through SidebarState's references.
struct UiState {
    std::optional<EntityID> followedEntity;
    std::string groundImpactMessage;  // non-empty once a plane has crashed

    // Off by default: a 3D-view click selects.
    bool waypointPlacementMode = false;

    // The grabbed point within the selected entity's queue, for altitude
    // editing.
    std::optional<size_t> selectedWaypointIndex;

    bool centerOnSelectedEntity = false;
    FollowSync followSync;  // SyncSelection's memory of what it last followed
};

// Mouse state carried between frames. The keyboard shortcuts are the
// Sidebar's.
struct InputState {
    ClickTracker click;
    PlacementClickTracker placement;
    std::optional<EntityID> lastSelection;
};

// Pauses the sim the moment an aircraft reaches the ground -- the same pause
// as the checkbox. One-shot, so the user can resume to watch it settle.
void StopOnGroundImpact(World& world, SimClock& simClock, std::string& groundImpactMessage) {
    if (!groundImpactMessage.empty() || simClock.Paused()) return;
    std::optional<EntityID> crashed = FindGroundImpact(world);
    if (!crashed.has_value()) return;
    const Tag* tag = world.Components<Tag>().TryGet(*crashed);
    std::string name = tag != nullptr ? tag->name : "Entity " + std::to_string(*crashed);
    groundImpactMessage = name + " hit the ground -- simulation stopped.";
    simClock.Pause();
}

// Keeps camera follow and waypoint editing consistent with the current
// selection and mode.
void SyncSelection(UiState& ui, InputState& input) {
    // While on, the camera follows the selection, from the list or a 3D-view
    // click -- the one place a selection becomes a follow. Reassigned only on a
    // selection change, so a manual release holds until the selection changes.
    std::optional<EntityID> currentSelection = EntityInspector::SelectedEntity();
    SyncFollow(ui.centerOnSelectedEntity && !ui.waypointPlacementMode, currentSelection, ui.followSync,
               ui.followedEntity);

    // A selection change must not leave a gizmo on the previous entity's
    // queue.
    if (currentSelection != input.lastSelection) {
        ui.selectedWaypointIndex.reset();
        input.lastSelection = currentSelection;
    }

    // No grabbed point outside placement mode.
    if (!ui.waypointPlacementMode) ui.selectedWaypointIndex.reset();

    // Waypoint editing never moves the camera: selecting another aircraft
    // to route, or a Reload recentering on the Target, would otherwise
    // yank the view away from the points being placed.
    if (ui.waypointPlacementMode) ui.followedEntity.reset();
}

// While following, the camera target snaps to the entity every frame and
// left-drag does not pan (Camera::panLocked); likewise while a waypoint is
// grabbed, where left-drag moves the point. A vanished entity releases
// the follow.
void UpdateCamera(GLFWwindow* window, Camera& camera, World& world, UiState& ui, float scrollDeltaY) {
    if (ui.followedEntity.has_value()) {
        const Transform* followed = world.Components<Transform>().TryGet(*ui.followedEntity);
        if (followed != nullptr) {
            camera.target = ToVec3f(followed->pos);
        } else {
            ui.followedEntity.reset();
        }
    }
    camera.panLocked = ui.followedEntity.has_value() || ui.selectedWaypointIndex.has_value();
    camera.Update(window, scrollDeltaY);
}

// Lens shift, so the followed entity sits in the middle of the visible 3D
// view, not the window (Camera::ProjectionMatrix). A fraction of the window
// width, which is NDC. Must be identical everywhere a frustum is built this
// frame -- rendering, picking, panel anchoring.
float ViewNdcShiftX() {
    float displayWidth = ImGui::GetIO().DisplaySize.x;
    return displayWidth > 0.0f ? -Sidebar::Width() / displayWidth : 0.0f;
}

// The tooltip of the event ticks under the cursor, with the projection
// picking uses.
void ShowEventMarkTooltip(GLFWwindow* window, const Camera& camera, float viewNdcShiftX) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    int viewportW = 0, viewportH = 0;
    glfwGetWindowSize(window, &viewportW, &viewportH);
    if (viewportW <= 0 || viewportH <= 0) return;
    Mat4 viewProj = Camera::ProjectionMatrix(static_cast<float>(viewportW) / static_cast<float>(viewportH),
                                             viewNdcShiftX) *
                    camera.ViewMatrix();
    double mouseX = 0.0, mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);
    std::string labels = RenderSystem::HoveredEventMarks(viewProj, viewportW, viewportH, static_cast<float>(mouseX),
                                                         static_cast<float>(mouseY));
    if (!labels.empty()) ImGui::SetTooltip("%s", labels.c_str());
}

// Where the grabbed waypoint sits on screen, for anchoring its edit panel.
struct WaypointScreenAnchor {
    bool valid = false;
    float x = 0.0f;
    float y = 0.0f;
};

// Routes this frame's clicks in the 3D view, and locates the grabbed
// waypoint on screen with the same projection.
WaypointScreenAnchor HandleViewClicks(GLFWwindow* window, InputState& input, World& world, const Camera& camera,
                                      float viewNdcShiftX, UiState& ui) {
    // Window units, not framebuffer pixels: the cursor and ImGui use them, and
    // the projection depends only on the aspect ratio.
    int viewportW, viewportH;
    glfwGetWindowSize(window, &viewportW, &viewportH);
    float aspect = viewportH > 0 ? static_cast<float>(viewportW) / static_cast<float>(viewportH) : 1.0f;
    Mat4 viewProj = Camera::ProjectionMatrix(aspect, viewNdcShiftX) * camera.ViewMatrix();

    // Placement mode replaces normal clicks: one left click, one meaning.
    if (ui.waypointPlacementMode) {
        HandleWaypointPlacement(window, input.placement, world, viewProj, viewportW, viewportH,
                                ui.selectedWaypointIndex);
    } else {
        HandleEntityClick(window, input.click, world, viewProj, viewportW, viewportH, ui.followedEntity);
    }
    gPressLatched[GLFW_MOUSE_BUTTON_LEFT] = gPressLatched[GLFW_MOUSE_BUTTON_RIGHT] = false;

    WaypointScreenAnchor anchor;
    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    if (!ui.selectedWaypointIndex.has_value() || !selected.has_value()) return anchor;
    const WaypointComponent* waypoint = world.Components<WaypointComponent>().TryGet(*selected);
    if (waypoint == nullptr || *ui.selectedWaypointIndex >= waypoint->queue.size()) return anchor;
    anchor.valid = WorldToScreen(ToVec3f(waypoint->queue[*ui.selectedWaypointIndex].pos), viewProj, viewportW,
                                 viewportH, anchor.x, anchor.y);
    return anchor;
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenarioPath = argc > 1 ? argv[1] : "scenarios/1_core_01_first_shot.json";

    std::printf("JSBSim version: %s\n", JSBSim::FGJSBBase::GetVersion().c_str());

    World world;
    ScenarioData scenario;
    try {
        scenario = ScenarioLoader::Load(scenarioPath, world);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load scenario: %s\n", e.what());
        return 1;
    }

    // Pre-creates the missile so it is inspectable before it fires (as a
    // reload does).
    LaunchSystem::InitFromTags(world, scenario);

    SimContext simContext;
    SimLoop::BeginRun(scenario, simContext, scenario.seed);
    std::printf("Scenario loaded: seed=%llu sim_speed=%.2f\n",
                static_cast<unsigned long long>(scenario.seed), scenario.simSpeed);

    SimClock simClock;
    // Starts paused, to look before anything moves; the first unpause runs at
    // the scenario's speed.
    simClock.StartPaused(scenario.simSpeed);

    double simTime = 0.0;

    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    constexpr int kStepBudgetMs = 50;  // longest a frame may spend stepping the sim
    constexpr int kGlVersionMajor = 3;  // core profile 3.3: #version 330 shaders
    constexpr int kGlVersionMinor = 3;
    constexpr int kMsaaSamples = 4;
    constexpr int kInitialWindowWidth = 1280;
    constexpr int kInitialWindowHeight = 720;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, kGlVersionMajor);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, kGlVersionMinor);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    // macOS only grants a 3.2+ core context to forward-compatible requests.
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    // A multisampled framebuffer, for RenderSystem's GL_MULTISAMPLE.
    glfwWindowHint(GLFW_SAMPLES, kMsaaSamples);

    GLFWwindow* window = glfwCreateWindow(kInitialWindowWidth, kInitialWindowHeight, "bvr_sim", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }
    // GLFW's Win32 backend enforces a minimum only when both dimensions are
    // set, so the height needs a real value too: room for the time panel
    // above the view's bottom edge and the sidebar's section headers.
    constexpr int kMinWindowHeight = 480;
    glfwSetWindowSizeLimits(window, static_cast<int>(Sidebar::MinWindowWidth()), kMinWindowHeight, GLFW_DONT_CARE,
                            GLFW_DONT_CARE);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (gl3wInit() != 0) {
        std::fprintf(stderr, "Failed to initialize gl3w\n");
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // The window layout is saved beside the executable rather than in the
    // launch directory, so running from the repo root leaves no file there.
    // ImGui keeps the pointer, so the string lives until DestroyContext.
    const std::string iniPath = (std::filesystem::absolute(argv[0]).parent_path() / "imgui.ini").string();
    ImGui::GetIO().IniFilename = iniPath.c_str();
    ViewSettingsStore::RegisterIniHandler();
    // Recordings sit beside the executable too, out of the repo tree.
    const std::string recordingDir = (std::filesystem::absolute(argv[0]).parent_path() / "recordings").string();
    TickRecorder recorder;
    // Tab selects the next aircraft (Sidebar's shortcuts), not the next
    // text field.
    ImGui::GetCurrentContext()->ConfigNavEnableTabbing = false;
    LatticeStyle::Apply();

    // Before ImGui_ImplGlfw_InitForOpenGL: GLFW keeps one callback per event,
    // so ImGui must find these to chain to, or it replaces them.
    glfwSetScrollCallback(window, ScrollCallback);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    RenderSystem::Init();

    Camera camera;
    UiState ui;
    InputState input;

    // Open on the engagement, as Reset does, not on an empty grid: select
    // and follow the Target (selection too, else the selection sync below
    // drops the follow on the first frame).
    if (std::optional<EntityID> target = EntityLookup::FindByTagName(world, "Target")) {
        EntityInspector::SetSelectedEntity(*target);
        ui.followedEntity = target;
    }

    while (!glfwWindowShouldClose(window)) {
        // A minimised window has nothing to draw: wait for events (the sim
        // still steps on its clock) instead of spinning a core on empty frames.
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) == GLFW_TRUE) {
            glfwWaitEventsTimeout(1.0 / 60.0);
        } else {
            glfwPollEvents();
        }

        // Fixed-timestep catch-up: results do not depend on frame rate.
        simClock.Tick();
        // Stepping gets a wall-clock budget per frame. When it cannot keep up
        // with the requested speed (a Debug build at 100x), the backlog is
        // dropped: the run goes slower than asked, rather than one frame
        // stepping for seconds while the window answers nothing.
        const std::chrono::steady_clock::time_point stepDeadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(kStepBudgetMs);
        while (simClock.ShouldStep()) {
            if (std::chrono::steady_clock::now() >= stepDeadline) {
                simClock.DropBacklog();
                break;
            }
            SimLoop::Step(world, simContext, scenario, simClock.ConsumeStep(), simTime);
            recorder.Sample(world, simContext, simTime);
        }
        StopOnGroundImpact(world, simClock, ui.groundImpactMessage);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // After ImGui::NewFrame(): WantCaptureMouse is valid only then.
        float scrollDeltaY = static_cast<float>(gScrollDeltaY);
        gScrollDeltaY = 0.0;

        // Placement is paused-only: while running, a click would route an aircraft
        // that has already flown past it. Forced off here, before click handling,
        // since several controls can start the sim.
        if (!simClock.Paused()) ui.waypointPlacementMode = false;

        SyncSelection(ui, input);
        UpdateCamera(window, camera, world, ui, scrollDeltaY);
        float viewNdcShiftX = ViewNdcShiftX();
        WaypointScreenAnchor anchor = HandleViewClicks(window, input, world, camera, viewNdcShiftX, ui);

        SidebarState sidebarState{simClock,
                                   scenario,
                                   scenarioPath,
                                   simTime,
                                   ui.groundImpactMessage,
                                   ui.followedEntity,
                                   ui.waypointPlacementMode,
                                   ui.selectedWaypointIndex,
                                   anchor.valid,
                                   anchor.x,
                                   anchor.y,
                                   ui.centerOnSelectedEntity,
                                   camera.distance,
                                   recorder,
                                   recordingDir};
        Sidebar::Draw(world, simContext, sidebarState);
        ShowEventMarkTooltip(window, camera, viewNdcShiftX);

        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(LatticeStyle::kBackground.x, LatticeStyle::kBackground.y,
                     LatticeStyle::kBackground.z, LatticeStyle::kBackground.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        RenderSystem::RenderOptions renderOptions;
        renderOptions.paused = simClock.Paused();
        renderOptions.simTime = simTime;
        renderOptions.view = ViewSettingsStore::Get();
        renderOptions.selectedWaypointIndex = ui.selectedWaypointIndex;
        renderOptions.viewNdcShiftX = viewNdcShiftX;
        RenderSystem::Draw(world, simContext, display_w, display_h, camera, renderOptions);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    simContext.eventLog.PrintAll();

    RenderSystem::Shutdown();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
