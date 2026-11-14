#include "Sidebar.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "imgui.h"

#include "components/AutopilotComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "components/WaypointComponent.h"
#include "ecs/ComponentStore.h"
#include "render/EntityInspector.h"
#include "render/Mat4.h"
#include "render/ParameterPanel.h"
#include "render/RenderSystem.h"
#include "render/ViewSettings.h"
#include "render/style/LatticeStyle.h"
#include "scenario/ScenarioLoader.h"
#include "scenario/ParamRegistry.h"
#include "scenario/ScenarioSaver.h"
#include "sim/EventLog.h"
#include "sim/SimLoop.h"
#include "sim/TickRecorder.h"
#include "systems/LaunchSystem.h"
#include "systems/WaypointSystem.h"
#include "util/EntityLookup.h"
#include "util/EntityVelocity.h"
#include "util/Numeric.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace {

using Units::kFeetPerMeter;

// Where the New Scenario template lives and Save As writes, apart from the
// teaching scenarios. The template's leading underscore sorts it first in
// the File dropdown.
constexpr const char* kCustomScenarioDir = "scenarios/custom";
constexpr const char* kNewScenarioTemplatePath = "scenarios/custom/_template.json";

constexpr float kSidebarDefaultWidth = 380.0f;
constexpr float kSaveAsFieldWidth = 160.0f;
constexpr float kEntityComboWidth = 180.0f;
constexpr float kEventLogHeight = 160.0f;
constexpr float kSidebarMinWidth = 260.0f;
constexpr float kSidebarMaxWidth = 700.0f;

// The width the splitter last set; gSidebarWidth is it after the window
// has had its say (Sidebar::Draw).
float gSidebarPreferredWidth = kSidebarDefaultWidth;
// Read by the HUDs too, so they stay aligned to the visible 3D view.
float gSidebarWidth = kSidebarDefaultWidth;

// CollapsingHeader plus the 2px accent underline that marks every section.
bool SectionHeader(const char* label, bool defaultOpen) {
    bool open = ImGui::CollapsingHeader(label, defaultOpen ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None);
    ImVec2 min = ImGui::GetItemRectMin();
    ImVec2 max = ImGui::GetItemRectMax();
    ImGui::GetWindowDrawList()->AddLine(ImVec2(min.x, max.y), ImVec2(max.x, max.y), ImGui::GetColorU32(LatticeStyle::kAccentPrimary), 2.0f);
    return open;
}

// Transient status text (the Reload/Reset toast, the Save As result) holds
// for kTransientDurationSec, fading out over the last kTransientFadeSec.
// Wall-clock, not sim time -- these fire while the sim is paused.
constexpr double kTransientDurationSec = 5.0;
constexpr double kTransientFadeSec = 0.6;  // tail of kTransientDurationSec, not extra time

// Opacity for a message shown at shownAtSec; 0 once it has expired.
float TransientAlpha(double shownAtSec) {
    double remainingSec = kTransientDurationSec - (ImGui::GetTime() - shownAtSec);
    if (remainingSec <= 0.0) return 0.0f;
    return remainingSec < kTransientFadeSec ? static_cast<float>(remainingSec / kTransientFadeSec) : 1.0f;
}

// Ephemeral top-right confirmation for Reload/Reset/New Scenario.
std::string gToastMessage;
double gToastShownAtSec = Numeric::kLongAgoS;
constexpr float kToastMargin = 20.0f;

void ShowToast(const char* message) {
    gToastMessage = message;
    gToastShownAtSec = ImGui::GetTime();
}

void DrawToast() {
    if (gToastMessage.empty() || !ViewSettingsStore::Get().showToasts) return;
    float alpha = TransientAlpha(gToastShownAtSec);
    if (alpha <= 0.0f) {
        gToastMessage.clear();
        return;
    }

    ImGuiIO& io = ImGui::GetIO();
    float viewWidth = io.DisplaySize.x - gSidebarWidth;
    ImGui::SetNextWindowPos(ImVec2(viewWidth - kToastMargin, kToastMargin), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);  // fades background + text together
    ImGui::Begin("Toast", nullptr,
                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
                      ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoFocusOnAppearing);
    LatticeStyle::TextBoldColored(LatticeStyle::kFriendly, "%s", gToastMessage.c_str());
    ImGui::End();
    ImGui::PopStyleVar();
}

void DrawSimulationSection(SidebarState& state) {
    if (!SectionHeader("Simulation", /*defaultOpen=*/true)) return;

    if (!state.groundImpactMessage.empty()) {
        ImGui::TextColored(LatticeStyle::kThreat, "%s", state.groundImpactMessage.c_str());
    }

    // Sim time, time scale and pause are in the bottom-centre HUD; what is
    // drawn is in Settings.

    // Paused-only: main.cpp forces the mode off when the clock starts, so this
    // greys out rather than offering a checkbox that switches itself off.
    bool simRunning = !state.simClock.Paused();
    ImGui::BeginDisabled(simRunning);
    ImGui::Checkbox("Waypoint Placement Mode", &state.waypointPlacementMode);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("%s", simRunning ? "Pause the simulation to edit waypoints."
                                           : "Click the 3D view to add a waypoint to the selected aircraft.\n"
                                             "Drag a waypoint up or down to change its altitude; right-click\n"
                                             "one to delete it and the rest of the route. Click another\n"
                                             "aircraft to switch to it. Ctrl+Z drops the last point. (W)");
    }
    // A state, not help: placement does nothing until something is selected.
    if (state.waypointPlacementMode && !EntityInspector::SelectedEntity().has_value()) {
        ImGui::TextColored(LatticeStyle::kCaution, "Select an aircraft first.");
    }

    ImGui::Checkbox("Center Camera on Selection", &state.centerOnSelectedEntity);
}

constexpr float kTimeHudWidth = 420.0f;
constexpr float kTimeHudBottomMargin = 20.0f;

// Bottom-centre playback HUD: sim time, Pause and time scale, centred on
// the 3D view rather than the window.
//
// The slider is a comfortable drag range; the typed box goes further,
// since a long BVR run wants more than 10x. The real ceiling is wall-clock
// cost: physics always sub-steps at the fixed rate, so a high scale just
// runs slower than requested. The floor is above zero because 0 is Pause.
constexpr float kTimeScaleSliderMin = 0.1f;
constexpr float kTimeScaleSliderMax = 10.0f;
constexpr float kTimeScaleTypedMin = 0.01f;
constexpr float kTimeScaleTypedMax = 100.0f;
constexpr float kTimeScaleInputWidth = 64.0f;

// Speed presets, on the toolbar and keys 1-4.
constexpr float kSpeedPresets[] = {1.0f, 10.0f, 50.0f, 100.0f};

void RestartRun(World& world, SimContext& simContext, SidebarState& state, bool fromFile);
bool gShowShortcuts = false;

// <recordingDir>/<scenario stem>_<local date>_<time>.csv, with a counter if
// two recordings start within the same second (a quick Reset).
std::string NewRecordingPath(const SidebarState& state) {
    std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
    std::string base = std::filesystem::path(state.scenarioPath).stem().string() + "_" + stamp;
    std::filesystem::path path = std::filesystem::path(state.recordingDir) / (base + ".csv");
    for (int n = 2; std::filesystem::exists(path); ++n) {
        path = std::filesystem::path(state.recordingDir) / (base + "_" + std::to_string(n) + ".csv");
    }
    return path.string();
}

// Opens a new file and writes the current state as its first sample, so a
// recording started at t=0 (or rolled over by a restart) includes the start.
void StartRecording(World& world, SimContext& simContext, SidebarState& state) {
    std::error_code ec;
    std::filesystem::create_directories(state.recordingDir, ec);
    std::string error;
    if (!state.recorder.Open(NewRecordingPath(state), error)) {
        ShowToast(("Recording failed: " + error).c_str());
        return;
    }
    state.recorder.Sample(world, simContext, state.simTime);
}

void StopRecording(SidebarState& state) {
    std::string path = state.recorder.Path();
    state.recorder.Close();
    ShowToast(("Recording saved to " + path).c_str());
}

void DrawTimeHud(World& world, SimContext& simContext, SidebarState& state) {
    if (!ViewSettingsStore::Get().showTimeHud) return;
    ImGuiIO& io = ImGui::GetIO();
    float viewWidth = io.DisplaySize.x - gSidebarWidth;
    ImGui::SetNextWindowPos(ImVec2(viewWidth * 0.5f, io.DisplaySize.y - kTimeHudBottomMargin), ImGuiCond_Always,
                             ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowSize(ImVec2(kTimeHudWidth, 0.0f), ImGuiCond_Always);

    ImGui::Begin("TimeHud", nullptr,
                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize);

    {
        LatticeStyle::ScopedDataFont dataFont;
        LatticeStyle::TextBold("Sim Time:");
        ImGui::SameLine();
        ImGui::Text("%.2fs", state.simTime);
    }

    bool isPaused = state.simClock.Paused();
    if (ImGui::Checkbox("Pause", &isPaused)) state.simClock.TogglePause();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pause / run (Space)");
    ImGui::SameLine();

    // Both are rebuilt from the same state every frame, so they cannot
    // disagree; a drag after typing a high value pulls it back into the
    // slider's range.
    // The run speed, not the live 0 while paused: setting it while paused
    // keeps the clock paused.
    float timeScale = static_cast<float>(state.simClock.RunScale());
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - kTimeScaleInputWidth - ImGui::GetStyle().ItemSpacing.x);
    bool changed = ImGui::SliderFloat("##TimeScale", &timeScale, kTimeScaleSliderMin, kTimeScaleSliderMax, "%.2fx",
                                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    // InputText cannot centre, so pad the left by the leftover half; the 1px
    // slack keeps rounding from making ImGui scroll the value.
    char timeScaleText[32];
    std::snprintf(timeScaleText, sizeof(timeScaleText), "%.2f", timeScale);
    float centeringPad = (kTimeScaleInputWidth - ImGui::CalcTextSize(timeScaleText).x) * 0.5f - 1.0f;
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
                         ImVec2(std::max(ImGui::GetStyle().FramePadding.x, centeringPad),
                                ImGui::GetStyle().FramePadding.y));
    ImGui::SetNextItemWidth(kTimeScaleInputWidth);
    // Committed on Enter only (clicking away discards the typed text), so
    // typing "25" never applies "2".
    changed |= ImGui::InputFloat("##TimeScaleInput", &timeScale, 0.0f, 0.0f, "%.2f",
                                  ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::PopStyleVar();
    if (changed) {
        state.simClock.SetRunScale(std::clamp(timeScale, kTimeScaleTypedMin, kTimeScaleTypedMax));
    }

    for (size_t i = 0; i < std::size(kSpeedPresets); ++i) {
        char label[16];
        std::snprintf(label, sizeof(label), "%.0fx", kSpeedPresets[i]);
        if (i > 0) ImGui::SameLine();
        if (ImGui::Button(label)) state.simClock.SetRunScale(kSpeedPresets[i]);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Key %zu", i + 1);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    if (ImGui::Button("Reset")) RestartRun(world, simContext, state, /*fromFile=*/false);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Restart keeping your edits, paused at t=0 (R)");
    ImGui::SameLine();
    if (ImGui::Button("Reload")) RestartRun(world, simContext, state, /*fromFile=*/true);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Re-read the file, discarding edits, paused at t=0 (Shift+R)");
    ImGui::SameLine();
    bool recording = state.recorder.IsOpen();
    if (recording) ImGui::PushStyleColor(ImGuiCol_Text, LatticeStyle::kAccentPrimary);
    if (ImGui::Button(recording ? "Stop Rec" : "Record")) {
        if (recording) StopRecording(state);
        else StartRecording(world, simContext, state);
    }
    if (recording) ImGui::PopStyleColor();
    if (ImGui::IsItemHovered()) {
        if (recording) {
            ImGui::SetTooltip("Recording every tick to\n%s\nClick to stop and close the file.",
                              state.recorder.Path().c_str());
        } else {
            ImGui::SetTooltip("Write every tick's changing state (every entity and radar track) to a CSV\n"
                              "in the recordings folder beside the app. Reset or Reload starts a new file.");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("?")) gShowShortcuts = !gShowShortcuts;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Keyboard shortcuts (?)");

    ImGui::End();
}

constexpr float kScaleHudMargin = 20.0f;

// Bottom-right scale legend: the ground grid's current spacing, from the
// same function that sizes the grid.
void DrawGridScaleHud(SidebarState& state) {
    if (!ViewSettingsStore::Get().showGrid) return;
    ImGuiIO& io = ImGui::GetIO();
    float viewWidth = io.DisplaySize.x - gSidebarWidth;
    ImGui::SetNextWindowPos(ImVec2(viewWidth - kScaleHudMargin, io.DisplaySize.y - kScaleHudMargin),
                             ImGuiCond_Always, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_Always);

    ImGui::Begin("GridScaleHud", nullptr,
                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
                      ImGuiWindowFlags_NoFocusOnAppearing);

    float spacingFt = RenderSystem::GridSpacingFt(state.cameraDistance);
    {
        LatticeStyle::ScopedDataFont dataFont;
        LatticeStyle::TextBoldColored(LatticeStyle::kTextSecondary, "Grid:");
        ImGui::SameLine();
        ImGui::TextColored(LatticeStyle::kTextSecondary, "%.0f ft", spacingFt);
    }

    ImGui::End();
}

constexpr float kWaypointPanelVerticalOffsetPx = 40.0f;
constexpr float kWaypointPanelSpeedFieldWidth = 120.0f;
// Wide enough for a six-digit coordinate plus sign -- scenario positions
// run to +-150000 ft.
constexpr float kWaypointPanelCoordFieldWidth = 110.0f;
// The Entity Inspector's speed-command range, so the two agree.
constexpr float kWaypointSpeedMinKts = 0.0f;
constexpr float kWaypointSpeedMaxKts = 600.0f;
// A new speed override's start when the entity has no autopilot to copy.
constexpr double kFallbackWaypointSpeedKts = 300.0;

// Panel above a selected waypoint: the live leg vector from the previous
// point (or the entity, for the first) and an editable speed override.
// main.cpp computes the screen position; drawn only when on screen.
void DrawWaypointEditPanel(World& world, SidebarState& state) {
    if (!state.selectedWaypointIndex.has_value() || !state.hasSelectedWaypointScreenPos) return;

    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    ComponentStore<WaypointComponent>& waypointStore = world.Components<WaypointComponent>();
    if (!selected.has_value() || !waypointStore.Has(*selected)) return;
    WaypointComponent& waypoint = waypointStore.Get(*selected);
    size_t index = *state.selectedWaypointIndex;
    if (index >= waypoint.queue.size()) return;
    WaypointPoint& point = waypoint.queue[index];

    ImGui::SetNextWindowPos(
        ImVec2(state.selectedWaypointScreenX, state.selectedWaypointScreenY - kWaypointPanelVerticalOffsetPx),
        ImGuiCond_Always, ImVec2(0.5f, 1.0f));

    ImGui::Begin("WaypointEditPanel", nullptr,
                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize);

    LatticeStyle::TextBoldColored(LatticeStyle::kTextSecondary, "Waypoint %zu", index);

    Vec3 reference;
    bool hasReference = false;
    if (index > 0) {
        reference = waypoint.queue[index - 1].pos;
        hasReference = true;
    } else {
        ComponentStore<Transform>& transforms = world.Components<Transform>();
        if (transforms.Has(*selected)) {
            reference = transforms.Get(*selected).pos;
            hasReference = true;
        }
    }
    // Absolute position in the scenario JSON's frame and units, so a value can
    // be typed straight in from a file. AGL = MSL: the ground is the plane y=0.
    // Written back only on a change, so it does not fight the altitude drag.
    // Clamped to the Placement rows' ranges, so a route cannot run where no
    // entity could be placed.
    auto placementField = [&](const char* label, int axis) {
        const ParamRegistry::ParamDef* param =
            ParamRegistry::Find("/entities/{}/transform/pos/" + std::to_string(axis));
        float value = static_cast<float>(point.pos[axis]);
        ImGui::SetNextItemWidth(kWaypointPanelCoordFieldWidth);
        if (ImGui::InputFloat(label, &value, 0.0f, 0.0f, "%.0f", ImGuiInputTextFlags_EnterReturnsTrue))
            point.pos[axis] = std::clamp(static_cast<double>(value), param->minValue, param->maxValue);
    };
    placementField("East (ft)", 0);
    placementField("Altitude AGL (ft)", 1);
    placementField("North (ft)", 2);

    if (hasReference) {
        {
            LatticeStyle::ScopedDataFont dataFont;
            LatticeStyle::TextBoldColored(LatticeStyle::kTextSecondary, "From previous:");
            ImGui::SameLine();
            ImGui::TextColored(LatticeStyle::kTextSecondary, "dE=%.0f  dAlt=%.0f  dN=%.0f ft", point.pos.x() - reference.x(),
                                point.pos.y() - reference.y(), point.pos.z() - reference.z());
        }
    }

    // Unset leaves the autopilot's speed alone (WaypointComponent.h). A new
    // override starts at the entity's commanded speed, never a silent stop.
    bool hasSpeed = point.speedKts.has_value();
    if (ImGui::Checkbox("Override Speed", &hasSpeed)) {
        if (hasSpeed) {
            ComponentStore<AutopilotComponent>& autopilots = world.Components<AutopilotComponent>();
            point.speedKts = autopilots.Has(*selected) ? autopilots.Get(*selected).targetSpeedKts : kFallbackWaypointSpeedKts;
        } else {
            point.speedKts.reset();
        }
    }
    if (point.speedKts.has_value()) {
        ImGui::SetNextItemWidth(kWaypointPanelSpeedFieldWidth);
        float speed = static_cast<float>(*point.speedKts);
        // AlwaysClamp: a Ctrl+click typed value would otherwise pass the range.
        if (ImGui::DragFloat("Speed (kts)", &speed, 1.0f, kWaypointSpeedMinKts, kWaypointSpeedMaxKts, "%.3f",
                             ImGuiSliderFlags_AlwaysClamp))
            point.speedKts = speed;
    }

    if (ImGui::SmallButton("Close")) state.selectedWaypointIndex.reset();

    ImGui::End();
}

// Whether the path exists; an unreadable one counts as existing, so Save As
// asks before overwriting rather than throwing.
bool FileExists(const std::string& path) {
    std::error_code ec;
    bool exists = std::filesystem::exists(path, ec);
    return exists || ec;
}

// Recursive, for scenarios/custom/. generic_string keeps forward slashes
// on every platform.
std::vector<std::string> ScanScenarioFiles() {
    std::vector<std::string> result;
    // Error-code forms throughout: an unreadable subfolder ends the listing
    // early rather than throwing out of the GUI.
    std::error_code ec;
    for (std::filesystem::recursive_directory_iterator it("scenarios", ec), end; !ec && it != end;
         it.increment(ec)) {
        std::error_code entryError;
        if (it->is_regular_file(entryError) && it->path().extension() == ".json") {
            std::filesystem::path relative = it->path().lexically_relative("scenarios");
            result.push_back("scenarios/" + relative.generic_string());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

// Cached: the folder changes at runtime only through Save As, which
// refreshes it.
std::vector<std::string>& ScenarioFiles() {
    static std::vector<std::string> files = ScanScenarioFiles();
    return files;
}

void RefreshScenarioFiles() { ScenarioFiles() = ScanScenarioFiles(); }

// Everything a fresh main() sets up (RNG seed, event log, SimClock,
// simTime, crash message, camera follow), once the world is repopulated.
// Shared by Reload and Reset so they agree on what "a new run" is.
//
// All of SimContext is cleared: a reloaded world restarts entity IDs at 1,
// so leftover radar tracks (keyed by ID pairs) would alias the new
// entities, and the outcome would still read the last run's `resolved`.
void ResetRunState(World& world, SimContext& simContext, SidebarState& state) {
    // Seeds the RNG, so a Reset replays the same run from the same seed.
    SimLoop::BeginRun(state.scenario, simContext, state.scenario.seed);
    simContext.eventLog.Clear();
    simContext.airborneRadarTracks.clear();
    simContext.outcome = EngagementOutcome{};

    state.simClock = SimClock();
    // Starts paused, as a fresh launch does.
    state.simClock.StartPaused(state.scenario.simSpeed);
    state.simTime = 0.0;
    state.groundImpactMessage.clear();
    RenderSystem::ResetTrails();

    LaunchSystem::InitFromTags(world, state.scenario);
    std::optional<EntityID> target = EntityLookup::FindByTagName(world, "Target");

    // Recentre on Target. Selects it too, or main.cpp's selection sync drops
    // the follow next frame.
    if (target.has_value()) {
        EntityInspector::SetSelectedEntity(*target);
        state.followedEntity = target;
    } else {
        state.followedEntity.reset();
        // Entity ids restart at 1 on a reload: a selection kept from the old
        // world would name whatever now has that number.
        EntityInspector::ClearSelection();
    }

    // A recording holds one run: a restart rolls it over to a new file.
    if (state.recorder.IsOpen()) StartRecording(world, simContext, state);
}

// The Scenario section's Reload button: rebuilds everything from the file
// on disk, discarding every edit.
void ReloadScenario(World& world, SimContext& simContext, SidebarState& state, std::string& errorMessage) {
    // Loaded into a scratch world and swapped in only on success, so a bad
    // file fails without wiping the current scenario.
    World fresh;
    ScenarioData freshScenario;
    try {
        freshScenario = ScenarioLoader::Load(state.scenarioPath, fresh);
    } catch (const std::exception& e) {
        errorMessage = e.what();
        return;
    }
    world.ReplaceWith(std::move(fresh));
    state.scenario = std::move(freshScenario);
    ResetRunState(world, simContext, state);
    errorMessage.clear();
}

// Loads a different scenario file. scenarioPath must always name the file
// the world was built from -- Reset and Save As overlay the current world
// onto it -- so the path only changes when the load succeeds.
void LoadScenarioFile(World& world, SimContext& simContext, SidebarState& state, const std::string& path,
                      std::string& errorMessage) {
    std::string previousPath = state.scenarioPath;
    state.scenarioPath = path;
    ReloadScenario(world, simContext, state, errorMessage);
    if (!errorMessage.empty()) state.scenarioPath = previousPath;
}

// Restarts the run from the scenario AS CURRENTLY EDITED rather than from
// the file: every parameter change and every waypoint survives, all run
// state goes. That is the difference from Reload, and what makes it
// usable for changing one number and watching what it does.
void ResetScenario(World& world, SimContext& simContext, SidebarState& state, std::string& errorMessage) {
    std::string error;
    if (!ScenarioSaver::ResetWorld(state.scenarioPath, world, state.scenario, error)) {
        errorMessage = error;
        return;
    }
    ResetRunState(world, simContext, state);
    errorMessage.clear();
}

// Lays short pieces of text left to right, moving the next piece to a new
// line at `wrapX` (window-local; default: where the line starts) when it
// would pass the panel's right edge. A narrow sidebar then wraps between
// fields instead of clipping one; a piece wider than a whole line wraps
// inside itself.
class FlowLine {
public:
    FlowLine() : FlowLine(ImGui::GetCursorPosX()) {}
    explicit FlowLine(float wrapX)
        : wrapX_(wrapX), rightX_(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x) {}

    void Text(const char* fmt, ...) {
        char text[128];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(text, sizeof(text), fmt, args);
        va_end(args);
        Place(ImGui::CalcTextSize(text).x);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
    }

    // A bold label and its value, kept together when they fit on one line.
    void Pair(const char* label, const char* fmt, ...) {
        char value[128];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(value, sizeof(value), fmt, args);
        va_end(args);
        float labelWidth = ImGui::CalcTextSize(label).x;
        float valueWidth = ImGui::CalcTextSize(value).x;
        if (labelWidth + valueWidth <= rightX_ - wrapX_) {
            Place(labelWidth + valueWidth);
            LatticeStyle::TextBold("%s", label);
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::TextUnformatted(value);
        } else {
            Place(labelWidth);
            LatticeStyle::TextBold("%s", label);
            Text("%s", value);
        }
    }

private:
    void Place(float width) {
        if (first_) {
            first_ = false;
            return;
        }
        ImGui::SameLine(0.0f, 0.0f);
        if (width > ImGui::GetContentRegionAvail().x) {
            ImGui::NewLine();
            ImGui::SetCursorPosX(wrapX_);
        }
    }

    float wrapX_;
    float rightX_;
    bool first_ = true;
};

// A coordinate rounded for display, with negative zero folded: "%.0f" of
// -0.3 would print "-0".
double DisplayFt(double v) {
    double rounded = std::round(v);
    return rounded == 0.0 ? 0.0 : rounded;
}

// The Scenario section's state between frames. Not in SidebarState,
// which main.cpp rebuilds every frame.
struct ScenarioPanelState {
    std::string reloadError;
    char saveAsFilename[128] = "";
    // The existing file a first Save As press found; a second press with
    // the same name overwrites it.
    std::string pendingOverwritePath;
    std::string saveStatus;
    bool saveStatusIsError = false;
    double saveStatusShownAtSec = Numeric::kLongAgoS;
};
ScenarioPanelState gScenarioPanel;

void DrawScenarioFileMenu(World& world, SimContext& simContext, SidebarState& state);

void DrawScenarioSection(World& world, SimContext& simContext, SidebarState& state) {
    // Open by default: it carries the scenario's "what to watch" text.
    if (!SectionHeader("Scenario", /*defaultOpen=*/true)) return;
    ScenarioPanelState& panel = gScenarioPanel;

    if (!state.scenario.description.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextUnformatted(state.scenario.description.c_str());
        ImGui::PopTextWrapPos();
        ImGui::Separator();
    }

    if (ImGui::BeginCombo("File", state.scenarioPath.c_str())) {
        for (const std::string& file : ScenarioFiles()) {
            bool selected = (file == state.scenarioPath);
            if (ImGui::Selectable(file.c_str(), selected) && !selected) {
                LoadScenarioFile(world, simContext, state, file, panel.reloadError);
                ShowToast(panel.reloadError.empty() ? "Scenario loaded" : "Scenario failed to load -- see the Scenario section");
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::Button("Reload")) RestartRun(world, simContext, state, /*fromFile=*/true);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Re-read the file from disk. Discards every parameter and waypoint edit. (Shift+R)");
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) RestartRun(world, simContext, state, /*fromFile=*/false);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Restart the run keeping your edits, so you can change one value and\n"
                          "re-run against the same seed. An unedited Reset reproduces the run exactly. (R)");
    }
    ImGui::SameLine();
    if (ImGui::Button("File...")) ImGui::OpenPopup("ScenarioFileMenu");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("New scenario, Save As");
    DrawScenarioFileMenu(world, simContext, state);
    if (!panel.reloadError.empty()) {
        // Wrapped: a loader error names the file, key and cause, far wider
        // than the sidebar.
        ImGui::PushStyleColor(ImGuiCol_Text, LatticeStyle::kThreat);
        ImGui::TextWrapped("Failed: %s", panel.reloadError.c_str());
        ImGui::PopStyleColor();
    }

    {
        LatticeStyle::ScopedDataFont dataFont;
        FlowLine line;
        line.Pair("Seed ", "%llu   ", static_cast<unsigned long long>(state.scenario.seed));
        // The file's starting speed, not the live one (the top bar has
        // that) -- labelled so the two don't read as contradicting.
        line.Pair("Start Speed ", "%.2fx", state.scenario.simSpeed);
    }

    if (!panel.saveStatus.empty()) {
        float alpha = TransientAlpha(panel.saveStatusShownAtSec);
        if (alpha <= 0.0f) {
            panel.saveStatus.clear();
        } else {
            ImVec4 color = panel.saveStatusIsError ? LatticeStyle::kThreat : LatticeStyle::kFriendly;
            color.w *= alpha;
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(color, "%s", panel.saveStatus.c_str());
            ImGui::PopTextWrapPos();
        }
    }
}

// The Scenario section's File popup: a fresh scenario from the template,
// and Save As.
void DrawScenarioFileMenu(World& world, SimContext& simContext, SidebarState& state) {
    ScenarioPanelState& panel = gScenarioPanel;
    if (!ImGui::BeginPopup("ScenarioFileMenu")) {
        // Closing the popup disarms Overwrite: reopened, it asks again.
        panel.pendingOverwritePath.clear();
        return;
    }

    // New Scenario reloads from the minimal 2-aircraft template.
    if (ImGui::Button("New Scenario")) {
        LoadScenarioFile(world, simContext, state, kNewScenarioTemplatePath, panel.reloadError);
        ShowToast(panel.reloadError.empty() ? "New scenario loaded" : "Scenario failed to load -- see the Scenario section");
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Start fresh from a minimal 2-aircraft template you can reposition and route.");
    }
    ImGui::Separator();

    ImGui::SetNextItemWidth(kSaveAsFieldWidth);
    ImGui::InputTextWithHint("##SaveAsFilename", "filename", panel.saveAsFilename, sizeof(panel.saveAsFilename));
    std::string nameError;
    std::string filename = ScenarioSaver::CustomFileName(panel.saveAsFilename, nameError);
    std::string destPath = filename.empty() ? "" : std::string(kCustomScenarioDir) + "/" + filename;
    bool confirmingOverwrite = !destPath.empty() && destPath == panel.pendingOverwritePath;
    // ImGui keeps a popup inside the window only on the frame it opens, so
    // the popup must not widen after that: the button is sized for its
    // longer label and the caution line wraps to the row.
    const ImGuiStyle& style = ImGui::GetStyle();
    float buttonWidth = std::max(ImGui::CalcTextSize("Save As").x, ImGui::CalcTextSize("Overwrite").x) +
                        2.0f * style.FramePadding.x;
    float rowWidth = kSaveAsFieldWidth + style.ItemSpacing.x + buttonWidth;
    ImGui::SameLine();
    if (ImGui::Button(confirmingOverwrite ? "Overwrite" : "Save As", ImVec2(buttonWidth, 0.0f))) {
        bool closePopup = true;
        if (filename.empty()) {
            panel.saveStatus = nameError;
            panel.saveStatusIsError = true;
        } else if (!confirmingOverwrite && FileExists(destPath)) {
            // Stay open so the second press is one click away.
            panel.pendingOverwritePath = destPath;
            closePopup = false;
        } else {
            std::error_code ec;
            std::filesystem::create_directories(kCustomScenarioDir, ec);
            std::string saveError;
            bool saved = ScenarioSaver::Save(state.scenarioPath, destPath, world, state.scenario, saveError);
            panel.saveStatusIsError = !saved;
            if (saved) {
                panel.saveStatus = "Saved to " + destPath;
                // The new file must show in the cached File dropdown.
                RefreshScenarioFiles();
            } else {
                panel.saveStatus = saveError;
            }
            panel.pendingOverwritePath.clear();
        }
        panel.saveStatusShownAtSec = ImGui::GetTime();
        if (closePopup) ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Saves every edited parameter and waypoint route into a copy of the loaded\n"
                          "scenario, under scenarios/custom/. Point an experiment at the saved file to\n"
                          "sweep against these values.");
    }
    if (confirmingOverwrite) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + rowWidth);
        ImGui::TextColored(LatticeStyle::kCaution, "%s exists. Press Overwrite to replace it.", filename.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::EndPopup();
}

// A finished placement edit is re-applied with a keep-edits Reset: the
// flight models were started from the old pose, and would otherwise put
// the aircraft back there when the clock runs. Reset rebuilds the world
// (new entity IDs) and reselects the Target, so the edited entity is
// reselected by name. Runs outside the panel, which is iterating the world.
void ApplySetupEdits(World& world, SimContext& simContext, SidebarState& state) {
    if (!ParameterPanel::TakeSetupEdit()) return;
    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    ComponentStore<Tag>& tags = world.Components<Tag>();
    std::string name = selected.has_value() && tags.Has(*selected) ? tags.Get(*selected).name : "";

    ResetScenario(world, simContext, state, gScenarioPanel.reloadError);
    if (!gScenarioPanel.reloadError.empty()) {
        ParameterPanel::RevertSetupEdit(state.scenario, world);
        gScenarioPanel.reloadError += " (edit undone)";
        return;
    }
    if (!name.empty()) {
        if (std::optional<EntityID> again = EntityLookup::FindByTagName(world, name)) {
            EntityInspector::SetSelectedEntity(*again);
            state.followedEntity = again;
        }
    }
}

// The selected entity's scenario parameters, generated from
// ParamRegistry -- the same table the saver writes and the batch runner
// overrides, so anything editable here is directly sweepable.
//
// Scoped to one entity because that is how a user thinks about it: pick
// the missile to tune the weapon, pick an aircraft to tune the airframe
// or its radar. Missile doctrine is stored once at the scenario level
// rather than per entity, but it describes the missile, so it appears
// under it.
void DrawParametersSection(World& world, SidebarState& state) {
    if (!SectionHeader("Parameters", /*defaultOpen=*/false)) return;

    // The same selection as the Entity Inspector and a 3D-view click, repeated
    // here so the section is usable on its own.
    ComponentStore<Tag>& tags = world.Components<Tag>();
    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    if (selected.has_value() && world.Entities().count(*selected) == 0) selected.reset();

    std::string preview = selected.has_value() && tags.Has(*selected) ? tags.Get(*selected).name : "(none)";
    ImGui::SetNextItemWidth(kEntityComboWidth);
    if (ImGui::BeginCombo("Entity", preview.c_str())) {
        for (auto& [entity, tag] : tags) {
            bool isSelected = selected.has_value() && *selected == entity;
            // Several entities can share a name (every launched missile is
            // "Missile"); the id keeps their rows distinct to ImGui.
            ImGui::PushID(static_cast<int>(entity));
            if (ImGui::Selectable(tag.name.c_str(), isSelected)) EntityInspector::SetSelectedEntity(entity);
            ImGui::PopID();
            if (isSelected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("\"(next launch)\" is copied into the missile when it fires, so editing one\n"
                          "mid-flight does nothing. \"(setup)\" is the start state: editing one restarts\n"
                          "the scenario from the new start (keeping your edits), and it locks once the\n"
                          "clock is running. Advanced mode (Settings) adds the finer knobs and the\n"
                          "type files' values, greyed out.");
    }

    if (!selected.has_value()) {
        ImGui::TextDisabled("Select an entity to edit its parameters.");
        return;
    }

    ImGui::Spacing();
    bool advancedMode = ViewSettingsStore::Get().advancedMode;
    bool drew = false;
    {
        LatticeStyle::ScopedDataFont dataFont;
        drew = ParameterPanel::DrawForEntity(state.scenario, world, *selected, state.simTime, advancedMode);
    }

    if (!drew) {
        ImGui::TextDisabled(advancedMode ? "This entity has no scenario parameters."
                                         : "Nothing to edit here; Advanced mode shows more.");
    }
}

void DrawEntityTelemetry(World& world, const char* name) {
    std::optional<EntityID> entity = EntityLookup::FindByTagName(world, name);
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    if (!entity.has_value() || !transforms.Has(*entity)) {
        ImGui::TextDisabled("%s: not present", name);
        return;
    }

    const Transform& transform = transforms.Get(*entity);
    Vec3 vel = EntityVelocity::Get(world, *entity);
    double speed = vel.norm();

    {
        LatticeStyle::ScopedDataFont dataFont;
        // The name padded to a fixed column, so the numbers line up across
        // entities; wrapped pieces hang at that column.
        char padded[16];
        std::snprintf(padded, sizeof(padded), "%-8s ", name);
        float valuesX = ImGui::GetCursorPosX() + ImGui::CalcTextSize(padded).x;
        {
            FlowLine line(valuesX);
            line.Pair(padded, "E %.0f ft   ", DisplayFt(transform.pos.x()));
            line.Text("N %.0f ft", DisplayFt(transform.pos.z()));
        }
        ImGui::SetCursorPosX(valuesX);
        ImGui::Text("Alt %.0f ft AGL", DisplayFt(transform.pos.y()));
        // A destroyed entity is frozen where it died; its last speed and
        // heading would read as live. Muted like the inspector's Status.
        ImGui::SetCursorPosX(valuesX);
        if (world.Components<DestroyedComponent>().Has(*entity)) {
            ImGui::TextColored(LatticeStyle::kTextSecondary, "Destroyed (frozen)");
        } else {
            FlowLine line(valuesX);
            line.Text("Speed %.1f ft/s   ", speed);
            line.Text("Hdg %.1f deg", transform.euler.z());
        }
    }
}

// Hidden unless verbose. The line is repetition rate, not importance: these
// fire every 120 Hz tick while over-G holds and outnumber every other
// event type combined by about a hundred to one.
bool IsVerboseOnlyEvent(const std::string& type) {
    return type == "G_LIMIT_EXCEEDED" || type == "MISSILE_G_LIMIT_EXCEEDED";
}

// A closest-approach MISS (the only kind with a Pk roll and no cause=)
// whose warhead, not its guidance, decided the outcome.
bool IsWarheadNoKill(const SimEvent& event) {
    return event.type == "MISS" && EventField(event.data, "warhead") == "no_kill";
}

// Readable lead-in for a closest-approach MISS: how close it passed, and
// why that did not kill when it was close enough to. Empty otherwise.
std::string MissSummary(const SimEvent& event) {
    if (event.type != "MISS" || !EventField(event.data, "cause").empty()) return "";
    std::string pk = EventField(event.data, "pk");
    std::string missM = EventField(event.data, "miss_distance_m");
    if (pk.empty() || missM.empty()) return "";
    char buf[96];
    if (IsWarheadNoKill(event)) {
        std::snprintf(buf, sizeof(buf), "- passed %.1f m, warhead did not kill (Pk %.0f%%) ", std::stod(missM),
                      100.0 * std::stod(pk));
    } else if (EventField(event.data, "armed") == "no") {
        std::snprintf(buf, sizeof(buf), "- passed %.1f m, not armed ", std::stod(missM));
    } else {
        std::snprintf(buf, sizeof(buf), "- passed %.1f m ", std::stod(missM));
    }
    return buf;
}

ImVec4 EventColor(const SimEvent& event) {
    const std::string& type = event.type;
    if (type == "LAUNCH") return LatticeStyle::kAccentSecondary;
    if (type == "INTERCEPT") return LatticeStyle::kFriendly;
    if (IsWarheadNoKill(event)) return LatticeStyle::kCaution;
    if (type == "MISS") return LatticeStyle::kThreat;
    if (type == "RWR_ILLUMINATED" || type == "RWR_CLEAR") return LatticeStyle::kAccentSecondary;
    return LatticeStyle::kTextPrimary;
}

// The event log, filtered, with each entry's laid-out height. An entry is
// its header, then its data wrapped by ImGui beside it, so wrapped lines
// hang under the data rather than the time stamp. When a long event type
// leaves no room beside it for the data's longest key=value pair, ImGui
// would split that pair mid-number, so the data goes on its own indented
// line instead. Heights are measured once per entry and again only when
// the log width changes, so a long verbose log draws just the entries in
// view.
struct EventLogView {
    struct Entry {
        std::string header;
        std::string data;
        ImVec4 color;
        bool dataOnOwnLine = false;
    };
    static constexpr const char* kDataIndent = "        ";

    std::vector<Entry> entries;
    std::vector<float> tops;  // content y of each measured entry, plus the end
    size_t eventsSeen = 0;
    bool verbose = false;
    float width = -1.0f;

    // How many events the non-verbose view hides. Independent of layout,
    // so the header can show it before the list is drawn.
    int verboseOnlyCount = 0;
    size_t eventsCounted = 0;
    // The log's generation each cache was built from. A new run's log can
    // already be as long as the old one when this view next looks (the
    // section was collapsed through a Reset), so size alone cannot tell.
    unsigned long long seenGeneration = 0;
    unsigned long long countedGeneration = 0;

    void CountVerboseOnly(const EventLog& log) {
        const std::vector<SimEvent>& events = log.Events();
        if (log.Generation() != countedGeneration || events.size() < eventsCounted) {
            verboseOnlyCount = 0;
            eventsCounted = 0;
            countedGeneration = log.Generation();
        }
        for (; eventsCounted < events.size(); ++eventsCounted) {
            if (IsVerboseOnlyEvent(events[eventsCounted].type)) ++verboseOnlyCount;
        }
    }

    // Call inside the log's child window with its font pushed: heights
    // are measured with the font and width actually drawn with.
    void Sync(const EventLog& log, bool verboseNow, float widthNow) {
        const std::vector<SimEvent>& events = log.Events();
        if (log.Generation() != seenGeneration || events.size() < eventsSeen || verboseNow != verbose) {
            entries.clear();
            eventsSeen = 0;
            verbose = verboseNow;
            seenGeneration = log.Generation();
        }
        if (widthNow != width || entries.empty()) {
            tops.assign(1, 0.0f);
            width = widthNow;
        }
        for (; eventsSeen < events.size(); ++eventsSeen) Append(events[eventsSeen]);

        float lineHeight = ImGui::GetTextLineHeight();
        float spacing = ImGui::GetStyle().ItemSpacing.y;
        float indentWidth = ImGui::CalcTextSize(kDataIndent).x;
        for (size_t i = tops.size() - 1; i < entries.size(); ++i) {
            // ImGui's TextWrapped wraps at the content edge, from wherever
            // the text starts (CalcWrapWidthForPos).
            Entry& entry = entries[i];
            float besideWidth = width - ImGui::CalcTextSize(entry.header.c_str()).x;
            entry.dataOnOwnLine = LongestWordWidth(entry.data) > besideWidth;
            float wrapWidth = std::max(entry.dataOnOwnLine ? width - indentWidth : besideWidth, 1.0f);
            float dataHeight = ImGui::CalcTextSize(entry.data.c_str(), nullptr, false, wrapWidth).y;
            float height = entry.dataOnOwnLine ? lineHeight + spacing + dataHeight : std::max(lineHeight, dataHeight);
            tops.push_back(tops.back() + height + spacing);
        }
    }

private:
    static float LongestWordWidth(const std::string& text) {
        float longest = 0.0f;
        size_t start = 0;
        while (start < text.size()) {
            size_t end = std::min(text.find(' ', start), text.size());
            longest = std::max(longest, ImGui::CalcTextSize(text.data() + start, text.data() + end).x);
            start = end + 1;
        }
        return longest;
    }

    void Append(const SimEvent& event) {
        if (!verbose && IsVerboseOnlyEvent(event.type)) return;
        char header[96];
        std::snprintf(header, sizeof(header), "%7.2f %-10s e%u ", event.simTime, event.type.c_str(), event.entity);
        entries.push_back({header, MissSummary(event) + event.data, EventColor(event)});
    }
};

// Scrollable, monospace event log, coloured by type (EventColor).
void DrawEventLog(SimContext& simContext) {
    // Function-local statics: main.cpp rebuilds SidebarState every frame.
    static bool verbose = false;
    static EventLogView view;

    // Display only: EventLog records everything, so unticking reveals the
    // backlog.
    view.CountVerboseOnly(simContext.eventLog);
    ImGui::Checkbox("Verbose", &verbose);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Show per-tick G-limit chatter (G_LIMIT_EXCEEDED,\n"
            "MISSILE_G_LIMIT_EXCEEDED), which fires at 120Hz for as long as an\n"
            "aircraft or missile is over its envelope. Always recorded either\n"
            "way -- toggling this on mid-run reveals everything logged so far.");
    }
    if (!verbose && view.verboseOnlyCount > 0) {
        ImGui::SameLine();
        ImGui::TextColored(LatticeStyle::kTextSecondary, "(%d hidden)", view.verboseOnlyCount);
    }

    ImGui::BeginChild("EventLog", ImVec2(0.0f, kEventLogHeight), true);
    {
        LatticeStyle::ScopedDataFont dataFont;

        view.Sync(simContext.eventLog, verbose, ImGui::GetContentRegionAvail().x);

        // Only the entries in view are drawn; Dummy items stand in for the
        // rest so the scroll range stays right. Dummy adds its own item
        // spacing, which the measured tops already include.
        const std::vector<float>& tops = view.tops;
        float spacing = ImGui::GetStyle().ItemSpacing.y;
        // One line of margin covers the window padding the tops leave out.
        float margin = ImGui::GetTextLineHeightWithSpacing();
        float viewTop = ImGui::GetScrollY() - margin;
        float viewBottom = ImGui::GetScrollY() + ImGui::GetWindowHeight() + margin;
        size_t first = static_cast<size_t>(std::upper_bound(tops.begin(), tops.end(), viewTop) - tops.begin());
        first = first > 0 ? first - 1 : 0;
        size_t last = std::min(
            static_cast<size_t>(std::lower_bound(tops.begin(), tops.end(), viewBottom) - tops.begin()),
            view.entries.size());
        first = std::min(first, last);
        if (first > 0) ImGui::Dummy(ImVec2(0.0f, tops[first] - spacing));
        for (size_t i = first; i < last; ++i) {
            const EventLogView::Entry& entry = view.entries[i];
            ImGui::PushStyleColor(ImGuiCol_Text, entry.color);
            ImGui::TextUnformatted(entry.header.c_str());
            if (entry.dataOnOwnLine) {
                float indentWidth = ImGui::CalcTextSize(EventLogView::kDataIndent).x;
                ImGui::Indent(indentWidth);
                ImGui::TextWrapped("%s", entry.data.c_str());
                ImGui::Unindent(indentWidth);
            } else {
                ImGui::SameLine(0.0f, 0.0f);
                ImGui::TextWrapped("%s", entry.data.c_str());
            }
            ImGui::PopStyleColor();
        }
        if (last < view.entries.size()) ImGui::Dummy(ImVec2(0.0f, tops.back() - tops[last] - spacing));

        // Follow the newest event only while already at the bottom, so reading
        // history is not yanked away.
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
}

void DrawTelemetrySection(World& world, SidebarState& state) {
    if (!ViewSettingsStore::Get().showTelemetry) return;
    if (!SectionHeader("Telemetry", /*defaultOpen=*/false)) return;

    {
        LatticeStyle::ScopedDataFont dataFont;
        FlowLine line;
        line.Pair("Sim Time: ", "%.2fs   ", state.simTime);
        line.Pair("Time Scale: ", "%.2fx%s", state.simClock.RunScale(), state.simClock.Paused() ? " (paused)" : "");
    }
    ImGui::Separator();

    DrawEntityTelemetry(world, "Pursuer");
    DrawEntityTelemetry(world, "Target");
    ImGui::Separator();

    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    bool anyActiveMissile = false;
    for (auto& [entity, guidance] : guidances) {
        if (!guidance.active) continue;
        anyActiveMissile = true;

        // Center to center, in feet like the rest of this section. An
        // active missile always has a Transform; its target might not.
        const Transform* missileTransform = transforms.TryGet(entity);
        const Transform* targetTransform = transforms.TryGet(guidance.target);
        if (missileTransform == nullptr || targetTransform == nullptr) break;
        double rangeFt = (missileTransform->pos - targetTransform->pos).norm();
        {
            LatticeStyle::ScopedDataFont dataFont;
            FlowLine line;
            // "to target" is implied: a missile ranges only on its own target.
            line.Pair("Missile range: ", "%.0f ft ", rangeFt);
            line.Text("(%.0f m)", rangeFt / kFeetPerMeter);
        }
        break;
    }
    if (!anyActiveMissile) ImGui::TextDisabled("No missile currently active.");
}

void DrawEventLogSection(SimContext& simContext) {
    if (!ViewSettingsStore::Get().showEventLog) return;
    if (!SectionHeader("Event Log", /*defaultOpen=*/false)) return;
    DrawEventLog(simContext);
}

// What the simulator is for, and what it is not, in the viewer itself: the
// same few lines as README.md and REFERENCE.md, so nobody mistakes a
// teaching model for a prediction. Closed by default; it is read once.
void DrawAboutSection() {
    if (!SectionHeader("About", /*defaultOpen=*/false)) return;

    ImGui::PushTextWrapPos(0.0f);
    LatticeStyle::TextBold("What this is");
    ImGui::TextWrapped(
        "A learning tool for beyond-visual-range air combat, built from public data and first-principles "
        "physics. It models the features a real system has -- how the radar detects and tracks, how the "
        "missile flies, guides and runs out of energy -- at the fidelity that changes what you see.");
    ImGui::Spacing();
    LatticeStyle::TextBold("What it isn't");
    ImGui::TextWrapped(
        "Not a validated model of any real aircraft, missile or radar, and not an engagement-planning tool. "
        "Launch ranges, detection ranges and kill probabilities are illustrative.");
    ImGui::Spacing();
    ImGui::TextColored(LatticeStyle::kTextSecondary, "%s",
                       "Only the teaching knobs are editable. Advanced mode (Settings) also shows each "
                       "aircraft and missile type's own values, greyed out. See docs/REFERENCE.md.");
    ImGui::PopTextWrapPos();
}

void DrawEntityInspectorSection(World& world, const SimContext& simContext) {
    if (!SectionHeader("Entity Inspector", /*defaultOpen=*/true)) return;

    EntityInspector::Draw(world, simContext, ViewSettingsStore::Get().advancedMode);
}

constexpr float kSplitterWidth = 10.0f;

// Draggable splitter left of the sidebar. Resizes the width directly
// rather than using ImGui's border-drag: that moves position and size
// together, which fights the sidebar being forced to the right edge every
// frame. The strip never overlaps the sidebar's own rect, so there is no
// hit-test ambiguity between the two windows.
void DrawSidebarSplitter() {
    ImGuiIO& io = ImGui::GetIO();
    float splitterX = io.DisplaySize.x - gSidebarWidth - kSplitterWidth;

    ImGui::SetNextWindowPos(ImVec2(splitterX, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(kSplitterWidth, io.DisplaySize.y), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    // Zero padding: the default 8px each side would leave the 10px window's
    // button no width to catch a drag.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("SidebarSplitter", nullptr,
                  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
                      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoNav);

    ImGui::InvisibleButton("##splitter", ImVec2(kSplitterWidth, io.DisplaySize.y));
    bool hovered = ImGui::IsItemHovered();
    bool active = ImGui::IsItemActive();
    if (hovered || active) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    if (active) {
        // Right-docked, so dragging the splitter left (negative
        // MouseDelta.x) should widen the sidebar -- subtract the delta.
        gSidebarPreferredWidth = std::clamp(gSidebarWidth - io.MouseDelta.x, kSidebarMinWidth, kSidebarMaxWidth);
    }

    // A bar always dimly visible, so the splitter can be found without
    // hovering, brightening on hover and drag.
    ImVec2 winPos = ImGui::GetWindowPos();
    ImVec2 winSize = ImGui::GetWindowSize();
    constexpr float kBarWidth = 4.0f;
    constexpr float kBarHoverAlpha = 0.85f;
    constexpr float kBarIdleAlpha = 0.5f;
    float barLeft = winPos.x + (winSize.x - kBarWidth) * 0.5f;
    ImVec4 barColor = LatticeStyle::kAccentPrimary;
    barColor.w = active ? 1.0f : (hovered ? kBarHoverAlpha : kBarIdleAlpha);
    ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(barLeft, winPos.y), ImVec2(barLeft + kBarWidth, winPos.y + winSize.y),
                                                ImGui::GetColorU32(barColor));

    ImGui::PopStyleVar(2);
    ImGui::End();
}

// A thin accent border around the 3D view (not the sidebar) while
// placement mode is on, so the next click's meaning is obvious. Drawn on
// the foreground list, above the scene.
constexpr float kPlacementBorderThickness = 3.0f;

void DrawPlacementModeIndicator(SidebarState& state) {
    if (!state.waypointPlacementMode) return;

    ImGuiIO& io = ImGui::GetIO();
    float viewWidth = io.DisplaySize.x - gSidebarWidth;
    ImGui::GetForegroundDrawList()->AddRect(ImVec2(0.0f, 0.0f), ImVec2(viewWidth, io.DisplaySize.y),
                                             ImGui::GetColorU32(LatticeStyle::kAccentPrimary), 0.0f,
                                             ImDrawFlags_None, kPlacementBorderThickness);
}

// ---- Settings ----------------------------------------------------------

// A checkbox that marks the settings for saving when it changes.
void SettingCheckbox(const char* label, bool* value, const char* tooltip = nullptr) {
    if (ImGui::Checkbox(label, value)) ViewSettingsStore::MarkChanged();
    if (tooltip != nullptr && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tooltip);
}

// A centred modal with a close X; Esc closes it too. While it is open
// ImGui reports WantCaptureKeyboard, so the sim shortcuts are inert.
void DrawSettingsWindow() {
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    bool open = true;
    if (!ImGui::BeginPopupModal("Settings", &open,
                                ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings)) {
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, /*repeat=*/false)) ImGui::CloseCurrentPopup();
    ViewSettings& settings = ViewSettingsStore::Get();

    LatticeStyle::TextBold("Display");
    SettingCheckbox("Trails", &settings.showTrails);
    ImGui::Indent();
    ImGui::BeginDisabled(!settings.showTrails);
    SettingCheckbox("Full trail history", &settings.showFullTrail,
                    "Every recorded position, not just the last 30 seconds.");
    SettingCheckbox("Event marks", &settings.showEventMarks,
                    "Ticks on the trails where radar, RWR, launch and seeker events happened,\n"
                    "on the part of the trail shown. Hover one for what and when.");
    ImGui::EndDisabled();
    ImGui::Unindent();
    SettingCheckbox("Radar cones", &settings.showRadarCones);
    SettingCheckbox("Seeker cones", &settings.showSeekerCones,
                    "Each missile's seeker cone while it flies, out to its acquisition range.");
    ImGui::BeginDisabled(!settings.showRadarCones && !settings.showSeekerCones);
    ImGui::SetNextItemWidth(LatticeStyle::kFieldWidth);
    // AlwaysClamp: a Ctrl+click typed value would otherwise pass the max.
    if (ImGui::SliderFloat("Cone opacity", &settings.coneOpacity, 0.0f, ViewSettings::kMaxConeOpacity, "%.2f",
                           ImGuiSliderFlags_AlwaysClamp)) {
        ViewSettingsStore::MarkChanged();
    }
    ImGui::EndDisabled();
    SettingCheckbox("Waypoint routes", &settings.showWaypointRoutes);
    SettingCheckbox("Ground grid", &settings.showGrid);
    SettingCheckbox("Time panel", &settings.showTimeHud,
                    "The bottom panel: sim time, pause, speed and the Reset/Reload toolbar.\n"
                    "Its keys work either way.");
    SettingCheckbox("Notifications", &settings.showToasts, "The brief top-right confirmations.");

    ImGui::Spacing();
    LatticeStyle::TextBold("Panels");
    SettingCheckbox("Telemetry", &settings.showTelemetry);
    SettingCheckbox("Event log", &settings.showEventLog);

    ImGui::Spacing();
    LatticeStyle::TextBold("Mode");
    SettingCheckbox("Advanced mode", &settings.advancedMode,
                    "Also shows the finer knobs (kill radius, max range, seeker cone and\n"
                    "coast, seeker range, RCS scale), every aircraft and missile type value\n"
                    "(greyed out: they belong to the type files), and the live autopilot\n"
                    "command in the Entity Inspector.");
    ImGui::EndPopup();
}

constexpr float kGearButtonSize = 20.0f;
constexpr int kGearTeeth = 8;

// A gear, drawn: neither shipped font has the glyph.
bool GearButton(const char* id) {
    bool pressed = ImGui::InvisibleButton(id, ImVec2(kGearButtonSize, kGearButtonSize));
    ImVec2 min = ImGui::GetItemRectMin();
    ImVec2 center(min.x + kGearButtonSize * 0.5f, min.y + kGearButtonSize * 0.5f);
    ImU32 color = ImGui::GetColorU32(ImGui::IsItemHovered() ? LatticeStyle::kAccentPrimary : LatticeStyle::kTextPrimary);
    constexpr float kRingRadiusFraction = 0.3f;
    constexpr float kToothInnerFraction = 0.32f;
    constexpr float kToothOuterFraction = 0.46f;
    constexpr float kStroke = 2.0f;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddCircle(center, kGearButtonSize * kRingRadiusFraction, color, 0, kStroke);
    for (int i = 0; i < kGearTeeth; ++i) {
        float angle = 2.0f * kPiF * i / kGearTeeth;
        ImVec2 dir(std::cos(angle), std::sin(angle));
        draw->AddLine(ImVec2(center.x + dir.x * kGearButtonSize * kToothInnerFraction,
                             center.y + dir.y * kGearButtonSize * kToothInnerFraction),
                      ImVec2(center.x + dir.x * kGearButtonSize * kToothOuterFraction,
                             center.y + dir.y * kGearButtonSize * kToothOuterFraction),
                      color, kStroke + 1.0f);
    }
    return pressed;
}

// The row above the sections: Settings and the shortcut list.
void DrawTopBar() {
    if (GearButton("##settings")) ImGui::OpenPopup("Settings");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Settings");
    DrawSettingsWindow();
    ImGui::SameLine();
    if (ImGui::Button("?")) gShowShortcuts = !gShowShortcuts;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Keyboard shortcuts (?)");
}

// ---- Shortcuts ---------------------------------------------------------

struct ShortcutRow {
    const char* keys;
    const char* action;
};
constexpr ShortcutRow kShortcutRows[] = {
    {"Space", "Pause / run"},
    {"R", "Reset: restart keeping your edits, paused at t=0"},
    {"Shift+R", "Reload the file, discarding edits, paused at t=0"},
    {"1 2 3 4", "Speed 1x / 10x / 50x / 100x"},
    {"F", "Follow the selected entity (again to stop)"},
    {"Tab", "Select the next aircraft"},
    {"M", "Select and follow the missile, once launched"},
    {"W", "Waypoint Placement Mode (while paused)"},
    {"Ctrl+Z", "Drop the last waypoint (in placement mode)"},
    {"?", "Show or hide this list"},
};

// Both land paused at t=0 (ResetRunState).
void RestartRun(World& world, SimContext& simContext, SidebarState& state, bool fromFile) {
    std::string& error = gScenarioPanel.reloadError;
    if (fromFile) {
        ReloadScenario(world, simContext, state, error);
    } else {
        ResetScenario(world, simContext, state, error);
    }
    if (error.empty()) {
        ShowToast(fromFile ? "Scenario reloaded" : "Scenario reset");
    } else {
        ShowToast(fromFile ? "Reload failed -- see the Scenario section" : "Reset failed -- see the Scenario section");
    }
}

// Ctrl+Z drops the selected entity's last waypoint. Deliberately a "drop
// the last point", not a general undo stack -- it reverses placement, not
// an altitude drag or a typed coordinate.
void DropLastWaypoint(World& world, std::optional<size_t>& selectedWaypointIndex) {
    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    if (!selected.has_value()) return;
    WaypointComponent* waypoint = world.Components<WaypointComponent>().TryGet(*selected);
    if (waypoint == nullptr || waypoint->queue.empty()) return;
    WaypointSystem::TruncateQueue(*waypoint, waypoint->queue.size() - 1);
    // The removed point may be the grabbed one, or the index now past the end.
    if (selectedWaypointIndex.has_value() && *selectedWaypointIndex >= waypoint->queue.size()) {
        selectedWaypointIndex.reset();
    }
}

// The aircraft after the selected one, by entity ID, wrapping round.
void SelectNextAircraft(World& world) {
    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    std::vector<EntityID> aircraft;
    for (EntityID entity : world.Entities()) {
        if (!guidances.Has(entity) && transforms.Has(entity)) aircraft.push_back(entity);
    }
    if (aircraft.empty()) return;
    std::sort(aircraft.begin(), aircraft.end());
    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    auto next = selected.has_value() ? std::upper_bound(aircraft.begin(), aircraft.end(), *selected) : aircraft.begin();
    EntityInspector::SetSelectedEntity(next == aircraft.end() ? aircraft.front() : *next);
}

// The latest-fired missile, preferring one still flying.
std::optional<EntityID> LatestMissile(World& world) {
    std::optional<EntityID> best;
    const GuidanceComponent* bestGuidance = nullptr;
    for (auto& [entity, guidance] : world.Components<GuidanceComponent>()) {
        if (!guidance.launched) continue;
        bool better = bestGuidance == nullptr || (guidance.active && !bestGuidance->active) ||
                      (guidance.active == bestGuidance->active && guidance.launchSimTime > bestGuidance->launchSimTime);
        if (better) {
            best = entity;
            bestGuidance = &guidance;
        }
    }
    return best;
}

// Keys are ignored while typing in a field (ImGui then wants the keyboard).
void HandleShortcuts(World& world, SimContext& simContext, SidebarState& state) {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) return;
    auto Pressed = [](ImGuiKey key) { return ImGui::IsKeyPressed(key, /*repeat=*/false); };
    bool plain = !io.KeyCtrl && !io.KeyAlt && !io.KeySuper;

    if (Pressed(ImGuiKey_Space)) state.simClock.TogglePause();
    if (plain && Pressed(ImGuiKey_R)) RestartRun(world, simContext, state, /*fromFile=*/io.KeyShift);

    constexpr ImGuiKey kPresetKeys[] = {ImGuiKey_1, ImGuiKey_2, ImGuiKey_3, ImGuiKey_4};
    for (size_t i = 0; i < std::size(kPresetKeys); ++i) {
        if (plain && Pressed(kPresetKeys[i])) state.simClock.SetRunScale(kSpeedPresets[i]);
    }

    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    if (plain && Pressed(ImGuiKey_F) && selected.has_value()) {
        if (state.followedEntity == selected) {
            state.followedEntity.reset();
        } else {
            state.followedEntity = selected;
        }
    }
    if (plain && Pressed(ImGuiKey_Tab)) SelectNextAircraft(world);
    if (plain && Pressed(ImGuiKey_M)) {
        if (std::optional<EntityID> missile = LatestMissile(world)) {
            EntityInspector::SetSelectedEntity(*missile);
            state.followedEntity = missile;
        } else {
            ShowToast("No missile launched yet");
        }
    }
    if (plain && Pressed(ImGuiKey_W)) {
        if (state.simClock.Paused()) {
            state.waypointPlacementMode = !state.waypointPlacementMode;
        } else {
            ShowToast("Pause to place waypoints");
        }
    }
    // Placement is paused-only, so the undo is too.
    if (io.KeyCtrl && Pressed(ImGuiKey_Z) && state.waypointPlacementMode) {
        DropLastWaypoint(world, state.selectedWaypointIndex);
    }
    if (plain && io.KeyShift && Pressed(ImGuiKey_Slash)) gShowShortcuts = !gShowShortcuts;
    if (Pressed(ImGuiKey_Escape)) gShowShortcuts = false;
}

// The shortcut list, centred on the 3D view.
void DrawShortcutsOverlay() {
    if (!gShowShortcuts) return;
    ImGuiIO& io = ImGui::GetIO();
    float viewWidth = io.DisplaySize.x - gSidebarWidth;
    ImGui::SetNextWindowPos(ImVec2(viewWidth * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Keyboard shortcuts", &gShowShortcuts,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
    if (ImGui::BeginTable("Shortcuts", 2, ImGuiTableFlags_SizingFixedFit)) {
        for (const ShortcutRow& row : kShortcutRows) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            {
                LatticeStyle::ScopedDataFont dataFont;
                LatticeStyle::TextBoldColored(LatticeStyle::kAccentPrimary, "%s", row.keys);
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(row.action);
        }
        ImGui::EndTable();
    }
    ImGui::TextDisabled("Keys do nothing while typing in a field. Esc closes this.");
    ImGui::End();
}

}  // namespace

namespace Sidebar {

void Draw(World& world, SimContext& simContext, SidebarState& state) {
    ImGuiIO& io = ImGui::GetIO();

    // In a narrow window the sidebar gives up width, down to its minimum,
    // so the 3D view still holds the time panel. MinWindowWidth() covers
    // the rest.
    float roomForSidebar = io.DisplaySize.x - kSplitterWidth - kTimeHudWidth;
    gSidebarWidth = std::max(kSidebarMinWidth, std::min(gSidebarPreferredWidth, roomForSidebar));

    DrawSidebarSplitter();

    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - gSidebarWidth, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(gSidebarWidth, io.DisplaySize.y), ImGuiCond_Always);

    HandleShortcuts(world, simContext, state);

    ImGui::Begin("Sidebar", nullptr,
                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoTitleBar);

    DrawTopBar();
    DrawSimulationSection(state);
    DrawScenarioSection(world, simContext, state);
    DrawParametersSection(world, state);
    ApplySetupEdits(world, simContext, state);
    DrawTelemetrySection(world, state);
    DrawEventLogSection(simContext);
    DrawEntityInspectorSection(world, simContext);
    DrawAboutSection();

    ImGui::End();

    DrawTimeHud(world, simContext, state);
    DrawGridScaleHud(state);
    DrawToast();
    DrawWaypointEditPanel(world, state);
    DrawPlacementModeIndicator(state);
    DrawShortcutsOverlay();
}

float Width() { return gSidebarWidth; }

float MinWindowWidth() { return kSidebarMinWidth + kSplitterWidth + kTimeHudWidth; }

}  // namespace Sidebar
