#pragma once

// What the viewer shows, chosen in the Settings window and saved
// beside the executable with ImGui's window layout (imgui.ini). Display
// only: nothing here is simulated or written to a scenario.
struct ViewSettings {
    // Fill alpha of a searching radar's cone; the other tiers scale from it
    // (LatticeStyle::RadarTrackModeConeWeight).
    static constexpr float kDefaultConeOpacity = 0.18f;
    static constexpr float kMaxConeOpacity = 0.5f;

    // Display
    bool showTrails = true;
    bool showFullTrail = false;  // every recorded sample, not the rolling window
    bool showRadarCones = true;
    // Off: several missiles can be in flight at once.
    bool showSeekerCones = false;
    bool showWaypointRoutes = true;
    bool showGrid = true;
    bool showTimeHud = true;
    bool showToasts = true;
    float coneOpacity = kDefaultConeOpacity;

    // Panels
    bool showTelemetry = true;
    bool showEventLog = true;

    // Shows the type files' read-only values, the advanced knobs
    // (ParamRegistry::ParamDef::advanced) and the live autopilot command.
    bool advancedMode = false;
};

namespace ViewSettingsStore {

ViewSettings& Get();

// Adds the [InterceptSim][Settings] section to imgui.ini. Call after
// ImGui::CreateContext() and before the first NewFrame(), which reads it.
void RegisterIniHandler();

// Call after an edit, so ImGui writes the ini on its next save.
void MarkChanged();

}  // namespace ViewSettingsStore
