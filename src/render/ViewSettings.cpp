#include "ViewSettings.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>

#include "imgui.h"
#include "imgui_internal.h"

namespace {

ViewSettings gSettings;

// One table drives both reading and writing, so a new setting cannot be
// saved under one name and read under another.
struct BoolEntry {
    const char* key;
    bool ViewSettings::*field;
};
constexpr BoolEntry kBoolEntries[] = {
    {"ShowTrails", &ViewSettings::showTrails},
    {"ShowFullTrail", &ViewSettings::showFullTrail},
    {"ShowEventMarks", &ViewSettings::showEventMarks},
    {"ShowRadarCones", &ViewSettings::showRadarCones},
    {"ShowSeekerCones", &ViewSettings::showSeekerCones},
    {"ShowWaypointRoutes", &ViewSettings::showWaypointRoutes},
    {"ShowGrid", &ViewSettings::showGrid},
    {"ShowTimeHud", &ViewSettings::showTimeHud},
    {"ShowToasts", &ViewSettings::showToasts},
    {"ShowTelemetry", &ViewSettings::showTelemetry},
    {"ShowEventLog", &ViewSettings::showEventLog},
    {"AdvancedMode", &ViewSettings::advancedMode},
};
constexpr const char* kConeOpacityKey = "ConeOpacity";

void* ReadOpen(ImGuiContext*, ImGuiSettingsHandler*, const char* name) {
    return std::strcmp(name, "Settings") == 0 ? &gSettings : nullptr;
}

// Unknown keys are ignored, so an ini from another build still loads.
void ReadLine(ImGuiContext*, ImGuiSettingsHandler*, void*, const char* line) {
    const char* equals = std::strchr(line, '=');
    if (equals == nullptr) return;
    std::string key(line, equals);
    float value = std::strtof(equals + 1, nullptr);
    for (const BoolEntry& entry : kBoolEntries) {
        if (key == entry.key) gSettings.*entry.field = value != 0.0f;
    }
    if (key == kConeOpacityKey) {
        gSettings.coneOpacity = std::clamp(value, 0.0f, ViewSettings::kMaxConeOpacity);
    }
}

void WriteAll(ImGuiContext*, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out) {
    out->appendf("[%s][Settings]\n", handler->TypeName);
    for (const BoolEntry& entry : kBoolEntries) out->appendf("%s=%d\n", entry.key, gSettings.*entry.field ? 1 : 0);
    out->appendf("%s=%.3f\n\n", kConeOpacityKey, gSettings.coneOpacity);
}

}  // namespace

namespace ViewSettingsStore {

ViewSettings& Get() { return gSettings; }

void RegisterIniHandler() {
    ImGuiSettingsHandler handler;
    handler.TypeName = "BvrSim";
    handler.TypeHash = ImHashStr(handler.TypeName);
    handler.ReadOpenFn = ReadOpen;
    handler.ReadLineFn = ReadLine;
    handler.WriteAllFn = WriteAll;
    ImGui::AddSettingsHandler(&handler);
}

void MarkChanged() { ImGui::MarkIniSettingsDirty(); }

}  // namespace ViewSettingsStore
