#include "ParameterPanel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "imgui.h"

#include "components/GuidanceComponent.h"
#include "components/JSBSimComponent.h"
#include "components/SpawnPose.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "render/style/LatticeStyle.h"
#include "scenario/ParamRegistry.h"
#include "systems/LaunchSystem.h"

namespace {

using LatticeStyle::kFieldWidth;

bool gSetupEdited = false;  // see TakeSetupEdit

// The value a "(setup)" row held when its edit began, for RevertSetupEdit.
struct SetupUndo {
    const ParamRegistry::ParamDef* param = nullptr;
    EntityID entity{};
    double value = 0.0;
};
SetupUndo gSetupUndo;

// Thresholds and gains span twenty orders of magnitude, so no single drag
// widget works for all of them. Anything below this gets a scientific
// format and a proportional drag speed instead of the entry's own step.
constexpr double kScientificBelow = 1e-4;
constexpr double kScientificFrom = 1e7;  // e.g. a 1e10 Hz frequency
constexpr double kNoDecimalsFrom = 10000.0;
constexpr double kOneDecimalFrom = 100.0;

// Below kScientificBelow a drag moves this fraction of the value per pixel.
constexpr double kProportionalDragFraction = 0.02;
constexpr double kDefaultDragStep = 0.01;  // an entry with no step of its own

constexpr float kTooltipWrapEm = 24.0f;  // tooltip width, in font heights

// Reads as "0.30 mg" rather than "0.300000", and keeps a 1.9e-16
// threshold legible.
const char* FormatFor(double value) {
    double magnitude = std::fabs(value);
    if (magnitude != 0.0 && magnitude < kScientificBelow) return "%.4e";
    if (magnitude >= kScientificFrom) return "%.4e";
    if (magnitude >= kNoDecimalsFrom) return "%.0f";
    if (magnitude >= kOneDecimalFrom) return "%.1f";
    return "%.3f";
}

// A drag speed that stays usable across the whole range: the entry's own
// step where it makes sense, otherwise a fraction of the current value so
// a 1e-16 threshold is draggable at all.
float DragSpeedFor(const ParamRegistry::ParamDef& param, double value) {
    double magnitude = std::fabs(value);
    if (magnitude != 0.0 && magnitude < kScientificBelow) {
        return static_cast<float>(magnitude * kProportionalDragFraction);
    }
    return static_cast<float>(param.dragStep != 0.0 ? param.dragStep : kDefaultDragStep);
}

// Compact display of a min/max bound. %g keeps 1e7 and 1e-20 legible
// without the trailing zeros a fixed format would show on 0.0001.
std::string FormatBound(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%g", value);
    return buffer;
}

// Whether the last item is hovered for a row's tooltip: after a ~1s dwell
// (style.HoverDelayNormal, set in LatticeStyle), each row demanding its own
// full dwell, and locked (setup, type) rows still explaining themselves.
bool RowItemHovered() {
    return ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled | ImGuiHoveredFlags_DelayNormal |
                                ImGuiHoveredFlags_NoSharedDelay);
}

// Explains a parameter: shown while its widget or its label is hovered.
void DrawParamTooltip(const ParamRegistry::ParamDef& param) {
    if (!ImGui::BeginTooltip()) return;
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * kTooltipWrapEm);

    LatticeStyle::TextBold("%s", param.label);
    if (param.description && param.description[0] != '\0') {
        ImGui::TextUnformatted(param.description);
    }

    if (param.type == ParamRegistry::ParamType::Double) {
        std::string range = "Range: " + FormatBound(param.minValue) + " to " + FormatBound(param.maxValue);
        if (param.unit && param.unit[0] != '\0') range += " " + std::string(param.unit);
        ImGui::TextColored(LatticeStyle::kTextSecondary, "%s", range.c_str());
    } else if (param.type == ParamRegistry::ParamType::Enum) {
        std::string values = "Values: ";
        for (size_t i = 0; i < param.enumAccess.names.size(); ++i) {
            if (i != 0) values += ", ";
            values += param.enumAccess.names[i];
        }
        ImGui::TextColored(LatticeStyle::kTextSecondary, "%s", values.c_str());
    }

    if (!param.knob) {
        ImGui::TextColored(LatticeStyle::kTextSecondary, "%s",
                           param.owner == ParamRegistry::ParamOwner::Doctrine
                               ? "Read-only: the missile type's value (data/missiles)."
                               : "Read-only: the aircraft type's value (data/aircraft).");
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
        return;
    }

    const char* when = "Takes effect immediately (live).";
    switch (param.timing) {
        case ParamRegistry::ParamTiming::Live:
            break;
        case ParamRegistry::ParamTiming::NextLaunch:
            when = "Takes effect at the next missile launch.";
            break;
        case ParamRegistry::ParamTiming::Setup:
            when = "Start-of-run value; editable only before the clock advances.";
            break;
    }
    ImGui::TextColored(LatticeStyle::kTextSecondary, "%s", when);

    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

// Beside the label when it fits, else under it at labelX: at the row's
// start it would read as the next row's label.
void DrawTimingNote(ParamRegistry::ParamTiming timing, float labelX) {
    const char* note = nullptr;
    switch (timing) {
        case ParamRegistry::ParamTiming::Live:
            return;
        case ParamRegistry::ParamTiming::NextLaunch:
            note = "(next launch)";
            break;
        case ParamRegistry::ParamTiming::Setup:
            note = "(setup)";
            break;
    }
    ImGui::SameLine();
    if (ImGui::GetContentRegionAvail().x < ImGui::CalcTextSize(note).x) {
        ImGui::NewLine();
        ImGui::SetCursorPosX(labelX);
    }
    ImGui::TextColored(LatticeStyle::kTextSecondary, "%s", note);
}

// An enum combo wide enough for its longest name, so none is cut off.
float ComboWidthFor(const ParamRegistry::ParamDef& param) {
    float widest = 0.0f;
    for (const char* name : param.enumAccess.names) widest = std::max(widest, ImGui::CalcTextSize(name).x);
    return std::max(kFieldWidth, widest + ImGui::GetStyle().FramePadding.x * 2.0f + ImGui::GetFrameHeight());
}

// Setup rows edit the entity's SpawnPose; mirror it onto the live
// Transform so the marker moves too. Only reachable before the clock
// starts, while the two are meant to agree.
void MoveToSpawnPose(World& world, EntityID entity) {
    const SpawnPose* spawn = world.Components<SpawnPose>().TryGet(entity);
    Transform* transform = world.Components<Transform>().TryGet(entity);
    if (!spawn || !transform) return;
    transform->pos = spawn->pos;
    transform->euler.z() = spawn->headingDeg;
}

// One row. Returns false when the subject has no such component, so the
// caller can tell an empty group from a drawn one.
bool DrawRow(const ParamRegistry::ParamDef& param, ScenarioData& scenario, World& world, EntityID entity,
             double simTime) {
    // Setup fields describe the starting state: editing one mid-run would
    // teleport the entity and be saved as its new start on the next Reset.
    // A type value is shown, not edited: only the teaching knobs are.
    bool locked = !param.knob || (param.timing == ParamRegistry::ParamTiming::Setup && simTime > 0.0);

    // The widget's own label is hidden ("##pointer", which also keeps its
    // state unique: several groups share a label like "East"); the label is
    // drawn beside it as wrapped text, so a long one wraps in its column
    // instead of clipping at the sidebar's edge.
    std::string label = std::string(param.label);
    if (param.unit && param.unit[0] != '\0') label += " (" + std::string(param.unit) + ")";
    const std::string id = "##" + std::string(param.pointer);

    if (locked) ImGui::BeginDisabled();
    bool drawn = true;
    bool changed = false;

    switch (param.type) {
        case ParamRegistry::ParamType::Double: {
            double* value = param.dbl(scenario, world, entity);
            if (!value) {
                drawn = false;
                break;
            }
            // Shown, edited and clamped in the JSON's units -- the unit the
            // label names and the [min, max] and step are declared in. The
            // value itself is held internally in feet / slugs / lbf.
            // Edited as a double: a float shows 9.86e9 Hz as 9859999744.
            double shown = param.jsonScale.ToJson(*value);
            double widget = shown;
            const double before = *value;
            ImGui::SetNextItemWidth(kFieldWidth);
            // AlwaysClamp so a Ctrl+Click manual entry can't push the value
            // outside the schema's declared [min, max] -- ImGui only clamps
            // dragging by default.
            if (ImGui::DragScalar(id.c_str(), ImGuiDataType_Double, &widget, DragSpeedFor(param, shown),
                                  &param.minValue, &param.maxValue, FormatFor(shown),
                                  ImGuiSliderFlags_AlwaysClamp)) {
                *value = param.jsonScale.ToInternal(widget);
                changed = true;
                if (param.timing == ParamRegistry::ParamTiming::Setup) MoveToSpawnPose(world, entity);
            }
            if (param.timing == ParamRegistry::ParamTiming::Setup && ImGui::IsItemActivated()) {
                gSetupUndo = SetupUndo{&param, entity, before};
            }
            // Once per finished edit, not every drag frame: the re-apply
            // rebuilds the flight models.
            if (param.timing == ParamRegistry::ParamTiming::Setup && ImGui::IsItemDeactivatedAfterEdit()) {
                gSetupEdited = true;
            }
            break;
        }
        case ParamRegistry::ParamType::Bool: {
            bool* value = param.flag(scenario, world, entity);
            if (!value) {
                drawn = false;
                break;
            }
            changed = ImGui::Checkbox(id.c_str(), value);
            break;
        }
        case ParamRegistry::ParamType::Enum: {
            int current = param.enumAccess.get(scenario, world, entity);
            if (current < 0) {
                drawn = false;
                break;
            }
            ImGui::SetNextItemWidth(ComboWidthFor(param));
            if (ImGui::BeginCombo(id.c_str(), param.enumAccess.names[static_cast<size_t>(current)])) {
                for (size_t i = 0; i < param.enumAccess.names.size(); ++i) {
                    bool selected = static_cast<int>(i) == current;
                    if (ImGui::Selectable(param.enumAccess.names[i], selected)) {
                        param.enumAccess.set(scenario, world, entity, static_cast<int>(i));
                        changed = true;
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            break;
        }
    }

    float labelX = 0.0f;
    bool hovered = false;
    if (drawn) {
        hovered = RowItemHovered();
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        labelX = ImGui::GetCursorPosX();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextUnformatted(label.c_str());
        ImGui::PopTextWrapPos();
        hovered |= RowItemHovered();
    }
    if (locked) ImGui::EndDisabled();
    // The missile was pre-created at load with the doctrine of the time;
    // until it fires, a "(next launch)" edit must reach it too.
    if (changed && param.timing == ParamRegistry::ParamTiming::NextLaunch) LaunchSystem::RefreshUnlaunched(world, scenario);
    if (drawn) {
        if (hovered) DrawParamTooltip(param);
        if (param.knob) DrawTimingNote(param.timing, labelX);
        // A SARH missile needs the launcher's lock, so it fires from STT
        // whatever this row says (UsesStt); say so rather than show a
        // setting that has no effect.
        if (std::string(param.pointer) == "/missile_doctrine/launch_mode" &&
            scenario.missileSeekerMode == SeekerMode::SARH) {
            ImGui::TextColored(LatticeStyle::kTextSecondary, "SARH: always fires from STT");
        }
    }
    return drawn;
}

// Draws every entry matching `owner` (and, for the entity owner, the
// given subject), grouped under its ParamDef::group heading. Group
// headings are only emitted once a group turns out to have a visible row,
// so an entity without a radar never shows an empty "Airborne radar".
bool DrawByOwner(ParamRegistry::ParamOwner owner, ScenarioData& scenario, World& world, EntityID entity,
                 double simTime, bool advancedMode) {
    const char* currentGroup = nullptr;
    bool groupHeaderDrawn = false;
    bool anyDrawn = false;

    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        if (param.owner != owner) continue;
        if (!advancedMode && (!param.knob || param.advanced)) continue;
        if (currentGroup == nullptr || std::string(currentGroup) != param.group) {
            currentGroup = param.group;
            groupHeaderDrawn = false;
        }

        // Probe before committing a heading: a row that has no component
        // to point at must not leave a bare group title behind.
        bool hasValue = false;
        switch (param.type) {
            case ParamRegistry::ParamType::Double:
                hasValue = param.dbl(scenario, world, entity) != nullptr;
                break;
            case ParamRegistry::ParamType::Bool:
                hasValue = param.flag(scenario, world, entity) != nullptr;
                break;
            case ParamRegistry::ParamType::Enum:
                hasValue = param.enumAccess.get(scenario, world, entity) >= 0;
                break;
        }
        if (!hasValue) continue;

        if (!groupHeaderDrawn) {
            if (anyDrawn) ImGui::Spacing();
            LatticeStyle::TextBold("%s", param.group);
            groupHeaderDrawn = true;
        }
        if (DrawRow(param, scenario, world, entity, simTime)) anyDrawn = true;
    }
    return anyDrawn;
}

}  // namespace

namespace ParameterPanel {

bool DrawForEntity(ScenarioData& scenario, World& world, EntityID entity, double simTime, bool advancedMode) {
    // Doctrine describes the missile, so it is shown under the missile though
    // stored scenario-wide. GuidanceComponent, not the tag, is what makes an
    // entity a missile.
    bool isMissile = world.Components<GuidanceComponent>().Has(entity);

    bool drew = false;
    if (isMissile) {
        drew |= DrawByOwner(ParamRegistry::ParamOwner::Doctrine, scenario, world, entity, simTime, advancedMode);
    }
    // Same reasoning for the environment: stored once, shown under the
    // aircraft it acts on.
    if (world.Components<JSBSimComponent>().Has(entity)) {
        drew |= DrawByOwner(ParamRegistry::ParamOwner::Environment, scenario, world, entity, simTime, advancedMode);
    }
    drew |= DrawByOwner(ParamRegistry::ParamOwner::Entity, scenario, world, entity, simTime, advancedMode);
    return drew;
}

bool TakeSetupEdit() {
    bool edited = gSetupEdited;
    gSetupEdited = false;
    return edited;
}

void RevertSetupEdit(ScenarioData& scenario, World& world) {
    if (gSetupUndo.param == nullptr) return;
    if (double* value = gSetupUndo.param->dbl(scenario, world, gSetupUndo.entity)) {
        *value = gSetupUndo.value;
        MoveToSpawnPose(world, gSetupUndo.entity);
    }
    gSetupUndo = SetupUndo{};
}

}  // namespace ParameterPanel
