#include "EntityInspector.h"

#include <cmath>
#include <string>

#include "imgui.h"

#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/AutopilotComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/JSBSimComponent.h"
#include "components/KinematicsComponent.h"
#include "components/RwrComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "render/RenderSystem.h"
#include "render/style/LatticeStyle.h"
#include "util/Units.h"

namespace {

// This panel's own UI state. 0 is never a valid EntityID, so it means
// "nothing selected".
EntityID gSelectedEntity = 0;

// Narrower than ImGui's default item width, so long labels fit the
// sidebar instead of clipping at its edge.
using LatticeStyle::kFieldWidth;

// Upper end of the autopilot speed-command drag. Comfortably above either
// airframe's cruise, well short of anything the trim could hold.
constexpr float kMaxCommandSpeedKts = 600.0f;
constexpr float kMaxCommandAltitudeFt = 100000.0f;  // the Placement altitude range

constexpr float kAltitudeDragStepFt = 10.0f;

// Tall enough for a handful of rows before the list scrolls.
constexpr float kEntityListHeight = 100.0f;

// Live state the scenario schema has no key for: telemetry, and commands
// that only make sense while something is flying. Scenario parameters are
// ParameterPanel's (see the header). JSBSimComponent's state lives inside
// FGFDMExec, so it is shown as presence only. The autopilot command is an
// Advanced-mode control.
void DrawLiveFields(World& world, const SimContext& simContext, EntityID entity, bool advancedMode) {
    // A destroyed aircraft keeps its last values in every row below; say
    // so up front. (A missile reports its end under Guidance instead.)
    if (world.Components<DestroyedComponent>().Has(entity) && !world.Components<GuidanceComponent>().Has(entity)) {
        LatticeStyle::TextBoldColored(LatticeStyle::kTextSecondary, "Status:");
        ImGui::SameLine();
        ImGui::TextColored(LatticeStyle::kTextSecondary, "Destroyed (rows show its last state)");
    }

    ComponentStore<Transform>& transforms = world.Components<Transform>();
    if (transforms.Has(entity)) {
        const Transform& transform = transforms.Get(entity);
        LatticeStyle::TextBold("Attitude");
        ImGui::Text("Roll %.1f  Pitch %.1f  Yaw %.1f (deg)", transform.euler.x(), transform.euler.y(),
                    transform.euler.z());
    }

    ComponentStore<KinematicsComponent>& kinematics = world.Components<KinematicsComponent>();
    if (kinematics.Has(entity)) {
        const KinematicsComponent& kin = kinematics.Get(entity);
        LatticeStyle::TextBold("Kinematics");
        LatticeStyle::TextBold("  Velocity (ft/s):");
        ImGui::SameLine();
        ImGui::Text("%.1f, %.1f, %.1f", kin.vel.x(), kin.vel.y(), kin.vel.z());
        LatticeStyle::TextBold("  Speed:");
        ImGui::SameLine();
        ImGui::Text("%.1f ft/s", kin.speed);
    }

    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    if (guidances.Has(entity)) {
        const GuidanceComponent& guidance = guidances.Get(entity);
        LatticeStyle::TextBold("Guidance");
        LatticeStyle::TextBold("  Target:");
        ImGui::SameLine();
        ImGui::Text("entity %u", guidance.target);
        LatticeStyle::TextBold("  Launched:");
        ImGui::SameLine();
        ImGui::Text("%s", guidance.launched ? "yes" : "no");
        ImGui::SameLine();
        LatticeStyle::TextBold("  Active:");
        ImGui::SameLine();
        ImGui::Text("%s", guidance.active ? "yes" : "no");

        // Nav gain is edited in Parameters (Missile / Proportional Navigation),
        // where the saver persists it and intercept_batch can sweep it.

        // The guidance phase, read-only: for a MIDCOURSE_ARH shot, whether it still
        // depends on the launcher's track or is autonomous.
        const char* modeLabel = guidance.seekerMode == SeekerMode::ARH    ? "ARH"
                                 : guidance.seekerMode == SeekerMode::SARH ? "SARH"
                                                                            : "Midcourse ARH";
        LatticeStyle::TextBold("  Seeker Mode:");
        ImGui::SameLine();
        ImGui::Text("%s", modeLabel);
        // Once the engagement has resolved, the phase and lock fields only
        // hold their last values; show what became of the missile instead.
        // Muted grey, the NONE sensor-tier color: a resolved missile has no
        // guidance tier.
        if (guidance.launched && !guidance.active) {
            bool destroyed = world.Components<DestroyedComponent>().Has(entity);
            LatticeStyle::TextBoldColored(LatticeStyle::kTextSecondary, "  Status:");
            ImGui::SameLine();
            ImGui::TextColored(LatticeStyle::kTextSecondary, "%s",
                               destroyed ? "Destroyed" : "Ended (coasting, no guidance)");
        } else if (guidance.launched && guidance.seekerMode == SeekerMode::MIDCOURSE_ARH) {
            // On the rail there is no phase yet; "Has Lock: not launched"
            // below says so.
            if (guidance.seekerActive) {
                LatticeStyle::TextBoldColored(LatticeStyle::kFriendly, "  Phase:");
                ImGui::SameLine();
                ImGui::TextColored(LatticeStyle::kFriendly, "Active (autonomous)");
            } else {
                LatticeStyle::TextBoldColored(LatticeStyle::kCaution, "  Phase:");
                ImGui::SameLine();
                ImGui::TextColored(LatticeStyle::kCaution, "Midcourse (needs launcher track)");
            }
        }
        if (guidance.active || !guidance.launched) {
            LatticeStyle::TextBold("  Has Lock:");
            ImGui::SameLine();
            // hasLock defaults true for the guidance law; on the rail there is no lock
            // to report yet.
            ImGui::Text("%s", !guidance.launched ? "not launched" : guidance.hasLock ? "yes" : "no");
        }
        // The loft, once fired: the climb angle the launch range chose, and
        // whether it is still being flown.
        if (guidance.launched && guidance.active) {
            LatticeStyle::TextBold("  Loft:");
            ImGui::SameLine();
            if (guidance.loftClimbDeg <= 0.0) {
                ImGui::Text("none");
            } else if (guidance.lofting) {
                ImGui::Text("climbing at %.0f deg", guidance.loftClimbDeg);
            } else {
                ImGui::Text("done (%.0f deg climb)", guidance.loftClimbDeg);
            }
        }
    }

    // Autopilot setpoints are a live command with no schema key (the loader
    // seeds them from the jsbsim block's initial conditions), so they are
    // edited here rather than through ParamRegistry.
    ComponentStore<AutopilotComponent>& autopilots = world.Components<AutopilotComponent>();
    if (advancedMode && autopilots.Has(entity)) {
        AutopilotComponent& autopilot = autopilots.Get(entity);
        LatticeStyle::TextBold("Autopilot command");
        ImGui::SetNextItemWidth(kFieldWidth);
        float targetHeading = static_cast<float>(autopilot.targetHeadingDeg);
        // AlwaysClamp on all three: a Ctrl+click typed value would otherwise pass the range.
        if (ImGui::DragFloat("Target Heading (deg)", &targetHeading, 1.0f, 0.0f, 360.0f, "%.3f",
                             ImGuiSliderFlags_AlwaysClamp))
            autopilot.targetHeadingDeg = targetHeading;
        ImGui::SetNextItemWidth(kFieldWidth);
        float targetAltitude = static_cast<float>(autopilot.targetAltitudeFt);
        if (ImGui::DragFloat("Target Altitude (ft AGL)", &targetAltitude, kAltitudeDragStepFt, 0.0f,
                             kMaxCommandAltitudeFt, "%.3f", ImGuiSliderFlags_AlwaysClamp))
            autopilot.targetAltitudeFt = targetAltitude;
        ImGui::SetNextItemWidth(kFieldWidth);
        float targetSpeed = static_cast<float>(autopilot.targetSpeedKts);
        if (ImGui::DragFloat("Target Speed (kts)", &targetSpeed, 1.0f, 0.0f, kMaxCommandSpeedKts, "%.3f",
                             ImGuiSliderFlags_AlwaysClamp))
            autopilot.targetSpeedKts = targetSpeed;

        // TRUE airspeed, what the throttle loop compares against the command;
        // calibrated would show a ~125 kt error that is not there. A disabled
        // DragFloat, to match the editable rows above.
        ComponentStore<JSBSimComponent>& jsbsimForSpeed = world.Components<JSBSimComponent>();
        if (jsbsimForSpeed.Has(entity)) {
            double actualSpeedKts = jsbsimForSpeed.Get(entity).props.vtKts->getDoubleValue();
            float actualSpeed = static_cast<float>(actualSpeedKts);
            ImGui::SetNextItemWidth(kFieldWidth);
            ImGui::BeginDisabled();
            ImGui::DragFloat("Actual Speed (kts)", &actualSpeed);
            ImGui::EndDisabled();
        }
    }

    if (world.Components<JSBSimComponent>().Has(entity)) {
        LatticeStyle::TextBold("JSBSim Flight Model:");
        ImGui::SameLine();
        ImGui::Text("present");
    }

    ComponentStore<AirborneRadarComponent>& airborneRadarComponents = world.Components<AirborneRadarComponent>();
    if (airborneRadarComponents.Has(entity)) {
        const AirborneRadarComponent& airborneRadar = airborneRadarComponents.Get(entity);
        LatticeStyle::TextBold("Airborne Radar");
        // The cone's own tier and colour, in words.
        RadarTrackMode tier = RenderSystem::RadarConeTier(simContext, entity, airborneRadar);
        ImVec4 color = LatticeStyle::RadarTrackModeColor(tier);
        LatticeStyle::TextBoldColored(color, "  Mode:");
        ImGui::SameLine();
        switch (tier) {
            case RadarTrackMode::FIRING_SOLUTION:
                ImGui::TextColored(color, "STT lock on %u", airborneRadar.sttTarget);
                break;
            case RadarTrackMode::TRACK:
                ImGui::TextColored(color, "Scanning, confirmed track");
                break;
            case RadarTrackMode::SEARCH:
                ImGui::TextColored(color, "Scanning, tentative track");
                break;
            case RadarTrackMode::NONE:
                ImGui::TextColored(color, "Scanning, no track");
                break;
        }
    }

    // Coloured by the shared tier ramp (LatticeStyle), like the cones.
    ComponentStore<RwrComponent>& rwrComponents = world.Components<RwrComponent>();
    if (rwrComponents.Has(entity)) {
        const RwrComponent& rwr = rwrComponents.Get(entity);
        LatticeStyle::TextBold("Radar Warning Receiver");

        // A lock is the top of the ladder, the colour a firing solution was.
        const char* modeName = "None";
        RadarTrackMode tier = RadarTrackMode::NONE;
        switch (rwr.perceivedMode) {
            case RwrMode::NONE:
                break;
            case RwrMode::SEARCH:
                modeName = "Search";
                tier = RadarTrackMode::SEARCH;
                break;
            case RwrMode::LOCK:
                modeName = "Lock";
                tier = RadarTrackMode::FIRING_SOLUTION;
                break;
        }
        ImVec4 color = LatticeStyle::RadarTrackModeColor(tier);
        LatticeStyle::TextBoldColored(color, "  Perceived Mode:");
        ImGui::SameLine();
        ImGui::TextColored(color, "%s", modeName);
        LatticeStyle::TextBold("  Currently Illuminated:");
        ImGui::SameLine();
        ImGui::Text("%s", rwr.currentlyIlluminated ? "yes" : "no");
        // A missile warning outranks every radar mode: the lock's colour.
        ImVec4 missileColor = rwr.missileWarning ? LatticeStyle::RadarTrackModeColor(RadarTrackMode::FIRING_SOLUTION)
                                                 : LatticeStyle::RadarTrackModeColor(RadarTrackMode::NONE);
        LatticeStyle::TextBoldColored(missileColor, "  Missile:");
        ImGui::SameLine();
        ImGui::TextColored(missileColor, "%s", rwr.missileWarning ? "Active" : "None");
    }
}

}  // namespace

namespace EntityInspector {

void Draw(World& world, const SimContext& simContext, bool advancedMode) {
    ComponentStore<Tag>& tags = world.Components<Tag>();
    ComponentStore<GuidanceComponent>& guidances = world.Components<GuidanceComponent>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();

    ImGui::BeginChild("EntityList", ImVec2(0.0f, kEntityListHeight), true);
    for (EntityID entity : world.Entities()) {
        std::string label = std::to_string(entity) + "  " + (tags.Has(entity) ? tags.Get(entity).name : "(untagged)");

        // Greyed: a missile before launch or after it resolves, or a destroyed
        // aircraft -- nothing live to track, but still inspectable.
        bool inactive = (guidances.Has(entity) && !guidances.Get(entity).active) || destroyed.Has(entity);
        if (inactive) ImGui::PushStyleColor(ImGuiCol_Text, LatticeStyle::kTextSecondary);
        // Clicking the selected row deselects; Selectable fires on every click, so
        // the toggle is done here.
        if (ImGui::Selectable(label.c_str(), entity == gSelectedEntity)) {
            gSelectedEntity = (entity == gSelectedEntity) ? 0 : entity;
        }
        if (inactive) ImGui::PopStyleColor();
    }
    ImGui::EndChild();

    if (gSelectedEntity == 0 || world.Entities().count(gSelectedEntity) == 0) {
        ImGui::TextDisabled("Select an entity above.");
        return;
    }

    ImGui::Separator();
    {
        LatticeStyle::ScopedDataFont dataFont;
        DrawLiveFields(world, simContext, gSelectedEntity, advancedMode);
    }
}

std::optional<EntityID> SelectedEntity() {
    if (gSelectedEntity == 0) return std::nullopt;
    return gSelectedEntity;
}

void SetSelectedEntity(EntityID entity) { gSelectedEntity = entity; }

void ClearSelection() { gSelectedEntity = 0; }

}  // namespace EntityInspector
