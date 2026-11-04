#pragma once

#include "imgui.h"

#include "components/AirborneRadarTrack.h"

namespace LatticeStyle {

inline const ImVec4 kBackground      = ImVec4(0x0B / 255.0f, 0x0D / 255.0f, 0x10 / 255.0f, 1.0f);
inline const ImVec4 kPanelBackground = ImVec4(0x12 / 255.0f, 0x15 / 255.0f, 0x1A / 255.0f, 1.0f);
inline const ImVec4 kPanelHeader     = ImVec4(0x1A / 255.0f, 0x1F / 255.0f, 0x26 / 255.0f, 1.0f);
inline const ImVec4 kBorder          = ImVec4(0x23 / 255.0f, 0x2A / 255.0f, 0x33 / 255.0f, 1.0f);
inline const ImVec4 kAccentPrimary   = ImVec4(0x2F / 255.0f, 0xD1 / 255.0f, 0xFF / 255.0f, 1.0f);
inline const ImVec4 kAccentSecondary = ImVec4(0x4C / 255.0f, 0x8B / 255.0f, 0xF5 / 255.0f, 1.0f);
inline const ImVec4 kFriendly        = ImVec4(0x33 / 255.0f, 0xD6 / 255.0f, 0xA6 / 255.0f, 1.0f);
inline const ImVec4 kThreat          = ImVec4(0xFF / 255.0f, 0x3B / 255.0f, 0x30 / 255.0f, 1.0f);
inline const ImVec4 kCaution         = ImVec4(0xFF / 255.0f, 0xB0 / 255.0f, 0x20 / 255.0f, 1.0f);
inline const ImVec4 kTextPrimary     = ImVec4(0xE6 / 255.0f, 0xE9 / 255.0f, 0xED / 255.0f, 1.0f);
inline const ImVec4 kTextSecondary   = ImVec4(0x8A / 255.0f, 0x93 / 255.0f, 0xA3 / 255.0f, 1.0f);

// Shared numeric-field width for EntityInspector and ParameterPanel, so a
// value column lines up the same whichever panel it's read from.
inline constexpr float kFieldWidth = 110.0f;

// Sensor escalation tier -> colour, the same everywhere: the radar's FOV
// cone, the missile's seeker cone and the RWR readout. A green/amber/red
// ramp an operator reads as "benign / interested / about to shoot".
//
// NONE is deliberately NOT on the ramp: muted grey means no usable track
// at all, which on a missile cone separates a seeker still hunting from
// one that has gone ballistic.
ImVec4 RadarTrackModeColor(RadarTrackMode mode);

// An FOV cone's opacity at that tier, relative to SEARCH (1): more
// certain = more opaque, and a saturated red at the search tier's alpha
// would not read as red. Scales the viewer's cone-opacity setting.
float RadarTrackModeConeWeight(RadarTrackMode mode);

// Two fonts: a sans for UI chrome (loaded first, so ImGui's default) and
// gDataFont, a monospace face pushed around numbers so digits keep their
// columns. Both are TTFs ImGui ships (deps/imgui/misc/fonts), so the build
// downloads nothing. Populated by Apply(); null until then, or if its file
// is missing.
inline ImFont* gDataFont = nullptr;

// Pushes gDataFont for the lifetime of the scope, if it loaded.
class ScopedDataFont {
public:
    ScopedDataFont() : pushed_(gDataFont != nullptr) {
        if (pushed_) ImGui::PushFont(gDataFont);
    }
    ~ScopedDataFont() {
        if (pushed_) ImGui::PopFont();
    }
    ScopedDataFont(const ScopedDataFont&) = delete;
    ScopedDataFont& operator=(const ScopedDataFont&) = delete;

private:
    bool pushed_;
};

// Applies the dark, sharp-edged palette and loads the fonts above. Call once
// after ImGui::CreateContext() and before the OpenGL3 backend initializes
// (its font atlas upload needs the fonts added).
void Apply();

// Emphasized text for the NAME half of a "name: value" readout -- drop-in
// replacements for ImGui::Text / TextColored.
//
// Faux-bold (the string drawn again one pixel right), because neither
// shipped face has a bold weight; it also bolds whatever font is pushed.
// The extra pixel sits well inside ImGui's item spacing, so a following
// SameLine() cannot collide with it.
void TextBold(const char* fmt, ...);
void TextBoldColored(const ImVec4& color, const char* fmt, ...);

}  // namespace LatticeStyle
