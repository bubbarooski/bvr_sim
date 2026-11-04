#include "LatticeStyle.h"

#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <system_error>

namespace LatticeStyle {

namespace {

constexpr float kLabelFontPx = 16.0f;
constexpr float kDataFontPx = 15.0f;

// The shared half of TextBold/TextBoldColored. The cursor is read before
// the text is emitted: that is where ImGui draws it.
void DrawBold(const char* text, const ImVec4* color) {
    ImVec2 pos = ImGui::GetCursorScreenPos();
    if (color) {
        ImGui::TextColored(*color, "%s", text);
    } else {
        ImGui::TextUnformatted(text);
    }
    ImU32 packed = color ? ImGui::GetColorU32(*color) : ImGui::GetColorU32(ImGuiCol_Text);
    ImGui::GetWindowDrawList()->AddText(ImVec2(pos.x + 1.0f, pos.y), packed, text);
}

}  // namespace

ImVec4 RadarTrackModeColor(RadarTrackMode mode) {
    switch (mode) {
        case RadarTrackMode::SEARCH:
            return kFriendly;
        case RadarTrackMode::TRACK:
            return kCaution;
        case RadarTrackMode::FIRING_SOLUTION:
            return kThreat;
        case RadarTrackMode::NONE:
            break;
    }
    return kTextSecondary;
}

float RadarTrackModeConeWeight(RadarTrackMode mode) {
    switch (mode) {
        case RadarTrackMode::SEARCH:
            return 1.0f;
        case RadarTrackMode::TRACK:
            return 1.6f;
        case RadarTrackMode::FIRING_SOLUTION:
            return 2.25f;
        case RadarTrackMode::NONE:
            break;
    }
    return 0.65f;  // recedes -- no track, so the cone is reference geometry only
}

void TextBold(const char* fmt, ...) {
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    DrawBold(buffer, nullptr);
}

void TextBoldColored(const ImVec4& color, const char* fmt, ...) {
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    DrawBold(buffer, &color);
}

void Apply() {
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowRounding = 0.0f;
    style.FrameRounding = 0.0f;
    style.GrabRounding = 0.0f;
    style.TabRounding = 0.0f;
    style.ChildRounding = 0.0f;
    style.PopupRounding = 0.0f;
    style.ScrollbarRounding = 0.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;

    // Parameter tooltips (ParameterPanel) wait about a second before
    // explaining themselves; only they read this delay.
    style.HoverDelayNormal = 0.9f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]        = kPanelBackground;
    colors[ImGuiCol_ChildBg]         = kPanelBackground;
    colors[ImGuiCol_PopupBg]         = kPanelBackground;
    colors[ImGuiCol_Border]          = kBorder;
    colors[ImGuiCol_FrameBg]         = kPanelHeader;
    colors[ImGuiCol_FrameBgHovered]  = kPanelHeader;
    colors[ImGuiCol_FrameBgActive]   = kPanelHeader;
    colors[ImGuiCol_TitleBg]         = kPanelHeader;
    colors[ImGuiCol_TitleBgActive]   = kPanelHeader;
    colors[ImGuiCol_TitleBgCollapsed]= kPanelHeader;
    colors[ImGuiCol_Header]          = kPanelHeader;
    colors[ImGuiCol_HeaderHovered]   = kPanelHeader;
    colors[ImGuiCol_HeaderActive]    = kPanelHeader;
    colors[ImGuiCol_CheckMark]       = kAccentPrimary;
    colors[ImGuiCol_SliderGrab]      = kAccentPrimary;
    colors[ImGuiCol_SliderGrabActive]= kAccentPrimary;
    colors[ImGuiCol_Button]          = kPanelHeader;
    colors[ImGuiCol_ButtonHovered]   = kPanelHeader;
    colors[ImGuiCol_ButtonActive]    = kAccentPrimary;
    colors[ImGuiCol_Text]            = kTextPrimary;
    colors[ImGuiCol_TextDisabled]    = kTextSecondary;
    colors[ImGuiCol_ScrollbarBg]          = kPanelBackground;
    colors[ImGuiCol_ScrollbarGrab]        = kBorder;
    colors[ImGuiCol_ScrollbarGrabHovered] = kTextSecondary;
    colors[ImGuiCol_ScrollbarGrabActive]  = kAccentPrimary;
    colors[ImGuiCol_Separator]            = kBorder;
    colors[ImGuiCol_SeparatorHovered]     = kAccentPrimary;
    colors[ImGuiCol_SeparatorActive]      = kAccentPrimary;
    colors[ImGuiCol_ResizeGrip]           = kBorder;
    colors[ImGuiCol_ResizeGripHovered]    = kAccentPrimary;
    colors[ImGuiCol_ResizeGripActive]     = kAccentPrimary;
    colors[ImGuiCol_TextSelectedBg]       = ImVec4(kAccentPrimary.x, kAccentPrimary.y, kAccentPrimary.z, 0.35f);

    // "fonts/" is copied beside the executable at build time, and the path is
    // relative to the working directory. A missing font falls back to ImGui's
    // default -- checked first, since AddFontFromFileTTF asserts in Debug.
    ImGuiIO& io = ImGui::GetIO();
    auto AddFontIfPresent = [&io](const char* path, float sizePx) -> ImFont* {
        std::error_code ec;
        return std::filesystem::exists(path, ec) ? io.Fonts->AddFontFromFileTTF(path, sizePx) : nullptr;
    };
    AddFontIfPresent("fonts/Roboto-Medium.ttf", kLabelFontPx);  // first added = the default font
    gDataFont = AddFontIfPresent("fonts/Cousine-Regular.ttf", kDataFontPx);
}

}  // namespace LatticeStyle
