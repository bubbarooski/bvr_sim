#include "RenderSystem.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

// GLFW/gl3w pull in windows.h, whose min/max macros break std::min/max/clamp;
// must be defined before it is first included.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <GL/gl3w.h>

#include "components/AirborneRadarComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "components/WaypointComponent.h"
#include "ecs/ComponentStore.h"
#include "render/EntityInspector.h"
#include "render/Mat4.h"
#include "render/style/LatticeStyle.h"
#include "sim/RadarEquation.h"
#include "util/Units.h"

namespace {

// ---- Shaders --------------------------------------------------------------

const char* kVertexShaderSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 uViewProj;
void main() {
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

const char* kFragmentShaderSrc = R"(
#version 330 core
out vec4 FragColor;
uniform vec4 uColor;
void main() {
    FragColor = uColor;
}
)";

// Procedural ground grid on one large quad: the fragment shader derives
// grid lines from world-space XZ (fwidth-based, so they stay ~1px at any
// zoom) and fades them with distance. A finite vertex grid would
// eventually show its edge once the camera can pan freely.
const char* kGridVertexShaderSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 uViewProj;
out vec3 vWorldPos;
void main() {
    vWorldPos = aPos;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

const char* kGridFragmentShaderSrc = R"(
#version 330 core
in vec3 vWorldPos;
out vec4 FragColor;
uniform vec3 uCameraPos;
uniform vec3 uGridColor;
uniform float uSpacing;
uniform float uFadeStart;
uniform float uFadeEnd;
void main() {
    vec2 coord = vWorldPos.xz / uSpacing;
    vec2 deriv = fwidth(coord);
    vec2 gridUV = abs(fract(coord - 0.5) - 0.5) / max(deriv, vec2(0.0001));
    float line = min(gridUV.x, gridUV.y);
    float lineAlpha = 1.0 - clamp(line, 0.0, 1.0);

    float dist = distance(vWorldPos.xz, uCameraPos.xz);
    float fade = 1.0 - smoothstep(uFadeStart, uFadeEnd, dist);

    float alpha = lineAlpha * fade * 0.5;
    if (alpha < 0.01) discard;
    FragColor = vec4(uGridColor, alpha);
}
)";

// Entity trails: a ribbon of an entity's recent positions in its marker
// colour. The per-vertex alpha is constant today (kTrailAlpha), kept as an
// attribute so the ribbon layout is (pos, alpha).
const char* kTrailVertexShaderSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in float aAlpha;
uniform mat4 uViewProj;
out float vAlpha;
void main() {
    gl_Position = uViewProj * vec4(aPos, 1.0);
    vAlpha = aAlpha;
}
)";

const char* kTrailFragmentShaderSrc = R"(
#version 330 core
in float vAlpha;
out vec4 FragColor;
uniform vec3 uColor;
void main() {
    FragColor = vec4(uColor, vAlpha);
}
)";

// GLSL errors surface only at runtime, here: the window opens and ImGui
// draws, but the 3D scene never appears. Check stderr.
GLuint CompileShader(GLenum type, const char* src) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
        std::fprintf(stderr, "RenderSystem: shader compile error: %s\n", infoLog);
    }
    return shader;
}

GLuint LinkProgram(GLuint vertexShader, GLuint fragmentShader) {
    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(program, sizeof(infoLog), nullptr, infoLog);
        std::fprintf(stderr, "RenderSystem: shader link error: %s\n", infoLog);
    }
    return program;
}

GLuint gProgram = 0;
GLint gViewProjLoc = -1;
GLint gColorLoc = -1;

using RenderSystem::kMarkerScreenLengthPx;

// Entity markers are small planform icons oriented by full attitude, not
// just heading, sized to a constant on-screen length recomputed per entity
// per frame -- a fixed world size either vanishes at default zoom or
// swallows the screen up close. Each shape is triangles in body axes (+X
// right, +Y up, +Z forward), unit length, centred on the entity; each
// triangle is extruded into a thin slab along its own axis, since a flat
// outline vanishes to a line edge-on.
struct ShapePoint {
    float x, y, z;
};
struct ShapeTri {
    ShapePoint a, b, c;
};

// The aircraft: our own top-view outline in the F-16's proportions --
// pointed nose, cropped-delta wing with root extensions, tailplanes -- and
// a small fin. Right half listed; the planform is mirrored.
constexpr ShapeTri kAircraftPlanformRight[] = {
    {{0.0f, 0.0f, 0.5f}, {0.05f, 0.0f, 0.25f}, {0.0f, 0.0f, 0.25f}},          // nose
    {{0.0f, 0.0f, 0.25f}, {0.05f, 0.0f, 0.25f}, {0.05f, 0.0f, -0.5f}},        // fuselage
    {{0.0f, 0.0f, 0.25f}, {0.05f, 0.0f, -0.5f}, {0.0f, 0.0f, -0.5f}},
    {{0.05f, 0.0f, -0.22f}, {0.05f, 0.0f, 0.22f}, {0.1f, 0.0f, 0.0f}},        // root extension
    {{0.05f, 0.0f, -0.22f}, {0.1f, 0.0f, 0.0f}, {0.315f, 0.0f, -0.17f}},      // wing
    {{0.05f, 0.0f, -0.22f}, {0.315f, 0.0f, -0.17f}, {0.315f, 0.0f, -0.23f}},
    {{0.05f, 0.0f, -0.33f}, {0.186f, 0.0f, -0.43f}, {0.186f, 0.0f, -0.48f}},  // tailplane
    {{0.05f, 0.0f, -0.33f}, {0.186f, 0.0f, -0.48f}, {0.05f, 0.0f, -0.47f}},
};
constexpr ShapeTri kAircraftFin[] = {
    {{0.0f, 0.0f, -0.2f}, {0.0f, 0.2f, -0.4f}, {0.0f, 0.2f, -0.47f}},
    {{0.0f, 0.0f, -0.2f}, {0.0f, 0.2f, -0.47f}, {0.0f, 0.0f, -0.5f}},
};
constexpr float kAircraftHalfThickness = 0.025f;

// The missile: a slim body and cruciform tail fins (right and top listed;
// mirrored to left and bottom).
constexpr ShapeTri kMissileBodyRight[] = {
    {{0.0f, 0.0f, 0.5f}, {0.06f, 0.0f, 0.38f}, {0.0f, 0.0f, 0.38f}},
    {{0.0f, 0.0f, 0.38f}, {0.06f, 0.0f, 0.38f}, {0.06f, 0.0f, -0.5f}},
    {{0.0f, 0.0f, 0.38f}, {0.06f, 0.0f, -0.5f}, {0.0f, 0.0f, -0.5f}},
    {{0.06f, 0.0f, -0.3f}, {0.17f, 0.0f, -0.5f}, {0.06f, 0.0f, -0.5f}},  // fin
};
constexpr ShapeTri kMissileFinTop[] = {
    {{0.0f, 0.06f, -0.3f}, {0.0f, 0.17f, -0.5f}, {0.0f, 0.06f, -0.5f}},
};
constexpr float kMissileHalfThickness = 0.03f;
// Shorter than the aircraft, so it reads as a missile beside one.
constexpr float kMissileLengthScale = 0.75f;

// Below these (ft, or a unit-vector norm) a direction is degenerate and a
// fixed fallback is used.
constexpr float kMinSegmentFt = 0.01f;
constexpr float kMinCrossNorm = 0.001f;

// Waypoint diamond markers -- small, so a route point reads as secondary
// reference geometry, not competing with the entities actually flying it.
constexpr float kWaypointMarkerScale = 0.3f;

// Rotates a body-local point (+X right, +Y up, +Z forward) through roll,
// then pitch, then yaw, and places it at worldPos -- the standard
// aerospace body-to-world composition order, matching the flight model's
// own phi/theta/psi convention.
Vec3f RotateBodyToWorld(Vec3f local, float rollDeg, float pitchDeg, float yawDeg, Vec3f worldPos) {
    float rollRad = rollDeg * kDegToRadF;
    float pitchRad = pitchDeg * kDegToRadF;
    float yawRad = yawDeg * kDegToRadF;

    float cr = std::cos(rollRad), sr = std::sin(rollRad);
    float x1 = local.x() * cr + local.y() * sr;
    float y1 = -local.x() * sr + local.y() * cr;
    float z1 = local.z();

    float cp = std::cos(pitchRad), sp = std::sin(pitchRad);
    float y2 = y1 * cp + z1 * sp;
    float z2 = -y1 * sp + z1 * cp;
    float x2 = x1;

    float cy = std::cos(yawRad), sy = std::sin(yawRad);
    float worldX = x2 * cy + z2 * sy;
    float worldZ = -x2 * sy + z2 * cy;

    return {worldX + worldPos.x(), y2 + worldPos.y(), worldZ + worldPos.z()};
}

// A shape's placement this frame: its length and attitude at worldPos.
struct ShapePose {
    float lengthFt;
    float rollDeg, pitchDeg, yawDeg;
    Vec3f worldPos;
};

// Appends one shape triangle as a slab: two caps and three side quads (8
// triangles), offset +-halfThickness of the length along `axis`, and
// mirrored by `mirror` (componentwise sign). No backface culling anywhere,
// so winding does not matter; the hidden inner walls of adjacent slabs are
// harmless, the marker is opaque.
void AppendShapeSlab(std::vector<float>& out, const ShapeTri& tri, Vec3f axis, Vec3f mirror, float halfThickness,
                     const ShapePose& pose) {
    auto Corner = [&](const ShapePoint& p, float side) {
        Vec3f local = Vec3f{p.x, p.y, p.z}.cwiseProduct(mirror) + axis * (side * halfThickness);
        return RotateBodyToWorld(local * pose.lengthFt, pose.rollDeg, pose.pitchDeg, pose.yawDeg, pose.worldPos);
    };
    const Vec3f top[3] = {Corner(tri.a, 1.0f), Corner(tri.b, 1.0f), Corner(tri.c, 1.0f)};
    const Vec3f bottom[3] = {Corner(tri.a, -1.0f), Corner(tri.b, -1.0f), Corner(tri.c, -1.0f)};

    auto Push = [&](Vec3f v) {
        out.push_back(v.x());
        out.push_back(v.y());
        out.push_back(v.z());
    };
    auto Tri = [&](Vec3f a, Vec3f b, Vec3f c) {
        Push(a);
        Push(b);
        Push(c);
    };

    Tri(top[0], top[1], top[2]);
    Tri(bottom[0], bottom[1], bottom[2]);
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        Tri(top[i], top[j], bottom[i]);
        Tri(top[j], bottom[j], bottom[i]);
    }
}

// Every triangle of `shape`, as listed and mirrored by `mirror`.
template <size_t N>
void AppendShape(std::vector<float>& out, const ShapeTri (&shape)[N], Vec3f axis, Vec3f mirror, float halfThickness,
                 const ShapePose& pose) {
    for (const ShapeTri& tri : shape) AppendShapeSlab(out, tri, axis, mirror, halfThickness, pose);
}

// Basis for a camera-facing billboard: `right`/`up` span the plane
// perpendicular to the center-to-eye direction.
void BillboardBasis(Vec3f center, Vec3f eye, Vec3f& outRight, Vec3f& outUp) {
    Vec3f toCam = eye - center;
    float toCamLen = toCam.norm();
    Vec3f viewDir = toCamLen > kMinSegmentFt ? (toCam * (1.0f / toCamLen)) : Vec3f{0.0f, 1.0f, 0.0f};

    Vec3f worldUp{0.0f, 1.0f, 0.0f};
    Vec3f right = worldUp.cross(viewDir);
    float rightLen = right.norm();
    outRight = rightLen > kMinCrossNorm ? (right * (1.0f / rightLen)) : Vec3f{1.0f, 0.0f, 0.0f};
    outUp = viewDir.cross(outRight);
}

// Waypoint markers: a camera-facing diamond billboard. Billboarded
// rather than a fixed 3D shape because a waypoint has no heading of its
// own, and a diamond reads clearly as a route point, not a vehicle.
void AppendWaypointDiamond(std::vector<float>& out, Vec3f center, float sizeFt, Vec3f eye) {
    auto Push = [&](Vec3f v) {
        out.push_back(v.x());
        out.push_back(v.y());
        out.push_back(v.z());
    };

    Vec3f right, up;
    BillboardBasis(center, eye, right, up);

    Vec3f north = center + up * sizeFt;
    Vec3f east = center + right * sizeFt;
    Vec3f south = center - up * sizeFt;
    Vec3f west = center - right * sizeFt;

    Push(north);
    Push(east);
    Push(south);
    Push(north);
    Push(south);
    Push(west);
}

// Camera-facing ring (GL_LINES) around the selected entity: reference
// geometry, not a shape competing with the marker.
void AppendSelectionRing(std::vector<float>& out, Vec3f center, float radiusFt, Vec3f eye, int segments = 32) {
    auto Push = [&](Vec3f v) {
        out.push_back(v.x());
        out.push_back(v.y());
        out.push_back(v.z());
    };

    Vec3f right, up;
    BillboardBasis(center, eye, right, up);

    for (int i = 0; i < segments; ++i) {
        float a0 = (2.0f * kPiF * i) / segments;
        float a1 = (2.0f * kPiF * (i + 1)) / segments;
        Vec3f p0 = center + (right * (radiusFt * std::cos(a0)) + up * (radiusFt * std::sin(a0)));
        Vec3f p1 = center + (right * (radiusFt * std::cos(a1)) + up * (radiusFt * std::sin(a1)));
        Push(p0);
        Push(p1);
    }
}

// Dashed line between queued waypoints. Core profile has no
// GL_LINE_STIPPLE, so the dashes are explicit segments.
void AppendDashedLine(std::vector<float>& out, Vec3f a, Vec3f b, float dashLenFt, float gapLenFt) {
    auto Push = [&](Vec3f v) {
        out.push_back(v.x());
        out.push_back(v.y());
        out.push_back(v.z());
    };

    Vec3f segment = b - a;
    float totalLen = segment.norm();
    if (totalLen < kMinSegmentFt) return;
    Vec3f dir = segment * (1.0f / totalLen);

    float strideLen = dashLenFt + gapLenFt;
    float traveled = 0.0f;
    while (traveled < totalLen) {
        float dashEnd = std::min(traveled + dashLenFt, totalLen);
        Push(a + dir * traveled);
        Push(a + dir * dashEnd);
        traveled += strideLen;
    }
}

// Vertical drag handle for a selected waypoint: a shaft plus a 4-legged
// arrowhead at each end. World-space rather than billboarded -- geometry
// this simple shows at least a couple of legs from any orbit angle.
void AppendVerticalGizmo(std::vector<float>& out, Vec3f center, float halfLengthFt, float headSizeFt) {
    auto Push = [&](Vec3f v) {
        out.push_back(v.x());
        out.push_back(v.y());
        out.push_back(v.z());
    };
    auto Leg = [&](Vec3f tip, Vec3f legBase) {
        Push(tip);
        Push(legBase + Vec3f{headSizeFt, 0.0f, 0.0f});
        Push(tip);
        Push(legBase + Vec3f{-headSizeFt, 0.0f, 0.0f});
        Push(tip);
        Push(legBase + Vec3f{0.0f, 0.0f, headSizeFt});
        Push(tip);
        Push(legBase + Vec3f{0.0f, 0.0f, -headSizeFt});
    };

    Vec3f top = center + Vec3f{0.0f, halfLengthFt, 0.0f};
    Vec3f bottom = center + Vec3f{0.0f, -halfLengthFt, 0.0f};
    Push(bottom);
    Push(top);
    Leg(top, top + Vec3f{0.0f, -headSizeFt, 0.0f});
    Leg(bottom, bottom + Vec3f{0.0f, headSizeFt, 0.0f});
}

// Missiles (KinematicsComponent) are amber, checked first since they carry
// neither tag name. Shared by the marker and trail passes.
const ImVec4& EntityColor(ComponentStore<KinematicsComponent>& kinematicsComponents, ComponentStore<Tag>& tags,
                           EntityID entity) {
    if (kinematicsComponents.Has(entity)) return LatticeStyle::kCaution;
    if (tags.Has(entity) && tags.Get(entity).name == "Pursuer") return LatticeStyle::kAccentPrimary;
    return LatticeStyle::kAccentSecondary;
}

GLuint gGridProgram = 0;
GLint gGridViewProjLoc = -1;
GLint gGridCameraPosLoc = -1;
GLint gGridColorLoc = -1;
GLint gGridSpacingLoc = -1;
GLint gGridFadeStartLoc = -1;
GLint gGridFadeEndLoc = -1;

GLuint gGridVao = 0;
GLuint gGridVbo = 0;

GLuint gMarkerVao = 0;
GLuint gMarkerVbo = 0;

GLuint gTrailProgram = 0;
GLint gTrailViewProjLoc = -1;
GLint gTrailColorLoc = -1;

GLuint gTrailVao = 0;
GLuint gTrailVbo = 0;

// The full position history per entity, oldest first, sampled on the
// trail clock (below). Never pruned, so "Show Full Trail History" has it
// all; the window is chosen at draw time.
struct TrailPoint {
    Vec3f pos;
    double time;  // gTrailTime-relative seconds -- see below
};
std::unordered_map<EntityID, std::deque<TrailPoint>> gTrailHistory;

constexpr double kTrailMaxAgeSec = 30.0;
constexpr double kTrailSampleIntervalSec = 0.15;  // ~200 points/entity over 30s
constexpr float kTrailScreenWidthPx = 4.0f;        // constant on-screen thickness, like the markers
constexpr float kTrailAlpha = 0.85f;               // flat -- a solid, clearly visible line, not a fading tail

// A stopwatch that stops while paused, so trails do not age out during a
// pause.
using TrailClock = std::chrono::steady_clock;
TrailClock::time_point gTrailLastRealTime;
bool gTrailClockStarted = false;
double gTrailTime = 0.0;
double gTrailLastSampleTime = -1.0;

// Camera-facing ribbon: each segment becomes a quad offset perpendicular
// to both the segment and the view direction, since GL line width is
// capped at ~1px on most core-profile drivers. Independent quads rather
// than a mitered strip -- at this sample rate the joints are invisible.
//
// The last segment runs from the newest sample to livePos, the entity's
// current position. Samples are taken on real time, so at high sim speed
// the newest one can be many sim-seconds old; without this joint the line
// visibly ends well behind the marker.
void AppendTrailRibbon(std::vector<float>& out, const std::deque<TrailPoint>& trail, Vec3f livePos, Vec3f eye,
                       float viewportHeight, float focalY, double trailNow, bool showFullTrail) {
    auto HalfWidthAt = [&](Vec3f p) {
        float dist = (p - eye).norm();
        return kTrailScreenWidthPx * dist / (viewportHeight * focalY);
    };
    auto Push = [&](Vec3f v, float alpha) {
        out.push_back(v.x());
        out.push_back(v.y());
        out.push_back(v.z());
        out.push_back(alpha);
    };

    // Windowed mode draws the suffix within kTrailMaxAgeSec, found by binary
    // search: the log is never pruned, so a scan would grow with session length.
    size_t startIndex = 0;
    if (!showFullTrail) {
        double cutoff = trailNow - kTrailMaxAgeSec;
        auto firstVisible = std::lower_bound(trail.begin(), trail.end(), cutoff,
                                             [](const TrailPoint& p, double t) { return p.time < t; });
        startIndex = static_cast<size_t>(firstVisible - trail.begin());
    }

    auto AppendSegment = [&](Vec3f p0, Vec3f p1) {
        Vec3f segment = p1 - p0;
        float segmentLen = segment.norm();
        if (segmentLen < kMinSegmentFt) return;  // near-duplicate samples

        Vec3f dir = segment * (1.0f / segmentLen);
        Vec3f mid = (p0 + p1) * 0.5f;
        Vec3f toCam = eye - mid;
        float toCamLen = toCam.norm();
        Vec3f viewDir = toCamLen > kMinSegmentFt ? (toCam * (1.0f / toCamLen)) : Vec3f{0.0f, 1.0f, 0.0f};

        Vec3f right = dir.cross(viewDir);
        float rightLen = right.norm();
        // Looking straight down the segment: fall back to world up so the ribbon
        // keeps its width.
        right = rightLen > kMinCrossNorm ? (right * (1.0f / rightLen)) : dir.cross(Vec3f{0.0f, 1.0f, 0.0f}).normalized();

        float halfWidth0 = HalfWidthAt(p0);
        float halfWidth1 = HalfWidthAt(p1);

        Vec3f v0 = p0 + right * halfWidth0;
        Vec3f v1 = p0 - right * halfWidth0;
        Vec3f v2 = p1 + right * halfWidth1;
        Vec3f v3 = p1 - right * halfWidth1;

        Push(v0, kTrailAlpha);
        Push(v1, kTrailAlpha);
        Push(v2, kTrailAlpha);
        Push(v1, kTrailAlpha);
        Push(v3, kTrailAlpha);
        Push(v2, kTrailAlpha);
    };

    for (size_t i = startIndex; i + 1 < trail.size(); ++i) AppendSegment(trail[i].pos, trail[i + 1].pos);
    if (startIndex < trail.size()) AppendSegment(trail.back().pos, livePos);
}

using Units::kFeetPerMeter;

// Cone lengths are a visual reference, not a detection boundary: the
// radar equation solved for a representative fighter-sized target.
constexpr double kRepresentativeRcsM2 = 5.0;
// The reference range is where that target is detected half the time.
constexpr double kReferencePd = 0.5;

// At the radar's search threshold.
double AirborneRadarReferenceRangeFt(const AirborneRadarComponent& radar) {
    return RadarEquation::ReferenceRangeM(ReceiverOf(radar), kRepresentativeRcsM2,
                                          RadarEquation::SwerlingOneRequiredSnrDb(radar.searchThresholdDb, kReferencePd)) *
           kFeetPerMeter;
}

// At the active seeker's threshold, so the cone reads as "the volume inside
// which this missile can go autonomous".
double MissileSeekerReferenceRangeFt(const GuidanceComponent& guidance) {
    return RadarEquation::ReferenceRangeM(SeekerReceiverOf(guidance), kRepresentativeRcsM2,
                                          RadarEquation::SwerlingOneRequiredSnrDb(guidance.seekerThresholdDb, kReferencePd)) *
           kFeetPerMeter;
}

// The missile seeker's state expressed on the same escalation ladder the
// airborne radar uses, so one color code covers both cones. The
// correspondence is closer than the reused enum name suggests:
//
//   SEARCH           hunting -- a MIDCOURSE_ARH missile before its own
//                    seeker acquires, steering on the launcher's datalinked
//                    track rather than on anything it can see itself.
//   TRACK            coasting on memory -- lock is lost but still inside
//                    memoryCoastTimeS, so it steers on the frozen last real
//                    LOS rate. A track, just a stale one.
//   FIRING_SOLUTION  measuring the target with its own antenna right now.
//   NONE             guidance is dead: the coast window expired, the LOS
//                    rate is zeroed, and the missile is ballistic.
//
// GuidanceSystem holds hasLock true for the whole coast window and clears
// it only once that window expires, so hasLock alone cannot separate a
// seeker that is measuring from one that is remembering --
// timeSinceLockLostS is what distinguishes them.
RadarTrackMode SeekerCueMode(const GuidanceComponent& guidance) {
    if (!guidance.hasLock) return RadarTrackMode::NONE;
    if (guidance.timeSinceLockLostS > 0.0) return RadarTrackMode::TRACK;
    if (guidance.seekerMode == SeekerMode::MIDCOURSE_ARH && !guidance.seekerActive) return RadarTrackMode::SEARCH;
    return RadarTrackMode::FIRING_SOLUTION;
}

// Clips a segment to y >= 0; false if it is entirely below ground. A cone
// from a moving apex cannot restrict its angles up front, so it clips per
// segment.
bool ClipSegmentAboveGround(Vec3f a, Vec3f b, Vec3f& outA, Vec3f& outB) {
    bool aAbove = a.y() >= 0.0f;
    bool bAbove = b.y() >= 0.0f;
    if (!aAbove && !bAbove) return false;
    if (aAbove && bAbove) {
        outA = a;
        outB = b;
        return true;
    }
    float t = a.y() / (a.y() - b.y());  // fraction from a to b where y crosses 0
    Vec3f crossing = a + (b - a) * t;
    if (aAbove) {
        outA = a;
        outB = crossing;
    } else {
        outA = crossing;
        outB = b;
    }
    return true;
}

// The triangle a, b, c clipped to y >= 0 (one-plane Sutherland-Hodgman),
// appended as a fan: nothing, one triangle, or two.
void AppendTriangleAboveGround(std::vector<float>& out, Vec3f a, Vec3f b, Vec3f c) {
    const Vec3f in[3] = {a, b, c};
    Vec3f kept[4];
    int count = 0;
    for (int i = 0; i < 3; ++i) {
        Vec3f p = in[i], q = in[(i + 1) % 3];
        if (p.y() >= 0.0f) kept[count++] = p;
        if ((p.y() >= 0.0f) != (q.y() >= 0.0f)) kept[count++] = p + (q - p) * (p.y() / (p.y() - q.y()));
    }
    for (int i = 1; i + 1 < count; ++i) {
        for (Vec3f v : {kept[0], kept[i], kept[i + 1]}) {
            out.push_back(v.x());
            out.push_back(v.y());
            out.push_back(v.z());
        }
    }
}

// A sensor cone as a translucent surface plus a faint outline: the
// open-end rim and the two side silhouettes, the generators where the
// surface turns from facing the eye to facing away. Apex, axis along the
// forward direction implied by the Euler angles, half-angle and length.
// Symmetric around the nose rather than a rectangular azimuth/elevation
// gimbal -- a real radar's limits are not a crisp rectangle either.
// Everything below y=0 is clipped.
struct ConeGeometry {
    std::vector<float> fill;     // GL_TRIANGLES
    std::vector<float> outline;  // GL_LINES
};

ConeGeometry BuildCone(Vec3f apex, float rollDeg, float pitchDeg, float yawDeg, float halfAngleDeg, float length,
                       Vec3f eye, int ringSegments = 48) {
    float halfAngleRad = halfAngleDeg * kDegToRadF;
    float baseRadius = length * std::sin(halfAngleRad);
    float baseForward = length * std::cos(halfAngleRad);

    std::vector<Vec3f> ring(ringSegments);
    for (int i = 0; i < ringSegments; ++i) {
        float a = (2.0f * kPiF * i) / ringSegments;
        Vec3f local = {baseRadius * std::cos(a), baseRadius * std::sin(a), baseForward};
        ring[i] = RotateBodyToWorld(local, rollDeg, pitchDeg, yawDeg, apex);
    }

    ConeGeometry cone;
    auto PushLine = [&](Vec3f a, Vec3f b) {
        Vec3f ca, cb;
        if (!ClipSegmentAboveGround(a, b, ca, cb)) return;
        for (Vec3f v : {ca, cb}) {
            cone.outline.push_back(v.x());
            cone.outline.push_back(v.y());
            cone.outline.push_back(v.z());
        }
    };
    // Face i spans ring[i] to ring[i + 1].
    auto FacesEye = [&](int i) {
        Vec3f normal = (ring[i] - apex).cross(ring[(i + 1) % ringSegments] - apex);
        return normal.dot(eye - apex) > 0.0f;
    };

    for (int i = 0; i < ringSegments; ++i) {
        int next = (i + 1) % ringSegments;
        AppendTriangleAboveGround(cone.fill, apex, ring[i], ring[next]);
        PushLine(ring[i], ring[next]);
        if (FacesEye(i) != FacesEye(next)) PushLine(apex, ring[next]);
    }
    return cone;
}

// The VAO/VBO binding only: the quad is rebuilt each frame around the
// camera's focus (UpdateGridQuad), so panning cannot outrun it.
void InitGridQuad() {
    glGenVertexArrays(1, &gGridVao);
    glGenBuffers(1, &gGridVbo);
    glBindVertexArray(gGridVao);
    glBindBuffer(GL_ARRAY_BUFFER, gGridVbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

// Re-centers and re-sizes the ground quad on the camera's XZ focus.
void UpdateGridQuad(Vec3f center, float halfSize) {
    const float vertices[] = {
        center.x() - halfSize, 0.0f, center.z() - halfSize, center.x() + halfSize, 0.0f, center.z() - halfSize,
        center.x() - halfSize, 0.0f, center.z() + halfSize, center.x() + halfSize, 0.0f, center.z() + halfSize,
    };
    glBindBuffer(GL_ARRAY_BUFFER, gGridVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
}

// What every pass needs to know about this frame's view.
struct FrameView {
    Mat4 viewProj;
    Vec3f eye;
    float viewportHeight;
    // proj(1, 1), the vertical focal term 1/tan(fovY/2): converts a pixel
    // length to a world length at a given distance.
    float focalY;
    double trailNow;

    float DistanceTo(Vec3f pos) const { return (pos - eye).norm(); }

    // World-space length that spans `pixels` on screen at `pos`, so a
    // marker keeps a constant on-screen size regardless of zoom.
    float WorldLengthForPixels(Vec3f pos, float pixels) const {
        return pixels * DistanceTo(pos) / (viewportHeight * focalY);
    }
};

// Binds the flat-color marker program, used by every pass except the grid
// and trails.
void UseMarkerProgram(const FrameView& frame) {
    glUseProgram(gProgram);
    glUniformMatrix4fv(gViewProjLoc, 1, GL_FALSE, frame.viewProj.data());
    glBindVertexArray(gMarkerVao);
    glBindBuffer(GL_ARRAY_BUFFER, gMarkerVbo);
}

void SetColor(const ImVec4& color, float alpha) { glUniform4f(gColorLoc, color.x, color.y, color.z, alpha); }

// Uploads xyz vertices to the bound buffer and draws them.
void UploadAndDraw(const std::vector<float>& verts, GLenum mode) {
    if (verts.empty()) return;
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(mode, 0, static_cast<GLsizei>(verts.size() / 3));
}

// Coverage-antialiased lines: MSAA alone leaves angled 1px GL_LINES
// stepped. Needs GL_BLEND, which the blended pass has on.
class ScopedLineSmooth {
public:
    ScopedLineSmooth() {
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    }
    ~ScopedLineSmooth() { glDisable(GL_LINE_SMOOTH); }
    ScopedLineSmooth(const ScopedLineSmooth&) = delete;
    ScopedLineSmooth& operator=(const ScopedLineSmooth&) = delete;
};

// The aircraft or missile icon at `worldPos`, in its attitude.
void AppendEntityMarker(std::vector<float>& out, const FrameView& frame, Vec3f worldPos, const Vec3& eulerDeg,
                        bool isMissile) {
    // WorldLengthForPixels spans half the length; hence the 2.
    ShapePose pose{frame.WorldLengthForPixels(worldPos, kMarkerScreenLengthPx *
                                                            (isMissile ? kMissileLengthScale : 1.0f) * 2.0f),
                   static_cast<float>(eulerDeg.x()), static_cast<float>(eulerDeg.y()),
                   static_cast<float>(eulerDeg.z()), worldPos};
    const Vec3f kAsListed{1.0f, 1.0f, 1.0f};
    const Vec3f kMirrorX{-1.0f, 1.0f, 1.0f};
    const Vec3f kMirrorY{1.0f, -1.0f, 1.0f};
    const Vec3f kAxisX{1.0f, 0.0f, 0.0f};
    const Vec3f kAxisY{0.0f, 1.0f, 0.0f};
    if (isMissile) {
        AppendShape(out, kMissileBodyRight, kAxisY, kAsListed, kMissileHalfThickness, pose);
        AppendShape(out, kMissileBodyRight, kAxisY, kMirrorX, kMissileHalfThickness, pose);
        AppendShape(out, kMissileFinTop, kAxisX, kAsListed, kMissileHalfThickness, pose);
        AppendShape(out, kMissileFinTop, kAxisX, kMirrorY, kMissileHalfThickness, pose);
    } else {
        AppendShape(out, kAircraftPlanformRight, kAxisY, kAsListed, kAircraftHalfThickness, pose);
        AppendShape(out, kAircraftPlanformRight, kAxisY, kMirrorX, kAircraftHalfThickness, pose);
        AppendShape(out, kAircraftFin, kAxisX, kAsListed, kAircraftHalfThickness, pose);
    }
}

// Advances the trail clock: real time, but frozen while paused so trails
// don't fade away purely from time spent paused. Returns the new "now".
double AdvanceTrailClock(bool paused) {
    TrailClock::time_point nowReal = TrailClock::now();
    if (!gTrailClockStarted) {
        gTrailLastRealTime = nowReal;
        gTrailClockStarted = true;
    }
    if (!paused) gTrailTime += std::chrono::duration<double>(nowReal - gTrailLastRealTime).count();
    gTrailLastRealTime = nowReal;
    return gTrailTime;
}

// Opaque entity markers, one draw call each -- fine at this entity count.
// Colored by Tag name, a stopgap rather than a real faction concept. Drawn
// before the transparent grid so opaque markers occlude correctly. Also
// where trails are sampled, since this is the one pass over every entity's
// true position.
void DrawEntityMarkers(World& world, const FrameView& frame) {
    ComponentStore<Tag>& tags = world.Components<Tag>();
    ComponentStore<KinematicsComponent>& kinematicsComponents = world.Components<KinematicsComponent>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();

    bool trailSampleDue =
        gTrailLastSampleTime < 0.0 || (frame.trailNow - gTrailLastSampleTime) >= kTrailSampleIntervalSec;

    UseMarkerProgram(frame);
    for (auto& [entity, transform] : world.Components<Transform>()) {
        // A destroyed entity's marker goes muted grey; its trail keeps its
        // own color (GUI_DESIGN - Entity markers).
        SetColor(destroyed.Has(entity) ? LatticeStyle::kTextSecondary : EntityColor(kinematicsComponents, tags, entity),
                 1.0f);
        Vec3f worldPos = ToVec3f(transform.pos);

        if (trailSampleDue) gTrailHistory[entity].push_back(TrailPoint{worldPos, frame.trailNow});

        std::vector<float> verts;
        AppendEntityMarker(verts, frame, worldPos, transform.euler, kinematicsComponents.Has(entity));
        UploadAndDraw(verts, GL_TRIANGLES);
    }
    if (trailSampleDue) gTrailLastSampleTime = frame.trailNow;
}

// Ground grid: border color, low alpha, fading out with distance so it
// reads as receding toward the horizon rather than visibly stopping or
// competing with entity markers.
//
// Fade distances and line spacing scale with the camera's own orbit
// distance, so the grid never outruns the camera when zoomed out. The quad
// is rebuilt each frame centered on the focus point and sized past
// fadeEnd, so panning far from the origin cannot lose it.
void DrawGrid(const Camera& camera, const FrameView& frame) {
    constexpr float kFadeStartRatio = 3.0f;
    constexpr float kFadeEndRatio = 7.5f;
    float fadeStart = camera.distance * kFadeStartRatio;
    float fadeEnd = camera.distance * kFadeEndRatio;
    constexpr float kQuadMarginRatio = 1.1f;  // quad reaches past the fade, so its edge never shows
    UpdateGridQuad(Vec3f{camera.target.x(), 0.0f, camera.target.z()}, fadeEnd * kQuadMarginRatio);

    glUseProgram(gGridProgram);
    glUniformMatrix4fv(gGridViewProjLoc, 1, GL_FALSE, frame.viewProj.data());
    glUniform3f(gGridCameraPosLoc, frame.eye.x(), frame.eye.y(), frame.eye.z());
    glUniform3f(gGridColorLoc, LatticeStyle::kBorder.x, LatticeStyle::kBorder.y, LatticeStyle::kBorder.z);
    glUniform1f(gGridSpacingLoc, RenderSystem::GridSpacingFt(camera.distance));
    glUniform1f(gGridFadeStartLoc, fadeStart);
    glUniform1f(gGridFadeEndLoc, fadeEnd);
    glBindVertexArray(gGridVao);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

// The outline is this much more opaque than the fill (~50% over the
// default 18%), capped at solid.
constexpr float kConeOutlineToFill = 2.8f;

// Draws a cone in its tier's colour, scaled from the viewer's opacity.
void DrawCone(const ConeGeometry& cone, RadarTrackMode tier, float opacity) {
    float fillAlpha = opacity * LatticeStyle::RadarTrackModeConeWeight(tier);
    const ImVec4 color = LatticeStyle::RadarTrackModeColor(tier);
    SetColor(color, fillAlpha);
    UploadAndDraw(cone.fill, GL_TRIANGLES);
    SetColor(color, std::min(1.0f, fillAlpha * kConeOutlineToFill));
    UploadAndDraw(cone.outline, GL_LINES);
}

// Airborne radar FOV cone from each radar-carrying aircraft's nose.
void DrawAirborneRadarCones(World& world, const SimContext& simContext, const FrameView& frame, float opacity) {
    ComponentStore<Transform>& transforms = world.Components<Transform>();

    // The reference range is where a 5 m2 target is a coin flip, which is
    // about where the first detections come, so the cone is drawn this many
    // times longer to be easy to see. It is a picture of the field of view,
    // not of that range.
    constexpr float kConeLengthScale = 2.0f;

    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    UseMarkerProgram(frame);
    ScopedLineSmooth lineSmooth;
    for (auto& [owner, radar] : world.Components<AirborneRadarComponent>()) {
        const Transform* ownerTransform = transforms.TryGet(owner);
        if (ownerTransform == nullptr) continue;
        // A destroyed aircraft's radar is off (REFERENCE 9.6): no cone.
        if (destroyed.Has(owner)) continue;
        float coneLengthFt = static_cast<float>(AirborneRadarReferenceRangeFt(radar)) * kConeLengthScale;
        if (coneLengthFt <= 0.0f) continue;

        ConeGeometry cone =
            BuildCone(ToVec3f(ownerTransform->pos), static_cast<float>(ownerTransform->euler.x()),
                      static_cast<float>(ownerTransform->euler.y()), static_cast<float>(ownerTransform->euler.z()),
                      static_cast<float>(radar.fovHalfAngleDeg), coneLengthFt, frame.eye);

        // Green searching / amber tracking / red locked, so the escalation
        // is readable from the 3D view alone without opening the inspector.
        DrawCone(cone, RenderSystem::RadarConeTier(simContext, owner, radar), opacity);
    }
}

// Missile seeker FOV cone. Oriented along Transform.euler, which is the
// missile's actual velocity vector -- the exact vector the guidance
// in-cone check is computed against, so this renders that check's real
// gimbal limit rather than an approximation. Length is the seeker's own
// acquisition range, solved from the same radar equation the seeker uses
// at runtime, so the cone is a physically meaningful volume: inside it a
// MIDCOURSE_ARH missile can go active and stop needing the launcher's
// track.
void DrawSeekerCones(World& world, const FrameView& frame, float opacity) {
    ComponentStore<Transform>& transforms = world.Components<Transform>();

    UseMarkerProgram(frame);
    ScopedLineSmooth lineSmooth;
    for (auto& [entity, guidance] : world.Components<GuidanceComponent>()) {
        if (!guidance.active) continue;
        const Transform* missileTransform = transforms.TryGet(entity);
        if (missileTransform == nullptr) continue;
        float seekerConeLengthFt = static_cast<float>(MissileSeekerReferenceRangeFt(guidance));
        if (seekerConeLengthFt <= 0.0f) continue;

        ConeGeometry cone =
            BuildCone(ToVec3f(missileTransform->pos), static_cast<float>(missileTransform->euler.x()),
                      static_cast<float>(missileTransform->euler.y()), static_cast<float>(missileTransform->euler.z()),
                      static_cast<float>(guidance.seekerFovHalfAngleDeg), seekerConeLengthFt, frame.eye);

        // Same ramp as the radar cone (see SeekerCueMode for how a seeker's
        // states map onto those tiers): green while it is still hunting on
        // the launcher's track, red once it is homing on its own return,
        // amber while coasting on memory, and grey once guidance is dead.
        DrawCone(cone, SeekerCueMode(guidance), opacity);
    }
}

// Waypoint queues: a dashed line from the entity through each queued
// point, plus a diamond at each. queue.front reads brighter -- the same
// "more relevant = more opaque" convention used elsewhere.
//
// Capped to a generous lookahead, since hand-placed routes run long. Once
// a looping queue has rotated, its back entry is the point just left, so
// rendering it would draw a segment ending on top of the entity every time
// a waypoint is passed; that one trailing entry is excluded. Before the
// first rotation, and after an edit (which re-bases loopRotation to 0),
// the back entry is a point still to fly -- the user's newest one -- and
// is drawn.
//
// Every route's geometry is gathered first and drawn in three calls total
// -- all dashes, all front diamonds, all later diamonds -- rather than one
// upload and draw per segment and per diamond. Only the alpha differs
// between the three.
void DrawWaypointRoutes(World& world, const FrameView& frame) {
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    constexpr float kDashLenFt = 200.0f;
    constexpr float kGapLenFt = 120.0f;
    constexpr float kLineAlpha = 0.8f;
    constexpr float kFrontMarkerAlpha = 0.9f;
    constexpr float kLaterMarkerAlpha = 0.35f;
    constexpr size_t kMaxRenderedWaypoints = 32;

    std::vector<float> dashes;
    std::vector<float> frontDiamonds;
    std::vector<float> laterDiamonds;

    for (auto& [entity, waypoint] : world.Components<WaypointComponent>()) {
        const Transform* entityTransform = transforms.TryGet(entity);
        if (waypoint.queue.empty() || entityTransform == nullptr) continue;
        bool backIsJustLeft = waypoint.loop && waypoint.loopRotation > 0 && waypoint.queue.size() > 1;
        size_t effectiveCount = backIsJustLeft ? waypoint.queue.size() - 1 : waypoint.queue.size();
        size_t renderCount = std::min(effectiveCount, kMaxRenderedWaypoints);

        Vec3f prev = ToVec3f(entityTransform->pos);
        for (size_t i = 0; i < renderCount; ++i) {
            Vec3f current = ToVec3f(waypoint.queue[i].pos);
            AppendDashedLine(dashes, prev, current, kDashLenFt, kGapLenFt);
            prev = current;
            float sizeFt = frame.WorldLengthForPixels(current, kMarkerScreenLengthPx * kWaypointMarkerScale);
            AppendWaypointDiamond(i == 0 ? frontDiamonds : laterDiamonds, current, sizeFt, frame.eye);
        }
    }

    UseMarkerProgram(frame);
    ScopedLineSmooth lineSmooth;
    SetColor(LatticeStyle::kAccentPrimary, kLineAlpha);
    UploadAndDraw(dashes, GL_LINES);
    SetColor(LatticeStyle::kAccentPrimary, kLaterMarkerAlpha);
    UploadAndDraw(laterDiamonds, GL_TRIANGLES);
    SetColor(LatticeStyle::kAccentPrimary, kFrontMarkerAlpha);
    UploadAndDraw(frontDiamonds, GL_TRIANGLES);
}

// Selected-waypoint altitude-drag gizmo: a vertical handle at whichever
// queue entry main.cpp's placement-mode editing currently has grabbed,
// scaled the same way the diamond markers are so it reads as a consistent
// on-screen size regardless of zoom.
void DrawWaypointGizmo(World& world, const FrameView& frame, size_t waypointIndex) {
    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    if (!selected.has_value()) return;
    const WaypointComponent* waypoint = world.Components<WaypointComponent>().TryGet(*selected);
    if (waypoint == nullptr || waypointIndex >= waypoint->queue.size()) return;

    Vec3f center = ToVec3f(waypoint->queue[waypointIndex].pos);
    constexpr float kGizmoHalfLengthScale = 2.2f;  // x marker length
    constexpr float kGizmoHeadScale = 0.45f;
    float halfLengthFt = frame.WorldLengthForPixels(center, kMarkerScreenLengthPx * kGizmoHalfLengthScale);
    float headSizeFt = frame.WorldLengthForPixels(center, kMarkerScreenLengthPx * kGizmoHeadScale);

    UseMarkerProgram(frame);
    ScopedLineSmooth lineSmooth;
    std::vector<float> verts;
    AppendVerticalGizmo(verts, center, halfLengthFt, headSizeFt);
    SetColor(LatticeStyle::kCaution, 1.0f);
    UploadAndDraw(verts, GL_LINES);
}

// Selection outline: a camera-facing ring around whichever entity is
// selected in the Entity Inspector, sized a bit larger than the entity
// marker itself (kSelectionRingScale) so it visibly encircles any marker
// type -- aircraft or missile -- without touching it.
void DrawSelectionRing(World& world, const FrameView& frame) {
    constexpr float kSelectionRingScale = 1.2f;
    constexpr float kSelectionRingAlpha = 0.9f;
    std::optional<EntityID> selected = EntityInspector::SelectedEntity();
    if (!selected.has_value()) return;
    const Transform* transform = world.Components<Transform>().TryGet(*selected);
    if (transform == nullptr) return;

    Vec3f center = ToVec3f(transform->pos);
    float radiusFt = frame.WorldLengthForPixels(center, kMarkerScreenLengthPx * kSelectionRingScale);

    UseMarkerProgram(frame);
    ScopedLineSmooth lineSmooth;
    std::vector<float> verts;
    AppendSelectionRing(verts, center, radiusFt, frame.eye);
    SetColor(LatticeStyle::kAccentPrimary, kSelectionRingAlpha);
    UploadAndDraw(verts, GL_LINES);
}

// Entity trails: each entity's own marker color, as a solid camera-facing
// ribbon (constant on-screen width, like the markers) covering its last 30
// seconds. Drawn after the grid, in the same blended/no-depth-write pass,
// so a trail reads on top of the grid lines instead of getting lost in
// them.
void DrawTrails(World& world, const FrameView& frame, bool showFullTrail) {
    ComponentStore<Tag>& tags = world.Components<Tag>();
    ComponentStore<KinematicsComponent>& kinematicsComponents = world.Components<KinematicsComponent>();

    glUseProgram(gTrailProgram);
    glUniformMatrix4fv(gTrailViewProjLoc, 1, GL_FALSE, frame.viewProj.data());
    glBindVertexArray(gTrailVao);
    glBindBuffer(GL_ARRAY_BUFFER, gTrailVbo);
    for (auto& [entity, transform] : world.Components<Transform>()) {
        // find() rather than operator[] -- an entity that has not sampled a
        // trail yet shouldn't get an empty deque inserted for it every
        // frame just by being iterated here.
        auto trailIt = gTrailHistory.find(entity);
        if (trailIt == gTrailHistory.end() || trailIt->second.size() < 2) continue;

        const ImVec4& color = EntityColor(kinematicsComponents, tags, entity);
        glUniform3f(gTrailColorLoc, color.x, color.y, color.z);

        std::vector<float> verts;
        AppendTrailRibbon(verts, trailIt->second, ToVec3f(transform.pos), frame.eye, frame.viewportHeight,
                          frame.focalY, frame.trailNow, showFullTrail);
        if (verts.empty()) continue;
        // Trail vertices carry a fourth (alpha) component.
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts.size() / 4));
    }
}

}  // namespace

namespace RenderSystem {

float GridSpacingFt(float cameraDistance) {
    // About kTargetRatio squares between the camera and its focus, snapped to
    // the 1/2/5 ladder: 1000 ft at the default 20000 ft distance.
    constexpr float kTargetRatio = 20.0f;
    float target = std::max(cameraDistance, 1.0f) / kTargetRatio;
    float magnitude = std::pow(10.0f, std::floor(std::log10(target)));
    float residual = target / magnitude;
    float niceResidual = residual < 1.5f ? 1.0f : (residual < 3.5f ? 2.0f : (residual < 7.5f ? 5.0f : 10.0f));
    return niceResidual * magnitude;
}

void Init() {
    // Sampling for the multisampled framebuffer main.cpp requests (GLFW_SAMPLES).
    glEnable(GL_MULTISAMPLE);

    GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, kVertexShaderSrc);
    GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, kFragmentShaderSrc);
    gProgram = LinkProgram(vertexShader, fragmentShader);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    gViewProjLoc = glGetUniformLocation(gProgram, "uViewProj");
    gColorLoc = glGetUniformLocation(gProgram, "uColor");

    GLuint gridVertexShader = CompileShader(GL_VERTEX_SHADER, kGridVertexShaderSrc);
    GLuint gridFragmentShader = CompileShader(GL_FRAGMENT_SHADER, kGridFragmentShaderSrc);
    gGridProgram = LinkProgram(gridVertexShader, gridFragmentShader);
    glDeleteShader(gridVertexShader);
    glDeleteShader(gridFragmentShader);

    gGridViewProjLoc = glGetUniformLocation(gGridProgram, "uViewProj");
    gGridCameraPosLoc = glGetUniformLocation(gGridProgram, "uCameraPos");
    gGridColorLoc = glGetUniformLocation(gGridProgram, "uGridColor");
    gGridSpacingLoc = glGetUniformLocation(gGridProgram, "uSpacing");
    gGridFadeStartLoc = glGetUniformLocation(gGridProgram, "uFadeStart");
    gGridFadeEndLoc = glGetUniformLocation(gGridProgram, "uFadeEnd");

    InitGridQuad();

    // Data is uploaded per frame; only the attribute binding here.
    glGenVertexArrays(1, &gMarkerVao);
    glGenBuffers(1, &gMarkerVbo);
    glBindVertexArray(gMarkerVao);
    glBindBuffer(GL_ARRAY_BUFFER, gMarkerVbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    GLuint trailVertexShader = CompileShader(GL_VERTEX_SHADER, kTrailVertexShaderSrc);
    GLuint trailFragmentShader = CompileShader(GL_FRAGMENT_SHADER, kTrailFragmentShaderSrc);
    gTrailProgram = LinkProgram(trailVertexShader, trailFragmentShader);
    glDeleteShader(trailVertexShader);
    glDeleteShader(trailFragmentShader);

    gTrailViewProjLoc = glGetUniformLocation(gTrailProgram, "uViewProj");
    gTrailColorLoc = glGetUniformLocation(gTrailProgram, "uColor");

    // Interleaved (pos.xyz, alpha), uploaded per frame like the markers.
    glGenVertexArrays(1, &gTrailVao);
    glGenBuffers(1, &gTrailVbo);
    glBindVertexArray(gTrailVao);
    glBindBuffer(GL_ARRAY_BUFFER, gTrailVbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

void ResetTrails() {
    gTrailHistory.clear();
    gTrailLastSampleTime = -1.0;
}

void Shutdown() {
    glDeleteVertexArrays(1, &gGridVao);
    glDeleteBuffers(1, &gGridVbo);
    glDeleteVertexArrays(1, &gMarkerVao);
    glDeleteBuffers(1, &gMarkerVbo);
    glDeleteVertexArrays(1, &gTrailVao);
    glDeleteBuffers(1, &gTrailVbo);
    glDeleteProgram(gProgram);
    glDeleteProgram(gGridProgram);
    glDeleteProgram(gTrailProgram);
}

void Draw(World& world, const SimContext& simContext, int viewportWidth, int viewportHeight, const Camera& camera,
          const RenderOptions& options) {
    // A minimised window has no framebuffer: dividing by its height would
    // upload NaN geometry.
    if (viewportWidth <= 0 || viewportHeight <= 0) return;
    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);

    float aspect = static_cast<float>(viewportWidth) / static_cast<float>(viewportHeight);
    Mat4 proj = Camera::ProjectionMatrix(aspect, options.viewNdcShiftX);

    FrameView frame;
    frame.viewProj = proj * camera.ViewMatrix();
    frame.eye = camera.EyePosition();
    frame.viewportHeight = static_cast<float>(viewportHeight);
    frame.focalY = proj(1, 1);
    frame.trailNow = AdvanceTrailClock(options.paused);

    DrawEntityMarkers(world, frame);

    // Everything below is see-through: depth writes off so translucent
    // fragments do not occlude later ones. The opaque markers went first.
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    const ViewSettings& view = options.view;
    if (view.showGrid) DrawGrid(camera, frame);
    if (view.showRadarCones) DrawAirborneRadarCones(world, simContext, frame, view.coneOpacity);
    if (view.showSeekerCones) DrawSeekerCones(world, frame, view.coneOpacity);
    if (view.showWaypointRoutes) DrawWaypointRoutes(world, frame);
    if (options.selectedWaypointIndex.has_value()) DrawWaypointGizmo(world, frame, *options.selectedWaypointIndex);
    DrawSelectionRing(world, frame);
    if (view.showTrails) DrawTrails(world, frame, view.showFullTrail);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    glBindVertexArray(0);
    glUseProgram(0);
}

}  // namespace RenderSystem
