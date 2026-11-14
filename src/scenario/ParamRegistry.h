#pragma once

#include <functional>
#include <string>
#include <vector>

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"

// The C++ mirror of docs/SCHEMA.md: one entry per tunable scenario field,
// keyed by the same JSON Pointer the batch runner already accepts. One entry
// drives the GUI editor and the scenario saver and matches what
// `bvr_batch` overrides with no translation -- so a tunable has a
// single representation rather than separate loader/widget/pointer copies to
// keep in sync.
//
// Adding a field to SCHEMA.md means adding an entry here in the same commit,
// or it will be invisible to the viewer and lost on save.
//
// Only the entries marked `knob` -- the short list of teaching knobs
// (docs/SCHEMA.md - Teaching knobs) -- are editable: in the GUI, by a scenario
// key, by a batch override, and on save. Every other entry is a type-file
// value, shown read-only so a user can see what the model flies; a
// scenario key for one is a load error.
namespace ParamRegistry {

// Where the live value lives. Doctrine and Environment are scenario-wide
// (stored on ScenarioData); Entity entries carry a pointer template
// containing "{}", which resolves to the subject's index in the scenario's
// own array.
enum class ParamOwner { Doctrine, Environment, Entity };

// When an edit takes effect, which differs enough between fields to be
// worth showing in the UI rather than letting a user wonder why nothing
// happened.
enum class ParamTiming {
    Live,        // read by a system every tick -- takes effect immediately
    NextLaunch,  // copied into the missile's components at launch
    Setup,       // authored before a run but simulated during it (transforms)
};

enum class ParamType { Double, Bool, Enum };

// How a JSON value converts to the stored one. Non-unity only where the
// schema's authoring unit differs from the internal one (metres to feet,
// newtons to pounds-force, kilograms to slugs). Some conversions are
// naturally a division -- kg to slugs is x / kKgPerSlug -- and are stored
// that way rather than as a multiply by the reciprocal, which rounds
// differently.
struct JsonScale {
    double factor = 1.0;
    bool divides = false;

    JsonScale() = default;
    JsonScale(double multiplyBy) : factor(multiplyBy) {}  // NOLINT: implicit on purpose, the common case
    static JsonScale DividedBy(double divisor) {
        JsonScale scale(divisor);
        scale.divides = true;
        return scale;
    }

    double ToInternal(double jsonValue) const { return divides ? jsonValue / factor : jsonValue * factor; }
    double ToJson(double internalValue) const { return divides ? internalValue * factor : internalValue / factor; }
};

// Enums are string-valued in JSON and strongly typed in C++, so they need
// a get/set pair rather than a pointer to alias.
struct EnumAccess {
    std::vector<const char*> names;  // index order matches the JSON strings
    std::function<int(ScenarioData&, World&, EntityID)> get;
    std::function<void(ScenarioData&, World&, EntityID, int)> set;
};

struct ParamDef {
    const char* pointer = nullptr;
    const char* group = nullptr;
    const char* label = nullptr;
    const char* unit = nullptr;
    // One-line plain-English explanation, shown as a hover tooltip in the
    // editor. Filled in by DescriptionFor(pointer) after the table is built,
    // so the shorthand constructors stay uncluttered. May be null.
    const char* description = nullptr;
    ParamOwner owner = ParamOwner::Doctrine;
    ParamTiming timing = ParamTiming::Live;
    ParamType type = ParamType::Double;
    // A teaching knob (editable), or a type value shown read-only. Set by
    // All() from the one knob list.
    bool knob = false;
    // An advanced knob: still editable everywhere, but the GUI shows it only
    // in Advanced mode, with the type values. Every lesson's suggested knob
    // is a basic one. Set by All(); false for a non-knob.
    bool advanced = false;

    double minValue = 0.0;
    double maxValue = 0.0;
    double dragStep = 0.0;

    // Load applies ToInternal, save applies ToJson.
    JsonScale jsonScale;

    std::function<double*(ScenarioData&, World&, EntityID)> dbl;
    std::function<bool*(ScenarioData&, World&, EntityID)> flag;
    EnumAccess enumAccess;
};

// The whole table, built once on first call.
const std::vector<ParamDef>& All();

// `pointer` with "{}" replaced by `index`. Returns the pointer unchanged
// for Doctrine entries, which have no placeholder.
std::string ResolvePointer(const ParamDef& param, int index);

// The entry `pointer` addresses, or nullptr -- "{}" in an entry's
// template matches any array index, and a template itself matches too.
const ParamDef* Find(const std::string& pointer);

// The index of `name` in an Enum entry's names, or -1 if it is not one.
int EnumIndex(const ParamDef& param, const std::string& name);

}  // namespace ParamRegistry
