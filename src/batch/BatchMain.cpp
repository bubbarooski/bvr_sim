// Headless Monte Carlo driver: runs a list of scenario variants in
// parallel and writes two CSVs -- a wide one-row-per-run results table
// and a long-format event table joinable back on run_id.
//
// It has no sweep logic of its own. The caller (experiments/_framework.py)
// enumerates every run it wants -- seed plus a set of JSON Pointer
// overrides against the base scenario -- and this executes that list. Any
// sweep shape, from a regular grid to an adaptive search, is just a
// different list.
//
// Parallelism is in-process rather than one child process per run: a run
// spends a large fraction of its wall time in JSBSim model load and trim,
// and process startup would add to that rather than replace it. Nothing
// mutable is shared between runs -- each worker owns its World,
// SimContext, and JSBSim instances -- so the only synchronisation is the
// output mutex. `--jobs 1` is the determinism oracle: its results must
// match any other job count exactly.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX  // keep std::min/std::max usable below
#endif
#include <windows.h>
#endif

#include <nlohmann/json.hpp>

#include "FGJSBBase.h"
#include "input_output/FGXMLElement.h"

#include "components/Tag.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "scenario/ParamRegistry.h"
#include "scenario/ScenarioLoader.h"
#include "scenario/ScenarioLoaderJson.h"
#include "sim/SimClock.h"
#include "sim/Environment.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "systems/LaunchSystem.h"
#include "util/Units.h"

namespace {

using Units::kFeetPerMeter;
using Units::kFpsPerKt;
using Units::kRadToDeg;

constexpr int kDefaultRecordDecimation = 12;  // every 12th 120 Hz tick: 10 Hz
constexpr double kDefaultMaxSimTimeS = 400.0;
constexpr long kMaxIntArg = 1000000;  // sanity cap on --jobs / --record-decimation
constexpr int kProgressEveryRuns = 25;
constexpr unsigned int kMaxWorkers = 256;

struct Options {
    std::string specPath;
    std::string outPath;
    int jobs = 0;  // 0 means "use hardware concurrency"
    std::string recordDir;
    int recordDecimation = kDefaultRecordDecimation;
    bool affinity = false;  // opt-in: pin each worker to its own logical core (MSVC builds)
};

struct RunSpec {
    int runId = 0;
    std::optional<uint64_t> seed;  // absent means "use the scenario's own"
    nlohmann::json overrides = nlohmann::json::object();
};

// Every event type worth keeping in the events CSV. The full log runs to
// thousands of entries on a long engagement -- almost all of them
// per-sweep radar chatter -- so only the milestones are written.
bool IsMilestoneEvent(const std::string& type) {
    return type == "LAUNCH" || type == "INTERCEPT" || type == "MISS" || type == "SEEKER_ACTIVE" ||
           type == "SEEKER_LOST" || type == "DATALINK_LOST" || type == "LAUNCH_ZONE_ENTERED" || type == "RADAR_TRACK_ESTABLISHED" ||
           type == "RADAR_TRACK_DROPPED" || type == "RADAR_LOCK" || type == "RADAR_LOCK_LOST" ||
           type == "RADAR_LOCK_RELEASED" || type == "LOFT_END" ||
           // The target's view: a few state changes per run, which the
           // teaching scenarios check (lock warning, missile warning).
           type == "RWR_LOCK" || type == "RWR_LOCK_CLEAR" || type == "RWR_MISSILE" ||
           type == "STRUCTURAL_FAILURE" || type == "GROUND_IMPACT" || type == "SELF_DESTRUCT";
}

void PrintUsage() {
    std::printf(
        "usage: intercept_batch --spec <spec.json> --out <results.csv>\n"
        "                      [--jobs N] [--affinity] [--record <dir>] [--record-decimation N]\n"
        "\n"
        "  --spec               batch spec: scenario path, max_sim_time_s, and a runs[] list\n"
        "  --out                wide results CSV, one row per run. A long-format\n"
        "                       <out>_events.csv is written alongside it, keyed on run_id\n"
        "  --jobs               worker threads (default: hardware concurrency; 1 forces serial)\n"
        "  --affinity           pin each worker to its own logical core (helps on multi-CCX/\n"
        "                       NUMA hosts; measured neutral on a single-CCX desktop).\n"
        "                       MSVC builds only; ignored elsewhere\n"
        "  --record             also write <dir>/run_<id>.csv trajectory samples\n"
        "  --record-decimation  record every Nth tick (default 12, i.e. 10 Hz)\n");
}

// Whole-string integer parse, so "--jobs four" or "--jobs 4x" is an error
// rather than atoi's silent 0 or 4.
bool ParseInt(const char* text, int minValue, int& out) {
    char* end = nullptr;
    long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value < minValue || value > kMaxIntArg) return false;
    out = static_cast<int>(value);
    return true;
}

bool ParseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        bool hasValue = i + 1 < argc;
        if (arg == "--spec" && hasValue) {
            options.specPath = argv[++i];
        } else if (arg == "--out" && hasValue) {
            options.outPath = argv[++i];
        } else if (arg == "--jobs" && hasValue) {
            if (!ParseInt(argv[++i], 0, options.jobs)) {
                std::printf("intercept_batch: --jobs needs a non-negative integer, got '%s'\n\n", argv[i]);
                return false;
            }
        } else if (arg == "--record" && hasValue) {
            options.recordDir = argv[++i];
        } else if (arg == "--record-decimation" && hasValue) {
            if (!ParseInt(argv[++i], 1, options.recordDecimation)) {
                std::printf("intercept_batch: --record-decimation needs a positive integer, got '%s'\n\n", argv[i]);
                return false;
            }
        } else if (arg == "--affinity") {
            options.affinity = true;
        } else {
            std::printf("intercept_batch: unrecognized or incomplete argument '%s'\n\n", arg.c_str());
            return false;
        }
    }
    if (options.specPath.empty() || options.outPath.empty()) {
        std::printf("intercept_batch: --spec and --out are both required\n\n");
        return false;
    }
    return true;
}

// Times are emitted as null rather than a negative sentinel so the column
// reads as an empty cell (NaN once loaded), not as a value that could be
// averaged into a mean.
nlohmann::json OptionalTime(double simTime) {
    if (simTime < 0.0) return nullptr;
    return simTime;
}

// Same idea for a value that only exists in some outcomes.
nlohmann::json OptionalValue(bool present, double value) {
    if (!present) return nullptr;
    return value;
}

// The wide table's columns, in the order they appear. Fixed here rather
// than derived from the result object because nlohmann orders keys
// alphabetically, which would scatter related fields across the sheet.
// Override columns are spliced in after `seed` -- see OverrideColumns.
const char* const kScalarColumnsHead[] = {"run_id", "seed"};
const char* const kScalarColumnsTail[] = {
    "surface_wind_kts", "surface_wind_from_deg",
    "launched",       "launch_time_s",   "launch_range_m", "first_detection_time_s", "firing_solution_time_s",
    "seeker_active_time_s", "seeker_lost_time_s",
    "datalink_error_at_handoff_m", "datalink_staleness_at_handoff_s", "resolved", "hit", "armed",
    "resolution_time_s",    "miss_distance_m",    "pk",        "impact_region",       "impact_x_ft",
    "impact_y_ft",          "impact_z_ft",        "sim_duration_s", "timed_out",      "n_events",
    "wall_ms",              "error"};

// Column name for an override: the pointer's last segment, which is what
// makes `df.groupby("nav_gain")` work. Two different pointers can end
// in the same segment, so those fall back to the full dotted path.
std::vector<std::pair<std::string, std::string>> OverrideColumns(const std::vector<RunSpec>& runs) {
    std::vector<std::string> pointers;
    for (const RunSpec& run : runs) {
        for (auto& [pointer, value] : run.overrides.items()) {
            if (std::find(pointers.begin(), pointers.end(), pointer) == pointers.end()) pointers.push_back(pointer);
        }
    }
    std::sort(pointers.begin(), pointers.end());

    std::vector<std::pair<std::string, std::string>> columns;  // (pointer, column name)
    for (const std::string& pointer : pointers) {
        std::string leaf = pointer.substr(pointer.rfind('/') + 1);
        bool collides = false;
        for (const std::string& other : pointers) {
            if (other != pointer && other.substr(other.rfind('/') + 1) == leaf) collides = true;
        }
        if (collides) {
            leaf = pointer.substr(1);
            std::replace(leaf.begin(), leaf.end(), '/', '.');
        }
        columns.emplace_back(pointer, leaf);
    }
    return columns;
}

// Empty if `pointer` is a legal override for `scenario`, else why not.
// Assigning through a JSON pointer creates any missing key, so without
// this a typo'd pointer would silently produce a sweep of identical runs.
// Legal means it already exists in the scenario, or it names a known
// ParamRegistry field -- an optional key, possibly in an optional block,
// that the file leaves at its default -- on an array element that exists.
std::string ValidateOverride(const std::string& pointer, const nlohmann::json& scenario) {
    nlohmann::json::json_pointer jsonPointer;
    try {
        jsonPointer = nlohmann::json::json_pointer(pointer);
    } catch (const std::exception& e) {
        return "malformed override pointer '" + pointer + "': " + e.what();
    }
    const ParamRegistry::ParamDef* param = ParamRegistry::Find(pointer);
    if (param != nullptr && !param->knob) {
        return "override '" + pointer + "' is a type value, not a teaching knob: edit the type file instead "
               "(docs/SCHEMA.md lists the knobs)";
    }
    if (scenario.contains(jsonPointer)) return "";
    if (param == nullptr) {
        return "override '" + pointer + "' matches nothing in the scenario and is not a known parameter "
               "(see docs/SCHEMA.md)";
    }
    // A missing object is created harmlessly, but a missing array element
    // would be an entity the scenario does not have.
    for (nlohmann::json::json_pointer prefix = jsonPointer; !prefix.empty(); prefix = prefix.parent_pointer()) {
        const std::string& segment = prefix.back();
        bool isIndex = !segment.empty() && std::all_of(segment.begin(), segment.end(),
                                                         [](char c) { return c >= '0' && c <= '9'; });
        if (isIndex && !scenario.contains(prefix)) {
            return "override '" + pointer + "' refers to '" + prefix.to_string() + "', which the scenario lacks";
        }
    }
    return "";
}

// RFC 4180 quoting, applied only where needed so the common numeric case
// stays unquoted and readable.
std::string CsvEscape(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) return text;
    std::string quoted = "\"";
    for (char c : text) {
        if (c == '"') quoted += '"';
        quoted += c;
    }
    return quoted + '"';
}

// Booleans become 1/0 rather than true/false so a column of them averages
// directly -- df.hit.mean() is the empirical Pk -- and still reads
// naturally in a spreadsheet. Null, and a NaN or infinity (which JSON cannot
// write, and would dump as the text "null"), become an empty cell. An array
// or object override is quoted: its commas must not split the row.
std::string CsvCell(const nlohmann::json& value) {
    if (value.is_null()) return "";
    if (value.is_boolean()) return value.get<bool>() ? "1" : "0";
    if (value.is_string()) return CsvEscape(value.get<std::string>());
    if (value.is_number_float() && !std::isfinite(value.get<double>())) return "";
    if (value.is_structured()) return CsvEscape(value.dump());
    return value.dump();
}

// One CSV per run, positions only. Deliberately not JSON: a trajectory is
// a dense uniform table, and at 10 Hz over a 150s engagement this is
// already the largest artifact a sweep produces.
struct TrajectoryRecorder {
    std::ofstream file;
    bool enabled = false;

    void Open(const std::string& dir, int runId) {
        const std::string path = dir + "/run_" + std::to_string(runId) + ".csv";
        file.open(path);
        enabled = file.is_open();
        if (!enabled) throw std::runtime_error("cannot write trajectory file " + path);
        // The default six significant figures write 1052345.6 ft as
        // 1.05235e+06, which is no longer a position.
        file << std::setprecision(12);
        file << "sim_time_s,entity,name,x_ft,y_ft,z_ft,roll_deg,pitch_deg,yaw_deg\n";
    }

    void Sample(World& world, double simTime) {
        if (!enabled) return;
        ComponentStore<Transform>& transforms = world.Components<Transform>();
        ComponentStore<Tag>& tags = world.Components<Tag>();
        for (auto& [entity, transform] : transforms) {
            const Tag* tag = tags.TryGet(entity);
            file << simTime << ',' << entity << ',' << (tag ? CsvEscape(tag->name) : "") << ',' << transform.pos.x() << ',' << transform.pos.y() << ','
                 << transform.pos.z() << ',' << transform.euler.x() << ',' << transform.euler.y() << ','
                 << transform.euler.z() << '\n';
        }
    }
};

// An aircraft's start altitude and heading are written twice, in its
// transform and in its jsbsim block, and the loader insists the two agree.
// An override of one is carried to the other (as the GUI's save does), so
// either key sweeps the same start.
void MirrorPlacementOverride(nlohmann::json& scenario, const std::string& pointer, const nlohmann::json& value) {
    static const std::regex kPlacement(
        R"(^(/entities/\d+)/(transform/pos/1|transform/heading|components/jsbsim/altitude_ft|components/jsbsim/heading_deg)$)");
    std::smatch match;
    if (!std::regex_match(pointer, match, kPlacement)) return;
    const std::string key = match[2];
    std::string twin;
    if (key == "transform/pos/1") twin = "components/jsbsim/altitude_ft";
    else if (key == "transform/heading") twin = "components/jsbsim/heading_deg";
    else if (key == "components/jsbsim/altitude_ft") twin = "transform/pos/1";
    else twin = "transform/heading";
    nlohmann::json::json_pointer twinPointer(match[1].str() + "/" + twin);
    // An entity with no jsbsim block has no twin to carry it to.
    if (scenario.contains(twinPointer)) scenario[twinPointer] = value;
}

nlohmann::json ExecuteRun(const RunSpec& run, const nlohmann::json& baseScenario, double maxSimTime,
                          const Options& options) {
    std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();

    // Every pointer was checked by ValidateOverride before any run started.
    nlohmann::json scenarioJson = baseScenario;
    for (auto& [pointer, value] : run.overrides.items()) {
        scenarioJson[nlohmann::json::json_pointer(pointer)] = value;
        MirrorPlacementOverride(scenarioJson, pointer, value);
    }

    World world;
    SimContext simContext;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(scenarioJson, world);
    SimLoop::BeginRun(scenario, simContext, run.seed.value_or(scenario.seed));

    LaunchSystem::InitFromTags(world, scenario);

    TrajectoryRecorder recorder;
    if (!options.recordDir.empty()) recorder.Open(options.recordDir, run.runId);

    double simTime = 0.0;
    long long tick = 0;
    recorder.Sample(world, simTime);
    while (simTime < maxSimTime && !simContext.outcome.resolved) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
        ++tick;
        if (tick % options.recordDecimation == 0) recorder.Sample(world, simTime);
    }

    const EngagementOutcome& outcome = simContext.outcome;
    nlohmann::json result;
    result["run_id"] = run.runId;
    result["seed"] = run.seed.value_or(scenario.seed);
    result["sim_duration_s"] = simTime;
    result["timed_out"] = !outcome.resolved;

    // The wind this run flew (one steady wind, the same at every altitude).
    Vec3 surface = WindFps(scenario.environment);
    double surfaceKts = std::hypot(surface.x(), surface.z()) / kFpsPerKt;
    result["surface_wind_kts"] = surfaceKts;
    result["surface_wind_from_deg"] = OptionalValue(
        surfaceKts > 0.0, std::fmod(std::atan2(-surface.x(), -surface.z()) * kRadToDeg + 360.0, 360.0));

    result["launched"] = outcome.launched;
    result["launch_time_s"] = OptionalTime(outcome.launchSimTime);
    result["launch_range_m"] = OptionalValue(outcome.launched, outcome.launchRangeFt / kFeetPerMeter);
    result["first_detection_time_s"] = OptionalTime(outcome.firstDetectionSimTime);
    result["firing_solution_time_s"] = OptionalTime(outcome.firingSolutionSimTime);
    result["seeker_active_time_s"] = OptionalTime(outcome.seekerActiveSimTime);
    result["seeker_lost_time_s"] = OptionalTime(outcome.seekerLostSimTime);
    // Negative means "never measured": no handoff happened.
    result["datalink_error_at_handoff_m"] =
        OptionalValue(outcome.datalinkErrorAtHandoffFt >= 0.0, outcome.datalinkErrorAtHandoffFt / kFeetPerMeter);
    result["datalink_staleness_at_handoff_s"] =
        OptionalValue(outcome.datalinkStalenessAtHandoffS >= 0.0, outcome.datalinkStalenessAtHandoffS);

    result["resolved"] = outcome.resolved;
    result["hit"] = outcome.hit;
    result["armed"] = outcome.armed;
    result["resolution_time_s"] = OptionalTime(outcome.resolutionSimTime);
    result["miss_distance_m"] = OptionalValue(outcome.resolved, outcome.missDistanceFt / kFeetPerMeter);
    result["pk"] = OptionalValue(outcome.resolved, outcome.pk);
    result["impact_region"] = outcome.impactRegion;
    // Where on the airframe, in the target's body frame. Only meaningful
    // once the engagement resolved -- before that there is no impact point.
    result["impact_x_ft"] = OptionalValue(outcome.resolved, outcome.impactBodyFt.x());
    result["impact_y_ft"] = OptionalValue(outcome.resolved, outcome.impactBodyFt.y());
    result["impact_z_ft"] = OptionalValue(outcome.resolved, outcome.impactBodyFt.z());

    nlohmann::json events = nlohmann::json::array();
    for (const SimEvent& event : simContext.eventLog.Events()) {
        if (!IsMilestoneEvent(event.type)) continue;
        events.push_back({{"t", event.simTime}, {"type", event.type}, {"entity", event.entity}, {"data", event.data}});
    }
    result["events"] = events;
    result["n_events"] = events.size();

    result["wall_ms"] =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseArgs(argc, argv, options)) {
        PrintUsage();
        return 2;
    }

    // JSBSim prints a full aircraft config dump and trim report per
    // instance, which across hundreds of runs buries the sweep's own
    // output. Set before any FGFDMExec exists, so it is not a write
    // racing the worker threads: FGFDMExec's constructor only touches
    // debug_lvl if JSBSIM_DEBUG is set, which stays available as the
    // escape hatch for diagnosing a single bad run.
    JSBSim::FGJSBBase::debug_lvl = 0;

    nlohmann::json spec;
    nlohmann::json baseScenario;
    try {
        spec = ScenarioLoader::ReadJsonFile(options.specPath);
        baseScenario = ScenarioLoader::ReadJsonFile(spec.at("scenario").get<std::string>());
    } catch (const std::exception& e) {
        std::printf("intercept_batch: %s\n", e.what());
        return 1;
    }

    // A malformed spec is reported, not thrown: an exception escaping main
    // terminates with no message at all.
    double maxSimTime = kDefaultMaxSimTimeS;
    std::vector<RunSpec> runs;
    try {
        maxSimTime = spec.value("max_sim_time_s", kDefaultMaxSimTimeS);
        // run_id defaults to the run's position in the list, so a
        // hand-written spec can omit it.
        int position = 0;
        for (const nlohmann::json& runJson : spec.at("runs")) {
            RunSpec run;
            run.runId = runJson.value("run_id", position);
            if (runJson.contains("seed")) {
                // A negative seed would wrap to 2^64 - n, a fraction would truncate.
                const nlohmann::json& seedJson = runJson.at("seed");
                if (!seedJson.is_number_unsigned() && !(seedJson.is_number_integer() && seedJson.get<int64_t>() >= 0)) {
                    throw std::runtime_error("run " + std::to_string(run.runId) +
                                             ": seed must be a non-negative integer, got " + seedJson.dump());
                }
                run.seed = seedJson.get<uint64_t>();
            }
            if (runJson.contains("overrides")) run.overrides = runJson.at("overrides");
            // The run's seed is the "seed" field; an override of the
            // scenario's own seed would be shadowed by it, and silently do
            // nothing when the run carries one.
            if (run.overrides.is_object() && run.overrides.contains("/seed")) {
                throw std::runtime_error("run " + std::to_string(run.runId) +
                                         ": set the run's own \"seed\" field, not an override of /seed");
            }
            for (const RunSpec& earlier : runs) {
                if (earlier.runId == run.runId) {
                    throw std::runtime_error("run_id " + std::to_string(run.runId) +
                                             " appears more than once: rows could not be told apart");
                }
            }
            runs.push_back(std::move(run));
            ++position;
        }
    } catch (const std::exception& e) {
        std::printf("intercept_batch: malformed spec %s: %s\n", options.specPath.c_str(), e.what());
        return 1;
    }
    if (runs.empty()) {
        std::printf("intercept_batch: spec contains no runs\n");
        return 1;
    }
    std::vector<std::pair<std::string, std::string>> overrideColumns = OverrideColumns(runs);
    for (const auto& [pointer, name] : overrideColumns) {
        std::string error = ValidateOverride(pointer, baseScenario);
        if (!error.empty()) {
            std::printf("intercept_batch: %s\n", error.c_str());
            return 1;
        }
    }

    if (!options.recordDir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(options.recordDir, ec);
        if (ec || !std::filesystem::is_directory(options.recordDir)) {
            std::printf("intercept_batch: cannot create the --record directory %s\n", options.recordDir.c_str());
            return 1;
        }
    }

    std::ofstream out(options.outPath);
    if (!out.is_open()) {
        std::printf("intercept_batch: cannot write to %s\n", options.outPath.c_str());
        return 1;
    }

    // Events go to a sibling long-format table rather than being crushed
    // into a cell of the wide one: one row per event, joinable back on
    // run_id. Keeps the wide table one clean row per run while preserving
    // the detail the typed columns do not carry (a SEEKER_LOST reason,
    // the seeker's acquisition range).
    std::filesystem::path eventsPath = options.outPath;
    eventsPath.replace_extension();
    eventsPath += "_events.csv";
    std::ofstream eventsOut(eventsPath);
    if (!eventsOut.is_open()) {
        std::printf("intercept_batch: cannot write to %s\n", eventsPath.string().c_str());
        return 1;
    }
    eventsOut << "run_id,sim_time_s,type,entity,data\n";

    // Override columns read from the run's own spec, not from the result,
    // so a failed run still records what it was asked to do.
    struct Column {
        std::string name;
        std::string overridePointer;  // empty for a result column
    };
    std::vector<Column> columns;
    for (const char* column : kScalarColumnsHead) columns.push_back({column, ""});
    for (const auto& [pointer, name] : overrideColumns) columns.push_back({name, pointer});
    for (const char* column : kScalarColumnsTail) columns.push_back({column, ""});
    for (size_t i = 0; i < columns.size(); ++i) out << (i ? "," : "") << columns[i].name;
    out << '\n';

    unsigned int jobs = options.jobs > 0 ? static_cast<unsigned int>(options.jobs) : std::thread::hardware_concurrency();
    if (jobs == 0) jobs = 1;
    jobs = std::min<unsigned int>(jobs, static_cast<unsigned int>(runs.size()));
    jobs = std::min<unsigned int>(jobs, kMaxWorkers);  // a thread per run helps nothing

    std::printf("intercept_batch: %zu runs of %s across %u thread(s)\n", runs.size(),
                spec.at("scenario").get<std::string>().c_str(), jobs);

    std::atomic<size_t> nextRun{0};
    std::atomic<int> failures{0};
    std::atomic<int> completed{0};
    std::mutex outputMutex;
    // A run's text, held until every earlier run has been written, so the
    // files are in run order whatever order the workers finish in and two
    // job counts give the same bytes (the wall-clock column aside).
    struct Pending {
        std::string row;
        std::string events;
        bool ready = false;
    };
    std::vector<Pending> pending(runs.size());
    size_t nextToWrite = 0;
    std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();

    auto worker = [&]() {
        for (;;) {
            size_t index = nextRun.fetch_add(1);
            if (index >= runs.size()) return;
            const RunSpec& run = runs[index];

            nlohmann::json result;
            try {
                result = ExecuteRun(run, baseScenario, maxSimTime, options);
            } catch (const std::exception& e) {
                // One bad run must not abort the sweep -- the failure gets
                // its own row, with every outcome column empty, so the
                // sweep still accounts for every run_id.
                result = {{"run_id", run.runId}, {"error", e.what()}};
                failures.fetch_add(1);
            }

            std::string row;
            for (size_t i = 0; i < columns.size(); ++i) {
                if (i) row += ',';
                const Column& column = columns[i];
                const nlohmann::json& source = column.overridePointer.empty() ? result : run.overrides;
                const std::string& key = column.overridePointer.empty() ? column.name : column.overridePointer;
                if (source.contains(key)) row += CsvCell(source.at(key));
            }
            row += '\n';
            std::string eventRows;
            if (result.contains("events")) {
                for (const nlohmann::json& event : result.at("events")) {
                    eventRows += std::to_string(run.runId) + ',' + CsvCell(event.at("t")) + ',' +
                                 CsvCell(event.at("type")) + ',' + CsvCell(event.at("entity")) + ',' +
                                 CsvCell(event.at("data")) + '\n';
                }
            }

            std::lock_guard<std::mutex> lock(outputMutex);
            pending[index] = Pending{std::move(row), std::move(eventRows), true};
            while (nextToWrite < pending.size() && pending[nextToWrite].ready) {
                out << pending[nextToWrite].row;
                eventsOut << pending[nextToWrite].events;
                pending[nextToWrite] = Pending{};
                ++nextToWrite;
            }

            int done = completed.fetch_add(1) + 1;
            if (done % kProgressEveryRuns == 0 || done == static_cast<int>(runs.size())) {
                std::printf("  %d/%zu\n", done, runs.size());
                std::fflush(stdout);
            }
        }
    };

    // JSBSim fills its XML unit-conversion table lazily, flagging it ready
    // before filling it, so workers loading models concurrently can see a
    // half-built table ("DEG cannot be converted to RAD"). Building one
    // Element here fills it before any worker starts.
    { JSBSim::Element warmUnitTable("warm_unit_table"); }

    std::vector<std::thread> pool;
    for (unsigned int i = 0; i < jobs; ++i) pool.emplace_back(worker);

    // Pin each worker to its own logical core so the scheduler stops
    // migrating compute-bound workers between cores (each migration throws
    // away warm cache). Pure scheduling -- every run still owns its own
    // world/RNG, so results are byte-identical with or without this. MSVC
    // only, where std::thread::native_handle() is a Win32 HANDLE (MinGW's
    // is a pthread id); a no-op elsewhere.
#ifdef _MSC_VER
    if (options.affinity) {
        // A single affinity mask covers at most 64 cores.
        unsigned int nprocs = std::thread::hardware_concurrency();
        if (nprocs == 0) nprocs = jobs;
        nprocs = std::min(nprocs, static_cast<unsigned int>(sizeof(DWORD_PTR) * CHAR_BIT));
        for (unsigned int i = 0; i < pool.size(); ++i) {
            DWORD_PTR mask = static_cast<DWORD_PTR>(1) << (i % nprocs);
            SetThreadAffinityMask(pool[i].native_handle(), mask);
        }
    }
#endif

    for (std::thread& thread : pool) thread.join();
    out.close();
    eventsOut.close();

    double elapsedS = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    std::printf("intercept_batch: %zu runs in %.1fs (%.2fs/run wall, %.1f runs/s)\n", runs.size(), elapsedS,
                elapsedS / runs.size(), runs.size() / elapsedS);
    std::printf("  %s\n  %s\n", options.outPath.c_str(), eventsPath.string().c_str());
    if (failures > 0) {
        std::printf("intercept_batch: %d run(s) FAILED -- see the \"error\" column\n", failures.load());
        return 1;
    }
    return 0;
}
