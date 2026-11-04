// intercept_batch is how every sweep and batch_compare gate runs, so it is
// checked here as a user runs it: the built executable, a spec file, and
// the two CSVs it writes. The load-bearing guarantees are that `--jobs 1`
// and any other job count give identical results, that override columns
// record what each run was asked to do, that a bad run gets its own error
// row instead of aborting the sweep, and that a bad argument, spec or
// override pointer fails before any run with a message.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#ifndef _WIN32
#include <sys/wait.h>
#endif

namespace {

namespace fs = std::filesystem;

const char* const kScenario = "scenarios/1_core_01_first_shot.json";

struct BatchResult {
    int exitCode = 0;
    std::string output;
};

// A fresh, empty directory per test under GoogleTest's temp dir.
fs::path FreshDir(const std::string& name) {
    fs::path dir = fs::path(testing::TempDir()) / ("intercept_batch_" + name);
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

BatchResult RunBatch(const std::string& args, const fs::path& dir) {
    fs::path log = dir / "stdout.txt";
    std::string command = "\"" + std::string(INTERCEPT_BATCH_EXE) + "\" " + args + " > \"" + log.string() + "\" 2>&1";
#ifdef _WIN32
    // cmd /c strips the first and last quote of a line that starts with
    // one; an outer pair keeps the exe's own quotes intact.
    command = "\"" + command + "\"";
#endif
    BatchResult result;
    result.exitCode = std::system(command.c_str());
#ifndef _WIN32
    // POSIX system() returns the wait status, not the exit code.
    if (result.exitCode != -1 && WIFEXITED(result.exitCode)) result.exitCode = WEXITSTATUS(result.exitCode);
#endif
    std::ifstream in(log);
    std::stringstream text;
    text << in.rdbuf();
    result.output = text.str();
    return result;
}

fs::path WriteSpec(const fs::path& dir, const nlohmann::json& spec) {
    fs::path path = dir / "spec.json";
    std::ofstream(path) << spec.dump(2);
    return path;
}

std::string Quoted(const fs::path& path) { return "\"" + path.string() + "\""; }

// RFC 4180 fields of one line (the writer quotes only where needed).
std::vector<std::string> SplitCsvLine(const std::string& line) {
    std::vector<std::string> fields(1);
    bool quoted = false;
    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (quoted && c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
            fields.back() += '"';
            ++i;
        } else if (c == '"') {
            quoted = !quoted;
        } else if (c == ',' && !quoted) {
            fields.emplace_back();
        } else {
            fields.back() += c;
        }
    }
    return fields;
}

// Rows keyed by column name, in file order.
struct Csv {
    std::vector<std::string> header;
    std::vector<std::map<std::string, std::string>> rows;
};

Csv ReadCsv(const fs::path& path) {
    Csv csv;
    std::ifstream in(path);
    std::string line;
    if (!std::getline(in, line)) return csv;
    csv.header = SplitCsvLine(line);
    while (std::getline(in, line)) {
        std::vector<std::string> fields = SplitCsvLine(line);
        std::map<std::string, std::string> row;
        for (size_t i = 0; i < csv.header.size() && i < fields.size(); ++i) row[csv.header[i]] = fields[i];
        csv.rows.push_back(row);
    }
    return csv;
}

// Everything but the wall-clock column, ordered by run_id: threads finish
// in any order.
std::vector<std::map<std::string, std::string>> Comparable(Csv csv) {
    for (auto& row : csv.rows) row.erase("wall_ms");
    std::sort(csv.rows.begin(), csv.rows.end(),
              [](const auto& a, const auto& b) { return std::stoi(a.at("run_id")) < std::stoi(b.at("run_id")); });
    return csv.rows;
}

std::vector<std::string> SortedLines(const fs::path& path) {
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    std::sort(lines.begin(), lines.end());
    return lines;
}

nlohmann::json ThreeRunSpec() {
    return {{"scenario", kScenario},
            {"max_sim_time_s", 150.0},
            {"runs",
             {{{"run_id", 0}, {"seed", 1}},
              {{"run_id", 1}, {"seed", 2}, {"overrides", {{"/missile_doctrine/nav_gain", 3.0}}}},
              {{"run_id", 2}, {"seed", 1}, {"overrides", {{"/missile_doctrine/launch_mode", "STT"}}}}}}};
}

}  // namespace

TEST(BatchCli, JobCountDoesNotChangeResultsAndOverridesGetColumns) {
    fs::path dir = FreshDir("jobs");
    fs::path spec = WriteSpec(dir, ThreeRunSpec());
    fs::path serial = dir / "serial.csv";
    fs::path parallel = dir / "parallel.csv";
    fs::path record = dir / "trajectories";

    BatchResult one = RunBatch("--spec " + Quoted(spec) + " --out " + Quoted(serial) + " --jobs 1 --record " +
                                   Quoted(record) + " --record-decimation 120",
                               dir);
    ASSERT_EQ(one.exitCode, 0) << one.output;
    BatchResult two = RunBatch("--spec " + Quoted(spec) + " --out " + Quoted(parallel) + " --jobs 2 --affinity", dir);
    ASSERT_EQ(two.exitCode, 0) << two.output;

    Csv csv = ReadCsv(serial);
    ASSERT_EQ(csv.rows.size(), 3u);
    // Override columns sit after seed, named by the pointer's last segment.
    ASSERT_GE(csv.header.size(), 4u);
    EXPECT_EQ(csv.header[2], "launch_mode");
    EXPECT_EQ(csv.header[3], "nav_gain");
    EXPECT_EQ(Comparable(csv), Comparable(ReadCsv(parallel)));
    EXPECT_EQ(SortedLines(dir / "serial_events.csv"), SortedLines(dir / "parallel_events.csv"));

    for (const auto& row : Comparable(csv)) {
        EXPECT_EQ(row.at("resolved"), "1") << "run " << row.at("run_id");
        EXPECT_EQ(row.at("error"), "") << "run " << row.at("run_id");
    }
    const auto rows = Comparable(csv);
    EXPECT_EQ(rows[0].at("nav_gain"), "");  // no override: empty, not the scenario's value
    EXPECT_EQ(rows[1].at("nav_gain"), "3.0");
    EXPECT_EQ(rows[2].at("launch_mode"), "STT");

    Csv events = ReadCsv(dir / "serial_events.csv");
    EXPECT_EQ(events.header, (std::vector<std::string>{"run_id", "sim_time_s", "type", "entity", "data"}));
    EXPECT_TRUE(std::any_of(events.rows.begin(), events.rows.end(),
                            [](const auto& row) { return row.at("type") == "LAUNCH"; }));
    for (int run = 0; run < 3; ++run) {
        Csv trajectory = ReadCsv(record / ("run_" + std::to_string(run) + ".csv"));
        EXPECT_EQ(trajectory.header.front(), "sim_time_s");
        EXPECT_GT(trajectory.rows.size(), 10u) << "run " << run;
    }
}

// An aircraft's start altitude and heading are written twice in a scenario
// (transform and jsbsim). A sweep over either key must move the aircraft:
// overriding only the transform would leave every run flying the file's
// start and still label the rows with the override.
TEST(BatchCli, PlacementOverridesMoveTheAircraftWhicheverKeyNamesThem) {
    fs::path dir = FreshDir("placement");
    const nlohmann::json base = nlohmann::json::parse(std::ifstream(kScenario));
    const double baseHeading = base["entities"][0]["transform"]["heading"].get<double>();
    const double newHeading = std::fmod(baseHeading + 90.0, 360.0);
    nlohmann::json spec = {
        {"scenario", kScenario},
        {"max_sim_time_s", 5.0},
        {"runs",
         {{{"run_id", 0}, {"seed", 1}},
          {{"run_id", 1}, {"seed", 1}, {"overrides", {{"/entities/0/transform/pos/1", 35000.0}}}},
          {{"run_id", 2}, {"seed", 1}, {"overrides", {{"/entities/0/components/jsbsim/altitude_ft", 35000.0}}}},
          {{"run_id", 3}, {"seed", 1}, {"overrides", {{"/entities/0/transform/heading", newHeading}}}},
          {{"run_id", 4}, {"seed", 1}, {"overrides", {{"/entities/0/components/jsbsim/heading_deg", newHeading}}}}}}};
    fs::path record = dir / "trajectories";
    BatchResult result = RunBatch("--spec " + Quoted(WriteSpec(dir, spec)) + " --out " + Quoted(dir / "out.csv") +
                                      " --jobs 1 --record " + Quoted(record) + " --record-decimation 120",
                                  dir);
    ASSERT_EQ(result.exitCode, 0) << result.output;

    auto pursuerAtThreeSeconds = [&](int run) {
        Csv trajectory = ReadCsv(record / ("run_" + std::to_string(run) + ".csv"));
        for (const auto& row : trajectory.rows) {
            if (row.at("name") == "Pursuer" && std::stod(row.at("sim_time_s")) >= 3.0) return row;
        }
        ADD_FAILURE() << "run " << run << ": no Pursuer sample at 3 s";
        return std::map<std::string, std::string>{};
    };
    auto yawError = [](double yaw, double want) { return std::fabs(std::remainder(yaw - want, 360.0)); };

    const double baseAltitude = base["entities"][0]["transform"]["pos"][1].get<double>();
    EXPECT_NEAR(std::stod(pursuerAtThreeSeconds(0).at("y_ft")), baseAltitude, 300.0);
    EXPECT_NEAR(std::stod(pursuerAtThreeSeconds(1).at("y_ft")), 35000.0, 300.0) << "transform key";
    EXPECT_NEAR(std::stod(pursuerAtThreeSeconds(2).at("y_ft")), 35000.0, 300.0) << "jsbsim key";
    EXPECT_LT(yawError(std::stod(pursuerAtThreeSeconds(0).at("yaw_deg")), baseHeading), 5.0);
    EXPECT_LT(yawError(std::stod(pursuerAtThreeSeconds(3).at("yaw_deg")), newHeading), 5.0) << "transform key";
    EXPECT_LT(yawError(std::stod(pursuerAtThreeSeconds(4).at("yaw_deg")), newHeading), 5.0) << "jsbsim key";
}

std::string Slurp(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

// Every file a sweep writes is in run order whatever the job count, so the
// outputs of --jobs 1 and --jobs N can be compared byte for byte (the
// wall-clock column aside) rather than only after sorting.
TEST(BatchCli, RowsAreWrittenInRunOrderWhateverTheJobCount) {
    fs::path dir = FreshDir("row_order");
    nlohmann::json runs = nlohmann::json::array();
    for (int i = 0; i < 12; ++i) runs.push_back({{"run_id", 100 + i}, {"seed", i + 1}});
    nlohmann::json spec = {{"scenario", kScenario}, {"max_sim_time_s", 2.0}, {"runs", runs}};
    fs::path specPath = WriteSpec(dir, spec);
    ASSERT_EQ(RunBatch("--spec " + Quoted(specPath) + " --out " + Quoted(dir / "serial.csv") + " --jobs 1", dir).exitCode, 0);
    ASSERT_EQ(RunBatch("--spec " + Quoted(specPath) + " --out " + Quoted(dir / "parallel.csv") + " --jobs 4", dir).exitCode,
              0);

    Csv parallel = ReadCsv(dir / "parallel.csv");
    ASSERT_EQ(parallel.rows.size(), 12u);
    for (size_t i = 0; i < parallel.rows.size(); ++i) {
        EXPECT_EQ(parallel.rows[i].at("run_id"), std::to_string(100 + i)) << "row " << i;
    }
    // Same rows in the same order; only the wall-clock column may differ.
    auto withoutWallClock = [](Csv csv) {
        for (auto& row : csv.rows) row.erase("wall_ms");
        return csv.rows;
    };
    EXPECT_EQ(withoutWallClock(ReadCsv(dir / "serial.csv")), withoutWallClock(parallel));
    EXPECT_EQ(Slurp(dir / "serial_events.csv"), Slurp(dir / "parallel_events.csv"));
}

// An override can be a whole array. Its commas are quoted, so the row keeps
// the same number of fields as the header and the cell reads back whole.
TEST(BatchCli, AnArrayOverrideStaysOneCsvCell) {
    fs::path dir = FreshDir("array_cell");
    const nlohmann::json base = nlohmann::json::parse(std::ifstream(kScenario));
    nlohmann::json pos = base["entities"][0]["transform"]["pos"];
    pos[0] = pos[0].get<double>() + 100.0;
    nlohmann::json spec = {{"scenario", kScenario},
                           {"max_sim_time_s", 1.0},
                           {"runs", {{{"seed", 1}, {"overrides", {{"/entities/0/transform/pos", pos}}}}}}};
    fs::path out = dir / "out.csv";
    ASSERT_EQ(RunBatch("--spec " + Quoted(WriteSpec(dir, spec)) + " --out " + Quoted(out), dir).exitCode, 0);

    std::ifstream in(out);
    std::string header, row;
    std::getline(in, header);
    std::getline(in, row);
    EXPECT_EQ(SplitCsvLine(row).size(), SplitCsvLine(header).size()) << row;
    Csv csv = ReadCsv(out);
    ASSERT_EQ(csv.rows.size(), 1u);
    EXPECT_EQ(nlohmann::json::parse(csv.rows[0].at("pos")), pos);
}

// A trajectory sample keeps enough digits to be a position: at six
// significant figures 1052345.6 ft printed as 1.05235e+06.
TEST(BatchCli, TrajectoryPositionsKeepTheirPrecision) {
    fs::path dir = FreshDir("record_precision");
    nlohmann::json spec = {{"scenario", kScenario}, {"max_sim_time_s", 3.0}, {"runs", {{{"seed", 1}}}}};
    fs::path record = dir / "trajectories";
    ASSERT_EQ(RunBatch("--spec " + Quoted(WriteSpec(dir, spec)) + " --out " + Quoted(dir / "out.csv") +
                           " --record " + Quoted(record) + " --record-decimation 120",
                       dir)
                  .exitCode,
              0);
    Csv trajectory = ReadCsv(record / "run_0.csv");
    ASSERT_GT(trajectory.rows.size(), 2u);
    bool sawFractionalPosition = false;
    for (const auto& row : trajectory.rows) {
        const std::string& x = row.at("x_ft");
        EXPECT_EQ(x.find('e'), std::string::npos) << "x_ft in scientific notation: " << x;
        size_t point = x.find('.');
        if (std::fabs(std::stod(x)) >= 100000.0 && point != std::string::npos && x.size() - point - 1 >= 3) {
            sawFractionalPosition = true;
        }
    }
    EXPECT_TRUE(sawFractionalPosition) << "no position of 1e5 ft or more kept three decimal places";
}

TEST(BatchCli, ARecordDirectoryThatCannotBeCreatedFailsBeforeAnyRun) {
    fs::path dir = FreshDir("record_unwritable");
    std::ofstream(dir / "afile.txt") << "not a directory";
    nlohmann::json spec = {{"scenario", kScenario}, {"max_sim_time_s", 1.0}, {"runs", {{{"seed", 1}}}}};
    fs::path out = dir / "out.csv";
    BatchResult result = RunBatch("--spec " + Quoted(WriteSpec(dir, spec)) + " --out " + Quoted(out) + " --record " +
                                      Quoted(dir / "afile.txt" / "sub"),
                                  dir);
    EXPECT_EQ(result.exitCode, 1);
    EXPECT_NE(result.output.find("cannot create"), std::string::npos) << result.output;
}

TEST(BatchCli, AFailedRunGetsAnErrorRowAndTheSweepCarriesOn) {
    fs::path dir = FreshDir("failed_run");
    nlohmann::json spec = {{"scenario", kScenario},
                           {"max_sim_time_s", 1.0},
                           {"runs",
                            {{{"seed", 1}},
                             {{"seed", 1}, {"overrides", {{"/missile_doctrine/launch_mode", "BOGUS"}}}}}}};
    fs::path out = dir / "out.csv";
    BatchResult result = RunBatch("--spec " + Quoted(WriteSpec(dir, spec)) + " --out " + Quoted(out), dir);
    EXPECT_EQ(result.exitCode, 1);
    EXPECT_NE(result.output.find("1 run(s) FAILED"), std::string::npos) << result.output;

    auto rows = Comparable(ReadCsv(out));
    ASSERT_EQ(rows.size(), 2u);
    // run_id defaults to the run's position in the list.
    EXPECT_EQ(rows[0].at("error"), "");
    EXPECT_EQ(rows[0].at("timed_out"), "1");
    EXPECT_NE(rows[1].at("error").find("launch_mode"), std::string::npos) << rows[1].at("error");
    EXPECT_EQ(rows[1].at("launch_mode"), "BOGUS");
    EXPECT_EQ(rows[1].at("resolved"), "");
}

TEST(BatchCli, BadArgumentsPrintUsageAndExitTwo) {
    fs::path dir = FreshDir("bad_args");
    const std::vector<std::string> bad = {
        "",                                     // --spec and --out both required
        "--spec s.json",                        // --out missing
        "--spec s.json --out o.csv --jobs four",
        "--spec s.json --out o.csv --jobs -1",
        "--spec s.json --out o.csv --record-decimation 0",
        "--spec s.json --out o.csv --frobnicate",
        "--spec s.json --out",                  // incomplete
    };
    for (const std::string& args : bad) {
        BatchResult result = RunBatch(args, dir);
        EXPECT_EQ(result.exitCode, 2) << args;
        EXPECT_NE(result.output.find("usage: intercept_batch"), std::string::npos) << args;
    }
}

TEST(BatchCli, BadSpecsAndOverridesFailBeforeAnyRun) {
    fs::path dir = FreshDir("bad_spec");
    fs::path out = dir / "out.csv";
    struct Case {
        nlohmann::json spec;
        const char* message;
    };
    auto withOverride = [](const char* pointer, nlohmann::json value) {
        return nlohmann::json{{"scenario", kScenario}, {"runs", {{{"overrides", {{pointer, value}}}}}}};
    };
    const std::vector<Case> cases = {
        {{{"scenario", "scenarios/no_such_file.json"}, {"runs", {{{"seed", 1}}}}}, "no_such_file"},
        {{{"scenario", kScenario}}, "malformed spec"},
        {{{"scenario", kScenario}, {"runs", {{{"seed", "one"}}}}}, "malformed spec"},
        {{{"scenario", kScenario}, {"runs", nlohmann::json::array()}}, "spec contains no runs"},
        {withOverride("/missile_doctrine/nav_gian", 3.0), "matches nothing in the scenario"},
        {withOverride("missile_doctrine", 3.0), "malformed override pointer"},
        // A type value, not a teaching knob.
        {withOverride("/missile_doctrine/seeker/active_seeker/average_power_w", 1.0), "not a teaching knob"},
        // A known knob, but on an entity the scenario does not have.
        {withOverride("/entities/9/components/signature/rcs_scale", 2.0), "which the scenario lacks"},
        // A run's seed is its own field; a /seed override would be ignored.
        {withOverride("/seed", 7), "set the run's own \"seed\""},
        // A negative seed would wrap to 2^64 - 1.
        {{{"scenario", kScenario}, {"runs", {{{"seed", -1}}}}}, "seed must be a non-negative integer"},
        {{{"scenario", kScenario}, {"runs", {{{"seed", 1.5}}}}}, "seed must be a non-negative integer"},
        // Two rows with one id could not be told apart in the results.
        {{{"scenario", kScenario}, {"runs", {{{"run_id", 3}}, {{"run_id", 3}}}}}, "run_id 3 appears more than once"},
    };
    for (const Case& c : cases) {
        fs::remove(out);
        BatchResult result = RunBatch("--spec " + Quoted(WriteSpec(dir, c.spec)) + " --out " + Quoted(out), dir);
        EXPECT_EQ(result.exitCode, 1) << c.spec.dump();
        EXPECT_NE(result.output.find(c.message), std::string::npos) << c.spec.dump() << "\n" << result.output;
        EXPECT_FALSE(fs::exists(out)) << c.spec.dump();
    }

    // An unwritable output path fails cleanly too.
    nlohmann::json spec = {{"scenario", kScenario}, {"runs", {{{"seed", 1}}}}};
    BatchResult result = RunBatch(
        "--spec " + Quoted(WriteSpec(dir, spec)) + " --out " + Quoted(dir / "no_such_dir" / "out.csv"), dir);
    EXPECT_EQ(result.exitCode, 1);
    EXPECT_NE(result.output.find("cannot write to"), std::string::npos) << result.output;
}
