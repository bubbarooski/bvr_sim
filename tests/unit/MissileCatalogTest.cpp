// Missile types load from data/missiles/<id>/missile.json: every field
// reaches the MissileType, a type is parsed once and shared, and a missing
// or malformed type file fails at load naming the file rather than flying
// a missile with zero mass.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "missile/MissileCatalog.h"
#include "missile/MissileEntity.h"
#include "sim/RadarEquation.h"
#include "util/Units.h"

namespace {

// A scratch catalog root holding one type directory with the given file
// contents, removed when the test ends. Named per test: ctest -j runs each
// TEST() as its own process, so a shared root would race.
class ScratchCatalog {
public:
    explicit ScratchCatalog(const std::string& typeJson)
        : root_(std::filesystem::temp_directory_path() /
                ("bvr_sim_missile_catalog_test_" + std::string(testing::UnitTest::GetInstance()->current_test_info()->name()))) {
        std::filesystem::remove_all(root_);
        std::filesystem::create_directories(root_ / "test_type");
        std::ofstream(root_ / "test_type" / "missile.json") << typeJson;
    }
    ~ScratchCatalog() { std::filesystem::remove_all(root_); }
    const std::filesystem::path& Root() const { return root_; }

private:
    std::filesystem::path root_;
};

// A small valid normal_force block with one piece substituted.
std::string NormalForceJson(const std::string& cnMaxRows = "[[0.5, 10], [2, 20]]",
                            const std::string& alphaMaxDeg = "30", const std::string& extra = "") {
    return R"("normal_force": {"alpha_max_deg": )" + alphaMaxDeg + ", " + extra +
           R"("cn_alpha_per_deg_vs_mach": [[0.5, 0.33], [1, 0.3], [2, 0.2]], "cn_max_vs_mach": )" + cnMaxRows + "}";
}

// A complete, valid type file with one value substituted.
std::string TypeJson(const std::string& fovHalfAngleDeg = "45", const std::string& extraAutopilot = "",
                     const std::string& normalForce = NormalForceJson(),
                     const std::string& extraAero = "", const std::string& minAvailableG = "2", const std::string& maxTimeOfFlightS = "120",
                     const std::string& confirmHits = "3") {
    return R"({"name": "x",
        "propulsion": {"boost_duration_s": 3, "boost_thrust_lbf": 5670, "sustain_duration_s": 5,
                       "sustain_thrust_lbf": 3170},
        "mass": {"dry_mass_slug": 6.294, "propellant_mass_slug": 4.121},
        "aerodynamics": {"reference_area_ft2": 0.27, "cd_vs_mach": [[0, 0.3], [1, 0.6]], "structural_g_limit": 30,
                         )" +
           extraAero + normalForce + R"(},
        "seeker": {"mode": "ARH", "fov_half_angle_deg": )" +
           fovHalfAngleDeg + R"(, "memory_coast_time_s": 1.5,
                   "active_seeker": {"average_power_w": 100, "antenna_gain": 148, "frequency_hz": 1e10,
                                     "noise_figure_db": 4, "integration_time_s": 0.01, "system_loss_db": 5,
                                     "pfa": 1e-6, "sweep_period_s": 0.1,
                                     "confirm_hits": )" +
           confirmHits + R"(, "confirm_window": 5, "antenna_diameter_m": 0.15,
                                     "beamwidth_factor": 1.2, "notch_velocity_threshold_mps": 15}},
        "autopilot": {)" +
           extraAutopilot + R"("response_time_s": 0.35, "launch_turn_exit_deg": 15},
        "loft": {"start_range_nmi": 15, "full_range_nmi": 40, "max_climb_deg": 25,
                 "climb_time_constant_s": 2, "min_available_g": 8},
        "fuze": {"arming_range_m": 50, "falloff_width_m": 5},
        "self_destruct": {"min_available_g": )" +
           minAvailableG + R"(, "max_time_of_flight_s": )" + maxTimeOfFlightS + "}}";
}

nlohmann::json RealAim120Type() {
    std::ifstream real("data/missiles/aim120/missile.json");
    return nlohmann::json::parse(real);
}

// Loads the real AIM-120 type with one edit and expects a load error whose
// message contains `expected`.
void ExpectLoadError(const std::function<void(nlohmann::json&)>& edit, const std::string& expected) {
    nlohmann::json type = RealAim120Type();
    edit(type);
    ScratchCatalog scratch(type.dump());
    try {
        MissileCatalog(scratch.Root()).Get("test_type");
        ADD_FAILURE() << "expected a load error containing: " << expected;
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(expected), std::string::npos) << e.what();
    }
}

}  // namespace

TEST(MissileCatalog, LoadsTheAim120Type) {
    std::shared_ptr<const MissileType> aim120 = MissileCatalog::Default().Get("aim120");
    EXPECT_EQ(aim120->id, "aim120");
    EXPECT_DOUBLE_EQ(aim120->propulsion.boostDurationS, 3.0);
    EXPECT_DOUBLE_EQ(aim120->propulsion.boostThrustLbf, 5670.0);
    EXPECT_DOUBLE_EQ(aim120->propulsion.sustainDurationS, 5.0);
    EXPECT_DOUBLE_EQ(aim120->propulsion.sustainThrustLbf, 3170.0);
    EXPECT_DOUBLE_EQ(aim120->mass.dryMassSlug, 6.294);
    EXPECT_DOUBLE_EQ(aim120->mass.propellantMassSlug, 4.121);
    EXPECT_DOUBLE_EQ(aim120->aero.referenceAreaFt2, 0.267888);
    EXPECT_EQ(aim120->aero.drag.Points().size(), 11u);
    EXPECT_DOUBLE_EQ(aim120->aero.drag.Lookup(1.0), 0.60);
    ASSERT_NE(aim120->aero.normalForce, nullptr);
    EXPECT_DOUBLE_EQ(aim120->aero.normalForce->AlphaMaxRad(), 30.0 * Units::kDegToRad);
    EXPECT_DOUBLE_EQ(aim120->aero.normalForce->ReferenceAreaFt2(), 0.267888);  // the drag's: one reference area
    EXPECT_DOUBLE_EQ(aim120->aero.structuralGLimit, 30.0);
    EXPECT_EQ(aim120->seeker.mode, SeekerMode::MIDCOURSE_ARH);
    EXPECT_DOUBLE_EQ(aim120->seeker.fovHalfAngleDeg, 45.0);
    EXPECT_DOUBLE_EQ(aim120->seeker.memoryCoastTimeS, 1.5);
    EXPECT_DOUBLE_EQ(aim120->seeker.averagePowerW, 100.0);
    EXPECT_DOUBLE_EQ(aim120->seeker.antennaGain, 148.0);
    EXPECT_DOUBLE_EQ(aim120->seeker.frequencyHz, 1.0e10);
    EXPECT_DOUBLE_EQ(aim120->seeker.noiseFigureDb, 4.0);
    EXPECT_DOUBLE_EQ(aim120->seeker.integrationTimeS, 0.01);
    EXPECT_DOUBLE_EQ(aim120->seeker.pfa, 1e-6);
    EXPECT_DOUBLE_EQ(aim120->seeker.thresholdDb, 10.0 * std::log10(std::log(1e6)));  // ln(1/pfa)
    EXPECT_DOUBLE_EQ(aim120->seeker.sweepPeriodS, 0.1);
    EXPECT_EQ(aim120->seeker.confirmHits, 3);
    EXPECT_EQ(aim120->seeker.confirmWindow, 5);
    EXPECT_DOUBLE_EQ(aim120->autopilot.responseTimeS, 0.35);
    EXPECT_DOUBLE_EQ(aim120->fuze.armingRangeFt, 50.0 * Units::kFeetPerMeter);
    EXPECT_DOUBLE_EQ(aim120->selfDestruct.minAvailableG, 2.0);
    EXPECT_DOUBLE_EQ(aim120->selfDestruct.maxTimeOfFlightS, 120.0);
    EXPECT_EQ(aim120->directory, std::filesystem::path("data/missiles") / "aim120");
}

// The seeker's system loss is its one fitted term (tools/fit/radar_budget.py):
// the reference target must be acquired at exactly the fitted range.
TEST(MissileCatalog, SeekerAcquiresTheReferenceTargetAtTheFittedRange) {
    std::shared_ptr<const MissileType> aim120 = MissileCatalog::Default().Get("aim120");
    std::ifstream fitFile(aim120->directory / "fit" / "radar_fit.json");
    ASSERT_TRUE(fitFile.is_open());
    nlohmann::json fit = nlohmann::json::parse(fitFile);

    GuidanceComponent guidance = MissileEntity::MakeGuidance(*aim120);
    double rangeNmi = RadarEquation::ReferenceRangeM(
                          SeekerReceiverOf(guidance), fit.at("rcs_m2").get<double>(),
                          RadarEquation::SwerlingOneRequiredSnrDb(guidance.seekerThresholdDb, 0.5)) /
                      1852.0;
    EXPECT_NEAR(rangeNmi, fit.at("range_nmi").get<double>(), 0.001);
    EXPECT_NEAR(aim120->seeker.thresholdDb, 10.0 * std::log10(std::log(1.0 / aim120->seeker.pfa)), 1e-12);

    // The reference-range knob: the type's range is the baseline, and a new
    // one moves the coin flip there.
    EXPECT_DOUBLE_EQ(aim120->seeker.referenceRangeNmi, fit.at("range_nmi").get<double>());
    guidance.seekerReferenceRangeNmi = 15.0;
    double movedNmi = RadarEquation::ReferenceRangeM(
                          SeekerReceiverOf(guidance), fit.at("rcs_m2").get<double>(),
                          RadarEquation::SwerlingOneRequiredSnrDb(guidance.seekerThresholdDb, 0.5)) /
                      1852.0;
    EXPECT_NEAR(movedNmi, 15.0, 0.001);
}

TEST(MissileCatalog, SharesOneInstancePerType) {
    EXPECT_EQ(MissileCatalog::Default().Get("aim120").get(), MissileCatalog::Default().Get("aim120").get());
}

TEST(MissileCatalog, UnknownTypeNamesTheExpectedFile) {
    try {
        MissileCatalog::Default().Get("no_such_missile");
        FAIL() << "expected a throw";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("no_such_missile"), std::string::npos);
    }
}

TEST(MissileCatalog, MalformedTypeFileFailsAtLoad) {
    // The scratch file itself is valid, so the failures below are each
    // the one thing changed.
    {
        ScratchCatalog scratch(TypeJson());
        EXPECT_NO_THROW(MissileCatalog(scratch.Root()).Get("test_type"));
    }
    {
        ScratchCatalog scratch("{ not json");
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    {
        ScratchCatalog scratch(R"({"name": "x"})");
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    {
        ScratchCatalog scratch(TypeJson("0"));  // a seeker that cannot see anything
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    {
        // CN_max below the slope's straight line between its own rows (M1,
        // a breakpoint of the slope table only: 0.3 x 30 = 9 > 8.83): the
        // curve would bend down, so it is refused.
        ScratchCatalog scratch(TypeJson("45", "", NormalForceJson("[[0.5, 10], [2, 6.5]]")));
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    {
        ScratchCatalog scratch(TypeJson("45", "", NormalForceJson("[[2, 20], [0.5, 10]]")));  // Mach not increasing
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    {
        ScratchCatalog scratch(TypeJson("45", "", NormalForceJson("[]")));
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    {
        ScratchCatalog scratch(TypeJson("45", "", NormalForceJson("[[0.5, 10], [2, 20]]", "0")));
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    // Self-destruct: a floor of 0 or at the structural cap (30 G here),
    // and a time limit inside the 8 s burn, are load errors.
    for (const char* badG : {"0", "30"}) {
        ScratchCatalog scratch(TypeJson("45", "", NormalForceJson(), "",badG));
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error) << "min_available_g " << badG;
    }
    {
        ScratchCatalog scratch(TypeJson("45", "", NormalForceJson(), "","2", "8"));
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    // Seeker M-of-N (window 5 here): more hits than the window, none, or a
    // fraction of a sweep are load errors.
    for (const char* badHits : {"6", "0", "2.5"}) {
        ScratchCatalog scratch(
            TypeJson("45", "", NormalForceJson(), "","2", "120", badHits));
        EXPECT_THROW(MissileCatalog(scratch.Root()).Get("test_type"), std::runtime_error) << "confirm_hits " << badHits;
    }
}

// One Cd table for the whole flight, burning or coasting.
TEST(MissileCatalog, ZeroLiftCdIsTheTable) {
    const MissileAerodynamics& aero = MissileCatalog::Default().Get("aim120")->aero;
    EXPECT_DOUBLE_EQ(aero.ZeroLiftCd(2.0), 0.35);
    EXPECT_DOUBLE_EQ(aero.ZeroLiftCd(0.5), 0.30);
}

// Each check names the file's offending key, so a typo'd type file says what
// to change.
TEST(MissileCatalog, EachRejectedValueSaysWhy) {
    using nlohmann::json;
    ExpectLoadError([](json& t) { t["seeker"]["mode"] = "IR"; }, "\"mode\" must be");
    ExpectLoadError([](json& t) { t["aerodynamics"]["cd_vs_mach"] = json::array(); },
                    "missing or empty \"cd_vs_mach\"");
    ExpectLoadError([](json& t) { t["aerodynamics"]["cd_vs_mach"] = {{0.5, 0.3, 1.0}}; },
                    "every \"cd_vs_mach\" row must be [mach, cd]");
    ExpectLoadError([](json& t) { t["aerodynamics"]["cd_vs_mach"] = {{1.0, 0.5}, {0.5, 0.4}}; },
                    "strictly increasing mach");
    ExpectLoadError([](json& t) { t["aerodynamics"]["normal_force"]["cn_max_vs_mach"] = {{0.5}}; },
                    "every \"cn_max_vs_mach\" row must be [key, value]");
    ExpectLoadError([](json& t) { t["seeker"]["active_seeker"]["antenna_diameter_m"] = 0.0; },
                    "\"antenna_diameter_m\" and \"beamwidth_factor\" must be positive");
    ExpectLoadError([](json& t) { t["autopilot"]["launch_turn_exit_deg"] = 180.0; }, "launch_turn_exit_deg");
    ExpectLoadError([](json& t) { t["loft"]["full_range_nmi"] = t["loft"]["start_range_nmi"]; },
                    "\"start_range_nmi\" < \"full_range_nmi\"");
}

// A key the loader does not read is refused in every block it reads, so a
// misspelled or leftover key cannot leave a missile that differs from its
// file. Top-level "notes" is the one free-text exception.
TEST(MissileCatalog, AnUnknownKeyIsRefusedInEveryBlockTheLoaderReads) {
    using nlohmann::json;
    struct Block {
        const char* pointer;
        const char* expected;
    };
    // The leading space keeps " seeker:" from matching " active_seeker:".
    const Block blocks[] = {
        {"", "missile.json: unknown key"},
        {"/propulsion", " propulsion: unknown key"},
        {"/mass", " mass: unknown key"},
        {"/aerodynamics", " aerodynamics: unknown key"},
        {"/aerodynamics/normal_force", " normal_force: unknown key"},
        {"/seeker", " seeker: unknown key"},
        {"/seeker/active_seeker", " active_seeker: unknown key"},
        {"/autopilot", " autopilot: unknown key"},
        {"/loft", " loft: unknown key"},
        {"/fuze", " fuze: unknown key"},
        {"/self_destruct", " self_destruct: unknown key"},
    };
    for (const Block& block : blocks) {
        ExpectLoadError([&](json& t) { t[json::json_pointer(block.pointer)]["bogus"] = 1; },
                        std::string(block.expected) + " \"bogus\"");
    }
    {
        json t = RealAim120Type();
        t["notes"]["anything"] = "free text";
        ScratchCatalog scratch(t.dump());
        EXPECT_NO_THROW(MissileCatalog(scratch.Root()).Get("test_type"));
    }
}

// The launch-zone table is generated from missile.json; an edit to the type
// file without regenerating it marks the table stale (and warns).
TEST(MissileCatalog, EditedTypeFileMarksTheLaunchZoneStale) {
    namespace fs = std::filesystem;
    const fs::path real = "data/missiles/aim120";
    const fs::path root = fs::temp_directory_path() / "bvr_sim_missile_stale_test";
    auto load = [&](bool edit) {
        fs::remove_all(root);
        fs::create_directories(root / "test_type" / "fit");
        fs::copy_file(real / "missile.json", root / "test_type" / "missile.json");
        fs::copy_file(real / "fit" / "launch_zone.json", root / "test_type" / "fit" / "launch_zone.json");
        if (edit) std::ofstream(root / "test_type" / "missile.json", std::ios::app) << " ";
        return MissileCatalog(root).Get("test_type");
    };
    EXPECT_FALSE(load(false)->launchZoneStale);
    testing::internal::CaptureStderr();
    EXPECT_TRUE(load(true)->launchZoneStale);
    EXPECT_NE(testing::internal::GetCapturedStderr().find("rerun tools/drivers/launch_zone.cpp"), std::string::npos);
    fs::remove_all(root);
}
