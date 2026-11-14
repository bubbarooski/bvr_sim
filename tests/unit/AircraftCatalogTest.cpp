// Aircraft types load from data/aircraft/<id>/aircraft.json: every field
// reaches the AircraftType, a type is parsed once and shared, and a missing
// or malformed type file fails at load naming the file rather than flying
// an aircraft with zeroed limits.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "aircraft/AircraftCatalog.h"
#include "aircraft/AircraftEntity.h"
#include "sim/RadarEquation.h"

namespace {

// A scratch catalog root holding one type directory with the given file
// contents, removed when the test ends. Named per test: ctest -j runs each
// TEST() as its own process, so a shared root would race.
class ScratchCatalog {
public:
    explicit ScratchCatalog(const std::string& typeJson)
        : root_(std::filesystem::temp_directory_path() /
                ("bvr_sim_catalog_test_" + std::string(testing::UnitTest::GetInstance()->current_test_info()->name()))) {
        std::filesystem::remove_all(root_);
        std::filesystem::create_directories(root_ / "test_type");
        std::ofstream(root_ / "test_type" / "aircraft.json") << typeJson;
    }
    ~ScratchCatalog() { std::filesystem::remove_all(root_); }
    const std::filesystem::path& Root() const { return root_; }

private:
    std::filesystem::path root_;
};

nlohmann::json RealF16Type() {
    std::ifstream real("data/aircraft/f16c_block50/aircraft.json");
    return nlohmann::json::parse(real);
}

// Loads the real F-16 type with one edit and expects a load error whose
// message contains `expected`.
void ExpectLoadError(const std::function<void(nlohmann::json&)>& edit, const std::string& expected) {
    nlohmann::json type = RealF16Type();
    edit(type);
    ScratchCatalog scratch(type.dump());
    try {
        AircraftCatalog(scratch.Root()).Get("test_type");
        ADD_FAILURE() << "expected a load error containing: " << expected;
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(expected), std::string::npos) << e.what();
    }
}

}  // namespace

// Physically impossible values in a type file fail at load naming the key,
// rather than flying an aircraft that silently counts drop tanks as internal
// fuel or accumulates negative damage.
TEST(AircraftCatalog, ImpossibleTypeValuesAreRefusedNamingTheKey) {
    ExpectLoadError([](nlohmann::json& t) { t["fuel"]["internal_tank_prefix"] = ""; }, "internal_tank_prefix");
    ExpectLoadError([](nlohmann::json& t) { t["envelope"]["damage_accumulation_rate"] = -0.1; },
                    "damage_accumulation_rate");
    ExpectLoadError([](nlohmann::json& t) { t["geometry"]["hit_box_ft"]["span"] = 0.0; }, "hit_box_ft");
    ExpectLoadError([](nlohmann::json& t) { t["geometry"]["hit_box_ft"]["length"] = -49.3; }, "hit_box_ft");
    // 1e30 is not representable as the whole number of frames it must be.
    ExpectLoadError([](nlohmann::json& t) { t["radar"]["confirm_frames"] = 1e30; }, "confirm_frames");
    ExpectLoadError([](nlohmann::json& t) { t["rwr"]["sweep_period_s"] = 0.0; }, "sweep_period_s");
    ExpectLoadError([](nlohmann::json& t) { t["rwr"]["signal_jitter_db"] = -1.0; }, "signal_jitter_db");
}

// A key the loader does not read is refused in every block it reads, so a
// misspelled or leftover key cannot leave an aircraft that differs from its
// file. Top-level "notes" is the one free-text exception.
TEST(AircraftCatalog, AnUnknownKeyIsRefusedInEveryBlockTheLoaderReads) {
    using nlohmann::json;
    struct Block {
        const char* pointer;
        const char* expected;
    };
    const Block blocks[] = {
        {"", "aircraft.json: unknown key"},
        {"/geometry", " geometry: unknown key"},
        {"/geometry/hit_box_ft", " hit_box_ft: unknown key"},
        {"/envelope", " envelope: unknown key"},
        {"/flight_controls", " flight_controls: unknown key"},
        {"/fuel", " fuel: unknown key"},
        {"/radar", " radar: unknown key"},
        {"/rwr", " rwr: unknown key"},
        {"/signature", " signature: unknown key"},
        {"/signature/rcs_shape/0", " rcs_shape: unknown key"},
        {"/signature/rcs_shape/0/semi_axes_ft", " semi_axes_ft: unknown key"},
    };
    for (const Block& block : blocks) {
        ExpectLoadError([&](json& t) { t[json::json_pointer(block.pointer)]["bogus"] = 1; },
                        std::string(block.expected) + " \"bogus\"");
    }
    // The case that matters in practice: a misspelled optional block would
    // otherwise load as a type with no radar warning receiver.
    ExpectLoadError(
        [](json& t) {
            t["rwrr"] = t["rwr"];
            t.erase("rwr");
        },
        "unknown key \"rwrr\"");
    {
        json t = RealF16Type();
        t["notes"]["anything"] = "free text";
        ScratchCatalog scratch(t.dump());
        EXPECT_NO_THROW(AircraftCatalog(scratch.Root()).Get("test_type"));
    }
}

// A type id names one directory under the catalog root. A path that climbs
// out of it, or replaces it, would load an unrelated file as a type.
TEST(AircraftCatalog, AnIdThatIsNotOneDirectoryNameIsRefused) {
    AircraftCatalog& catalog = AircraftCatalog::Default();
    std::vector<const char*> ids = {"", ".", "..", "../aircraft/f16c_block50", "f16c_block50/..", "/f16c_block50",
                                    "sub/f16c_block50"};
#ifdef _WIN32
    // A drive prefix is a path component only on Windows; elsewhere "C:x" is
    // an ordinary directory name.
    ids.push_back("C:f16c_block50");
    ids.push_back("C:\\f16c_block50");
#endif
    for (const char* id : ids) {
        try {
            catalog.Get(id);
            ADD_FAILURE() << "'" << id << "' should be refused";
        } catch (const std::runtime_error& e) {
            EXPECT_NE(std::string(e.what()).find("invalid"), std::string::npos) << id << ": " << e.what();
        }
    }
}

TEST(AircraftCatalog, LoadsTheF16Type) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    EXPECT_EQ(f16->id, "f16c_block50");
    EXPECT_EQ(f16->jsbsimModel, "f16");
    EXPECT_DOUBLE_EQ(f16->wingAreaFt2, 300.0);
    EXPECT_DOUBLE_EQ(f16->hitBox.halfSpanFt, 15.0);
    EXPECT_DOUBLE_EQ(f16->hitBox.halfHeightFt, 8.35);
    EXPECT_DOUBLE_EQ(f16->hitBox.halfLengthFt, 24.65);
    EXPECT_DOUBLE_EQ(f16->envelope.clMax, 1.65);
    EXPECT_DOUBLE_EQ(f16->envelope.clMaxLossPerG, 0.049);
    EXPECT_DOUBLE_EQ(f16->envelope.structuralGLimitPos, 9.0);
    EXPECT_DOUBLE_EQ(f16->envelope.structuralGLimitNeg, -3.0);
    EXPECT_DOUBLE_EQ(f16->controls.gPerUnitStick, 8.0);
    EXPECT_DOUBLE_EQ(f16->controls.militaryThrottleCmd, 0.5);
    EXPECT_DOUBLE_EQ(f16->controls.maxThrottleCmd, 1.0);
    EXPECT_EQ(f16->internalTankPrefix, "internal");
    EXPECT_EQ(f16->directory, std::filesystem::path("data/aircraft") / "f16c_block50");
}

// The radar's system loss is the one fitted term (tools/fit/radar_budget.py).
// Its reference target must be detected at exactly the fitted range, so an
// edit to any other budget term that is not followed by a re-fit fails
// here rather than silently moving every detection range.
TEST(AircraftCatalog, RadarDetectsTheReferenceTargetAtTheFittedRange) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    ASSERT_TRUE(f16->radar.present);
    std::ifstream fitFile(f16->directory / "fit" / "radar_fit.json");
    ASSERT_TRUE(fitFile.is_open());
    nlohmann::json fit = nlohmann::json::parse(fitFile);

    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*f16);
    // Where one look at the Swerling 1 reference is a coin flip.
    auto coinFlipNmi = [&](double thresholdDb) {
        return RadarEquation::ReferenceRangeM(ReceiverOf(radar), fit.at("rcs_m2").get<double>(),
                                              RadarEquation::SwerlingOneRequiredSnrDb(thresholdDb, 0.5)) /
               1852.0;
    };
    EXPECT_NEAR(coinFlipNmi(radar.searchThresholdDb), fit.at("range_nmi").get<double>(), 0.001);

    // The reference-range knob: the type's range is the baseline, and a new
    // one moves the coin flip there, receive side only (the transmitted peak
    // power an RWR hears is untouched).
    EXPECT_DOUBLE_EQ(f16->radar.referenceRangeNmi, fit.at("range_nmi").get<double>());
    double peakPowerW = radar.peakPowerW;
    radar.referenceRangeNmi = 25.0;
    EXPECT_NEAR(coinFlipNmi(radar.searchThresholdDb), 25.0, 0.001);
    EXPECT_EQ(radar.peakPowerW, peakPowerW);

    // The search threshold is the square-law detector's for the type's
    // false-alarm probability: ln(1/pfa) over the noise power.
    EXPECT_NEAR(f16->radar.searchThresholdDb, 10.0 * std::log10(std::log(1.0 / f16->radar.pfa)), 1e-12);
}

// The signature's shape is tools/fit/rcs_shape.py's ESTIMATE from the type's
// geometry. Its nose-on projected area is the type's static RCS, pinned to
// the tool's own value so the C++ and Python projected areas agree.
TEST(AircraftCatalog, SignatureShapeGivesTheNoseOnRcs) {
    const AircraftSignatureType& signature = AircraftCatalog::Default().Get("f16c_block50")->signature;
    ASSERT_TRUE(signature.present);
    EXPECT_EQ(signature.shapeCount, 4);  // fuselage, wing, horizontal tail, fin
    EXPECT_NEAR(signature.staticRcsM2, 3.7752430577642215, 1e-9);
    const RcsEllipsoid* parts = signature.shape.data();
    EXPECT_NEAR(RadarEquation::ShapeProjectedAreaM2(parts, 4, Vec3{1.0, 0.0, 0.0}), 25.400990353958186, 1e-9);
    EXPECT_NEAR(RadarEquation::ShapeProjectedAreaM2(parts, 4, Vec3{0.0, 1.0, 0.0}), 55.79430009550157, 1e-9);
    EXPECT_NEAR(RadarEquation::ShapeProjectedAreaM2(parts, 4, Vec3{0.5, 0.0, std::sqrt(3.0) / 2.0}),
                13.726604149973099, 1e-9);
}

// The coin-flip ranges REFERENCE 9.8 quotes for the shaped F-16 (nose-on and
// beam-on at 25 000 ft in free space), computed through the same code path
// the radar flies, so the document and the sim cannot drift apart.
TEST(AircraftCatalog, RadarCoinFlipRangesOfTheShapedF16) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*f16);
    RadarCrossSection rcs = AircraftEntity::MakeRadarCrossSection(*f16);
    double required = RadarEquation::SwerlingOneRequiredSnrDb(radar.searchThresholdDb, 0.5);
    auto coinFlipNmi = [&](const Vec3& towardSensor) {
        return RadarEquation::ReferenceRangeM(ReceiverOf(radar), RadarEquation::MeanRcsM2(rcs, Vec3{}, towardSensor),
                                              required) /
               1852.0;
    };
    // The target flies north (+Z): the sensor is ahead of its nose for the
    // nose-on look, abeam (+X) for the beam-on one.
    EXPECT_NEAR(coinFlipNmi(Vec3{0.0, 0.0, 1.0}), 46.61, 0.02);  // REFERENCE: 46.6
    EXPECT_NEAR(coinFlipNmi(Vec3{1.0, 0.0, 0.0}), 75.06, 0.02);  // REFERENCE: 75.1
}

// The inverse of the Swerling 1 detection probability exists only for
// exp(-threshold) < pd < 1; outside it the formula returns NaN or infinity,
// which would pass silently into a range.
TEST(RadarEquation, SwerlingRequiredSnrRoundTripsAndRefusesAnImpossibleProbability) {
    const double thresholdDb = RadarEquation::DetectionThresholdDb(1e-6);
    for (double pd : {0.1, 0.5, 0.9, 0.99}) {
        double snrDb = RadarEquation::SwerlingOneRequiredSnrDb(thresholdDb, pd);
        EXPECT_NEAR(RadarEquation::SwerlingOneDetectionProbability(snrDb, thresholdDb), pd, 1e-9) << pd;
    }
    EXPECT_THROW(RadarEquation::SwerlingOneRequiredSnrDb(thresholdDb, 1.0), std::domain_error);
    EXPECT_THROW(RadarEquation::SwerlingOneRequiredSnrDb(thresholdDb, 0.0), std::domain_error);
    EXPECT_THROW(RadarEquation::SwerlingOneRequiredSnrDb(thresholdDb, 1e-7), std::domain_error);  // below pfa
    EXPECT_THROW(RadarEquation::SwerlingOneRequiredSnrDb(thresholdDb, std::nan("")), std::domain_error);
}

// The RCS-scale knob multiplies the RCS a sensor sees, after aspect, so a
// target scaled x16 is seen twice as far (range ~ RCS^(1/4)).
TEST(AircraftCatalog, RcsScaleMultipliesTheSeenRcs) {
    std::shared_ptr<const AircraftType> f16 = AircraftCatalog::Default().Get("f16c_block50");
    RadarCrossSection rcs = AircraftEntity::MakeRadarCrossSection(*f16);
    EXPECT_EQ(rcs.rcsScale, 1.0);
    Vec3 attitude{10.0, 5.0, 30.0}, look{0.6, 0.0, 0.8};
    double baseM2 = RadarEquation::MeanRcsM2(rcs, attitude, look);
    rcs.rcsScale = 16.0;
    EXPECT_NEAR(RadarEquation::MeanRcsM2(rcs, attitude, look), 16.0 * baseM2, 1e-9 * baseM2);

    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*f16);
    double required = RadarEquation::SwerlingOneRequiredSnrDb(radar.searchThresholdDb, 0.5);
    EXPECT_NEAR(RadarEquation::ReferenceRangeM(ReceiverOf(radar), 16.0 * baseM2, required) /
                    RadarEquation::ReferenceRangeM(ReceiverOf(radar), baseM2, required),
                2.0, 1e-9);
}

TEST(AircraftCatalog, SignatureNeedsAShapeNotAStaticRcs) {
    auto typeWith = [](const std::string& signature) {
        return R"({
            "name": "x", "jsbsim_model": "x",
            "geometry": {"wing_area_ft2": 300, "hit_box_ft": {"span": 30, "height": 16, "length": 49}},
            "envelope": {"cl_max": 1.6, "cl_max_loss_per_g": 0.0, "structural_g_limit_pos": 9, "structural_g_limit_neg": -3,
                         "damage_accumulation_rate": 0.01},
            "flight_controls": {"g_per_unit_stick": 8, "military_throttle_cmd": 0.5, "max_throttle_cmd": 1},
            "fuel": {"internal_tank_prefix": "internal"},
            "signature": )" + signature + "}";
    };
    const std::string part = R"({"part": "body", "semi_axes_ft": {"span": 2, "height": 3, "length": 20}})";
    {
        ScratchCatalog scratch(typeWith(R"({"fluctuation_model": "swerling_1", "rcs_shape": [)" + part + "]}"));
        // Held, not a temporary: the reference below points into it.
        std::shared_ptr<const AircraftType> type = AircraftCatalog(scratch.Root()).Get("test_type");
        const AircraftSignatureType& signature = type->signature;
        EXPECT_EQ(signature.shapeCount, 1);
        EXPECT_NEAR(signature.staticRcsM2, 3.14159265358979 * 2.0 * 3.0 * 0.3048 * 0.3048, 1e-9);
    }
    {
        // The RCS comes from the shape; a stated value would be a second, conflicting source.
        ScratchCatalog scratch(typeWith(R"({"static_rcs_m2": 5, "fluctuation_model": "swerling_1", "rcs_shape": [)" +
                                        part + "]}"));
        try {
            AircraftCatalog(scratch.Root()).Get("test_type");
            ADD_FAILURE() << "expected static_rcs_m2 to be refused";
        } catch (const std::runtime_error& e) {
            EXPECT_NE(std::string(e.what()).find("unknown key \"static_rcs_m2\""), std::string::npos) << e.what();
        }
    }
    {
        ScratchCatalog scratch(typeWith(R"({"fluctuation_model": "swerling_1"})"));
        EXPECT_THROW(AircraftCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    {
        ScratchCatalog scratch(typeWith(
            R"({"fluctuation_model": "swerling_1", "rcs_shape": [{"part": "flat", "semi_axes_ft": {"span": 2, "height": 0, "length": 20}}]})"));
        EXPECT_THROW(AircraftCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
}

TEST(AircraftCatalog, SharesOneInstancePerType) {
    EXPECT_EQ(AircraftCatalog::Default().Get("f16c_block50").get(),
              AircraftCatalog::Default().Get("f16c_block50").get());
}

TEST(AircraftCatalog, ListsAvailableTypes) {
    std::vector<std::string> ids = AircraftCatalog::Default().Available();
    EXPECT_NE(std::find(ids.begin(), ids.end(), "f16c_block50"), ids.end());
}

TEST(AircraftCatalog, UnknownTypeNamesTheExpectedFile) {
    try {
        AircraftCatalog::Default().Get("no_such_aircraft");
        FAIL() << "expected a throw";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("no_such_aircraft"), std::string::npos);
    }
}

TEST(AircraftCatalog, MalformedTypeFileFailsAtLoad) {
    // Not JSON at all.
    {
        ScratchCatalog scratch("{ not json");
        EXPECT_THROW(AircraftCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    // Valid JSON missing a required block.
    {
        ScratchCatalog scratch(R"({"name": "x", "jsbsim_model": "x"})");
        EXPECT_THROW(AircraftCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    // Present but physically impossible: a positive negative-G limit.
    {
        ScratchCatalog scratch(R"({
            "name": "x", "jsbsim_model": "x",
            "geometry": {"wing_area_ft2": 300, "hit_box_ft": {"span": 30, "height": 16, "length": 49}},
            "envelope": {"cl_max": 1.6, "cl_max_loss_per_g": 0.0, "structural_g_limit_pos": 9, "structural_g_limit_neg": 3,
                         "damage_accumulation_rate": 0.01},
            "flight_controls": {"g_per_unit_stick": 8, "military_throttle_cmd": 0.5, "max_throttle_cmd": 1},
            "fuel": {"internal_tank_prefix": "internal"}})");
        EXPECT_THROW(AircraftCatalog(scratch.Root()).Get("test_type"), std::runtime_error);
    }
    // The STT frame: required, positive, and no longer than a scan frame.
    for (double sttFrameTimeS : {-1.0, 0.0, 5.0}) {
        std::ifstream real("data/aircraft/f16c_block50/aircraft.json");
        nlohmann::json type = nlohmann::json::parse(real);
        if (sttFrameTimeS < 0.0) {
            type["radar"].erase("stt_frame_time_s");
        } else {
            type["radar"]["stt_frame_time_s"] = sttFrameTimeS;  // 5 s > the 4.4 s frame
        }
        ScratchCatalog scratch(type.dump());
        EXPECT_THROW(AircraftCatalog(scratch.Root()).Get("test_type"), std::runtime_error) << sttFrameTimeS;
    }
    // The STT drop: required, a whole number of frames, at least one.
    for (double sttDropFrames : {-1.0, 0.0, 2.5}) {
        std::ifstream real("data/aircraft/f16c_block50/aircraft.json");
        nlohmann::json type = nlohmann::json::parse(real);
        if (sttDropFrames < 0.0) {
            type["radar"].erase("stt_drop_frames");
        } else {
            type["radar"]["stt_drop_frames"] = sttDropFrames;
        }
        ScratchCatalog scratch(type.dump());
        EXPECT_THROW(AircraftCatalog(scratch.Root()).Get("test_type"), std::runtime_error) << sttDropFrames;
    }
}

// Each check names the file's offending key, so a typo'd type file says what
// to change.
TEST(AircraftCatalog, EachRejectedValueSaysWhy) {
    ExpectLoadError([](nlohmann::json& t) { t.erase("jsbsim_model"); }, "missing or non-string \"jsbsim_model\"");
    ExpectLoadError([](nlohmann::json& t) { t["signature"]["fluctuation_model"] = "chi_squared"; },
                    "fluctuation_model must be");
    ExpectLoadError([](nlohmann::json& t) { t["radar"]["pfa"] = 1.5; }, "out of its physical range");
    ExpectLoadError([](nlohmann::json& t) { t["rwr"]["antenna_gain"] = 0.0; }, "antenna_gain must be positive");
}

TEST(AircraftCatalog, SteadySignatureLoads) {
    nlohmann::json type = RealF16Type();
    type["signature"]["fluctuation_model"] = "steady";
    ScratchCatalog scratch(type.dump());
    EXPECT_EQ(AircraftCatalog(scratch.Root()).Get("test_type")->signature.fluctuation, FluctuationModel::STEADY);
}
