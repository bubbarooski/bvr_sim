// LaunchZone: the launch geometry's conventions (aspect from the target's
// tail, off-boresight folded to one side), the five-axis multilinear lookup
// and its clamping, load errors, the line-ending-blind file hash, and the
// AIM-120's generated table loading through the catalog.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "missile/LaunchZone.h"
#include "missile/MissileCatalog.h"
#include "sim/Atmosphere.h"

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kFeetPerNmi = 1852.0 * 3.280839895;

// Unit horizontal vector at a heading, degrees from +Z toward +X.
Vec3 Heading(double deg, double length) {
    return Vec3{length * std::sin(deg * kDegToRad), 0.0, length * std::cos(deg * kDegToRad)};
}

// A table over two values per axis whose Rmax is a known linear function
// of the axis values, so multilinear interpolation must reproduce it
// exactly anywhere inside the grid.
double Linear(double alt, double ms, double mt, double aspect, double off) {
    return 10.0 + alt / 1000.0 + 5.0 * ms - 3.0 * mt + aspect / 30.0 + off / 10.0;
}

LaunchZone LinearZone() {
    std::array<std::vector<double>, LaunchZone::kAxes> axes = {std::vector<double>{10000.0, 30000.0},
                                                               {0.6, 1.2},
                                                               {0.6, 1.2},
                                                               {0.0, 180.0},
                                                               {0.0, 60.0}};
    std::vector<double> rmax;
    for (double a : axes[0])
        for (double b : axes[1])
            for (double c : axes[2])
                for (double d : axes[3])
                    for (double e : axes[4]) rmax.push_back(Linear(a, b, c, d, e));
    return LaunchZone(axes, rmax, "0");
}

LaunchGeometry Point(double alt, double ms, double mt, double aspect, double off) {
    LaunchGeometry g;
    g.shooterAltitudeFt = alt;
    g.shooterMach = ms;
    g.targetMach = mt;
    g.aspectDeg = aspect;
    g.offBoresightDeg = off;
    return g;
}

class ScratchFile {
public:
    ScratchFile(const std::string& name, const std::string& text)
        : path_(std::filesystem::temp_directory_path() / name) {
        std::ofstream(path_, std::ios::binary) << text;
    }
    ~ScratchFile() { std::filesystem::remove(path_); }
    const std::filesystem::path& Path() const { return path_; }

private:
    std::filesystem::path path_;
};

std::string ZoneJson(const std::string& aspect = "[0, 180]", const std::string& rmax = "") {
    std::string values = rmax;
    if (values.empty()) {
        for (int i = 0; i < 32; ++i) values += (i ? ", " : "") + std::to_string(i);
    }
    return R"({"missile_file_hash": "0", "axes": {"shooter_altitude_ft": [10000, 30000], "shooter_mach": [0.6, 1.2],
               "target_mach": [0.6, 1.2], "aspect_deg": )" +
           aspect + R"(, "off_boresight_deg": [0, 60]}, "rmax_nmi": [)" + values + "]}";
}

}  // namespace

TEST(LaunchZone, GeometryHeadOnTailAndBeam) {
    const Vec3 shooterPos{0.0, 25000.0, 0.0};
    const double a = Atmosphere::Lookup(25000.0).speedOfSoundFtPerSec;
    const Vec3 shooterVel = Heading(0.0, 0.8 * a);
    const Vec3 ahead{0.0, 25000.0, 30.0 * kFeetPerNmi};

    LaunchGeometry headOn = LaunchZone::Geometry(shooterPos, shooterVel, ahead, Heading(180.0, 0.9 * a), 0.0);
    EXPECT_NEAR(std::abs(headOn.aspectDeg), 180.0, 1e-9);
    EXPECT_NEAR(headOn.offBoresightDeg, 0.0, 1e-9);
    EXPECT_NEAR(headOn.shooterMach, 0.8, 1e-12);
    EXPECT_NEAR(headOn.targetMach, 0.9, 1e-12);
    EXPECT_NEAR(headOn.rangeFt, 30.0 * kFeetPerNmi, 1e-6);
    EXPECT_DOUBLE_EQ(headOn.shooterAltitudeFt, 25000.0);

    LaunchGeometry tail = LaunchZone::Geometry(shooterPos, shooterVel, ahead, Heading(0.0, 0.9 * a), 0.0);
    EXPECT_NEAR(tail.aspectDeg, 0.0, 1e-9);

    // Target dead ahead crossing: the shooter is off its beam, 90 deg from
    // its tail.
    LaunchGeometry beam = LaunchZone::Geometry(shooterPos, shooterVel, ahead, Heading(90.0, 0.9 * a), 0.0);
    EXPECT_NEAR(std::abs(beam.aspectDeg), 90.0, 1e-9);
}

TEST(LaunchZone, GeometryFoldsTheOtherSideOntoTheMirrorImage) {
    const Vec3 shooterPos{0.0, 25000.0, 0.0};
    const Vec3 shooterVel = Heading(0.0, 800.0);
    const double r = 20.0 * kFeetPerNmi;
    // Target 30 deg right, heading 200; its mirror is 30 deg left, heading 160.
    LaunchGeometry right = LaunchZone::Geometry(shooterPos, shooterVel, Vec3{r * std::sin(30.0 * kDegToRad), 25000.0,
                                                                             r * std::cos(30.0 * kDegToRad)},
                                                Heading(200.0, 900.0), 0.0);
    LaunchGeometry left = LaunchZone::Geometry(shooterPos, shooterVel, Vec3{-r * std::sin(30.0 * kDegToRad), 25000.0,
                                                                            r * std::cos(30.0 * kDegToRad)},
                                               Heading(160.0, 900.0), 0.0);
    EXPECT_NEAR(right.offBoresightDeg, 30.0, 1e-9);
    EXPECT_NEAR(left.offBoresightDeg, 30.0, 1e-9);
    EXPECT_NEAR(left.aspectDeg, right.aspectDeg, 1e-9);
    // bearing 30 - heading 200 = -170.
    EXPECT_NEAR(right.aspectDeg, -170.0, 1e-9);
}

// The tool places the target from a grid cell; the runtime must read the
// same cell back from the positions and velocities.
TEST(LaunchZone, GeometryInvertsTheToolsPlacement) {
    const double a = Atmosphere::Lookup(15000.0).speedOfSoundFtPerSec;
    for (double aspect : {-150.0, -90.0, 0.0, 60.0, 120.0, 180.0}) {
        for (double off : {0.0, 15.0, 44.0}) {
            const double r = 25.0 * kFeetPerNmi;
            Vec3 targetPos{r * std::sin(off * kDegToRad), 15000.0, r * std::cos(off * kDegToRad)};
            LaunchGeometry g = LaunchZone::Geometry(Vec3{0.0, 15000.0, 0.0}, Heading(0.0, 1.0 * a), targetPos,
                                                    Heading(off - aspect, 1.2 * a), 0.0);
            EXPECT_NEAR(g.offBoresightDeg, off, 1e-9);
            if (std::abs(aspect) == 180.0) {
                EXPECT_NEAR(std::abs(g.aspectDeg), 180.0, 1e-9);
            } else {
                EXPECT_NEAR(g.aspectDeg, aspect, 1e-9) << "aspect " << aspect << " off " << off;
            }
            EXPECT_NEAR(g.shooterMach, 1.0, 1e-12);
            EXPECT_NEAR(g.targetMach, 1.2, 1e-12);
        }
    }
}

TEST(LaunchZone, MultilinearInsideAndClampedOutside) {
    LaunchZone zone = LinearZone();
    // On a node and between nodes alike.
    EXPECT_NEAR(zone.RmaxFt(Point(10000.0, 0.6, 1.2, 180.0, 0.0)), Linear(10000.0, 0.6, 1.2, 180.0, 0.0) * kFeetPerNmi,
                1e-6);
    EXPECT_NEAR(zone.RmaxFt(Point(17000.0, 0.95, 0.7, 75.0, 22.0)), Linear(17000.0, 0.95, 0.7, 75.0, 22.0) * kFeetPerNmi,
                1e-6);
    // Off the grid: held at the edge.
    EXPECT_NEAR(zone.RmaxFt(Point(50000.0, 2.0, 0.3, 75.0, 22.0)), Linear(30000.0, 1.2, 0.6, 75.0, 22.0) * kFeetPerNmi,
                1e-6);
    // A negative aspect off the axis clamps to its lowest value.
    EXPECT_NEAR(zone.RmaxFt(Point(17000.0, 0.95, 0.7, -40.0, 22.0)), Linear(17000.0, 0.95, 0.7, 0.0, 22.0) * kFeetPerNmi,
                1e-6);
}

TEST(LaunchZone, LoadsAndRejectsMalformedTables) {
    {
        ScratchFile file("bvr_sim_lz_ok.json", ZoneJson());
        LaunchZone zone = LaunchZone::Load(file.Path());
        EXPECT_EQ(zone.RmaxNmi().size(), 32u);
        // Last axis fastest: index 1 is off-boresight 60 at every other axis's low end.
        EXPECT_NEAR(zone.RmaxFt(Point(10000.0, 0.6, 0.6, 0.0, 60.0)), 1.0 * kFeetPerNmi, 1e-9);
        EXPECT_NEAR(zone.RmaxFt(Point(30000.0, 1.2, 1.2, 180.0, 60.0)), 31.0 * kFeetPerNmi, 1e-9);
    }
    for (const std::string& bad : {ZoneJson("[180, 0]"),                  // not increasing
                                   ZoneJson("[0, 270]"),                  // aspect beyond 180
                                   ZoneJson("[0, 180]", "1, 2, 3"),       // too few values
                                   ZoneJson("[0]"),                       // an axis of one value
                                   std::string("{ not json")}) {
        ScratchFile file("bvr_sim_lz_bad.json", bad);
        EXPECT_THROW(LaunchZone::Load(file.Path()), std::runtime_error) << bad;
    }
    {
        std::string values = "-1";
        for (int i = 1; i < 32; ++i) values += ", 1";
        ScratchFile file("bvr_sim_lz_neg.json", ZoneJson("[0, 180]", values));
        EXPECT_THROW(LaunchZone::Load(file.Path()), std::runtime_error);
    }
}

TEST(LaunchZone, FileHashIgnoresLineEndingsOnly) {
    ScratchFile lf("bvr_sim_hash_lf.json", "{\n  \"a\": 1\n}\n");
    ScratchFile crlf("bvr_sim_hash_crlf.json", "{\r\n  \"a\": 1\r\n}\r\n");
    ScratchFile other("bvr_sim_hash_other.json", "{\n  \"a\": 2\n}\n");
    EXPECT_EQ(LaunchZone::FileHash(lf.Path()), LaunchZone::FileHash(crlf.Path()));
    EXPECT_NE(LaunchZone::FileHash(lf.Path()), LaunchZone::FileHash(other.Path()));
    EXPECT_EQ(LaunchZone::FileHash(lf.Path()).size(), 16u);
}

// The generated AIM-120 table loads with its type, was generated from the
// missile.json now in the repo, and the catalog's copy reads back the file.
TEST(LaunchZone, Aim120TableLoadsThroughTheCatalog) {
    std::shared_ptr<const MissileType> aim120 = MissileCatalog::Default().Get("aim120");
    ASSERT_NE(aim120->launchZone, nullptr);
    EXPECT_FALSE(aim120->launchZoneStale) << "rerun tools/drivers/launch_zone.cpp";

    std::ifstream in(aim120->directory / "fit" / "launch_zone.json");
    nlohmann::json j = nlohmann::json::parse(in);
    const LaunchZone& zone = *aim120->launchZone;
    for (int a = 0; a < LaunchZone::kAxes; ++a) {
        EXPECT_EQ(zone.Axes()[a], j.at("axes").at(LaunchZone::kAxisNames[a]).get<std::vector<double>>());
    }
    // A node reads back its own value.
    const auto& axes = zone.Axes();
    size_t index = 0;
    std::array<size_t, LaunchZone::kAxes> node = {2, 1, 1, axes[3].size() - 1, 0};
    for (int a = 0; a < LaunchZone::kAxes; ++a) index = index * axes[a].size() + node[a];
    EXPECT_NEAR(zone.RmaxFt(Point(axes[0][2], axes[1][1], axes[2][1], axes[3].back(), axes[4][0])),
                j.at("rmax_nmi").at(index).get<double>() * kFeetPerNmi, 1e-6);
}
