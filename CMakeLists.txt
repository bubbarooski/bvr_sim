cmake_minimum_required(VERSION 3.25)
# C is enabled for deps/gl3w/src/gl3w.c: without it that lone C source compiles
# with cl.exe's default static CRT (/MT) instead of the /MD everything else
# uses, and the mismatch warns (LNK4098) when it reaches bvr_sim.
project(bvr_sim VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Link-time optimization (MSVC /GL + /LTCG) for Release only, so cross-module
# inlining reaches JSBSim's hot stepping path. Set before add_subdirectory so
# the JSBSim targets inherit it; /fp:precise is preserved, so Release stays
# bit-identical. Checked for CXX only -- the C deps (gl3w/glfw) never get
# /GL, so C IPO support is irrelevant and must not gate the C++ LTCG.
include(CheckIPOSupported)
check_ipo_supported(RESULT INTERCEPT_IPO_OK OUTPUT INTERCEPT_IPO_MSG LANGUAGES CXX)
if(INTERCEPT_IPO_OK)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    message(STATUS "IPO/LTCG enabled for Release builds")
else()
    message(STATUS "IPO/LTCG not available: ${INTERCEPT_IPO_MSG}")
endif()

# Cross-platform reproducibility: a seed must mean the same run on every
# OS. GCC and Clang may fuse a*b+c into one FMA instruction where the
# hardware has it, which rounds once instead of twice and changes results
# between machines; turn that off for everything, dependencies included.
# MSVC does not contract under its default /fp:precise. See sim/Rng.h.
if(NOT MSVC)
    add_compile_options(-ffp-contract=off)
endif()

# AddressSanitizer build for memory-error sweeps (MSVC). Applied to every
# target, dependencies included: MSVC's annotated std containers refuse to
# link instrumented and plain objects together. Use RelWithDebInfo -- ASan
# is incompatible with Debug's /RTC1 run-time checks.
option(INTERCEPT_ASAN "Build with AddressSanitizer (MSVC)" OFF)
if(INTERCEPT_ASAN AND MSVC)
    add_compile_options(/fsanitize=address /Zi)
    add_link_options(/INCREMENTAL:NO /DEBUG)
    string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REPLACE "/RTC1" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
endif()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
# SYSTEM on every third-party dependency: their headers are then included
# as system headers, so the warning level below applies to this project's
# own code and not to theirs.
add_subdirectory(deps/jsbsim SYSTEM)

# gl3w/GLFW are C (no MSVC IPO for C) and imgui gains nothing here -- turn IPO
# off for them, then back on for the C++ targets below.
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE OFF)

find_package(OpenGL REQUIRED)

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(deps/glfw SYSTEM)

add_library(gl3w STATIC deps/gl3w/src/gl3w.c)
target_include_directories(gl3w PUBLIC deps/gl3w/include)

set(IMGUI_DIR deps/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GL3W)
target_link_libraries(imgui PUBLIC glfw gl3w OpenGL::GL)
set_target_properties(gl3w imgui PROPERTIES SYSTEM TRUE)

# Warnings for this project's own targets only.
function(bvr_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra)
    endif()
endfunction()

# The output folder holds only the app (bvr_sim, bvr_batch and
# the data they load); tests and measurement tools go in tests/ and tools/
# below it. Multi-config generators (Visual Studio) add the config folder.
function(bvr_output_subdir target subdir)
    if(CMAKE_CONFIGURATION_TYPES)
        set(dir "${CMAKE_BINARY_DIR}/$<CONFIG>/${subdir}")
    else()
        set(dir "${CMAKE_BINARY_DIR}/${subdir}")
    endif()
    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${dir}" ARCHIVE_OUTPUT_DIRECTORY "${dir}")
endfunction()
bvr_output_subdir(gl3w lib)
bvr_output_subdir(imgui lib)

# Back on for the C++ targets, so their objects get /GL and the final link
# runs /LTCG against JSBSim.
if(INTERCEPT_IPO_OK)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
endif()

# Eigen, header-only. Zero-initialised by default: sim code treats a
# default-constructed Vec3 as the origin (util/Vec3.h).
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE deps/eigen)
target_compile_definitions(eigen INTERFACE EIGEN_INITIALIZE_MATRICES_BY_ZERO)

# Shared simulation core. Every driver of the sim -- the GUI, the headless
# validations, the batch runner -- links this instead of re-listing the system
# set. A static lib pulls only the objects a target actually references, so the
# full-pipeline targets get everything and the leaner ones stay lean without a
# separate source list. libJSBSim and the include dirs are PUBLIC so consumers
# inherit them.
add_library(bvr_core STATIC
    src/ecs/World.cpp
    src/scenario/ParamRegistry.cpp
    src/scenario/ScenarioLoader.cpp
    src/scenario/ScenarioLoaderJson.cpp
    src/scenario/ScenarioSaver.cpp
    src/aircraft/AircraftCatalog.cpp
    src/missile/LaunchZone.cpp
    src/missile/MissileCatalog.cpp
    src/missile/MissileNormalForce.cpp
    src/sim/AircraftBounds.cpp
    src/sim/Atmosphere.cpp
    src/sim/DragTable.cpp
    src/sim/Environment.cpp
    src/sim/EventLog.cpp
    src/sim/RadarEquation.cpp
    src/sim/RadarTrackFilter.cpp
    src/sim/SimLoop.cpp
    src/sim/TickRecorder.cpp
    src/systems/AirborneRadarSystem.cpp
    src/systems/ControlSystem.cpp
    src/systems/DatalinkSystem.cpp
    src/systems/EnvironmentSystem.cpp
    src/systems/FlightDynamicsSystem.cpp
    src/systems/GroundImpactSystem.cpp
    src/systems/GuidanceSystem.cpp
    src/systems/IntegrationSystem.cpp
    src/systems/InterceptSystem.cpp
    src/systems/LaunchSystem.cpp
    src/systems/MissileSelfDestructSystem.cpp
    src/systems/RwrSystem.cpp
    src/systems/WaypointSystem.cpp
)
target_include_directories(bvr_core PUBLIC src)
target_include_directories(bvr_core SYSTEM PUBLIC deps/json)
target_link_libraries(bvr_core PUBLIC libJSBSim eigen)
bvr_warnings(bvr_core)
bvr_output_subdir(bvr_core lib)

# GUI application: sim core + renderer + the scenario editor/saver (GUI-only,
# so not in the shared core). imgui pulls in glfw/gl3w/OpenGL transitively.
add_executable(bvr_sim
    src/app/main.cpp
    src/render/Camera.cpp
    src/render/EntityInspector.cpp
    src/render/EventMarks.cpp
    src/render/RadarConeTier.cpp
    src/render/ParameterPanel.cpp
    src/render/RenderSystem.cpp
    src/render/Sidebar.cpp
    src/render/ViewSettings.cpp
    src/render/style/LatticeStyle.cpp
)
target_link_libraries(bvr_sim PRIVATE bvr_core imgui)
bvr_warnings(bvr_sim)

# Link (don't copy) scenarios/ into the output dir so hand/GUI edits take effect
# with no rebuild. remove_directory first -- mklink/create_symlink fail if the
# link path already exists. The link is a separate -P script because mklink
# needs a native backslash path, which only exists once the generator
# expression has resolved.
add_custom_command(TARGET bvr_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "$<TARGET_FILE_DIR:bvr_sim>/scenarios"
    COMMAND ${CMAKE_COMMAND}
            "-DLINK_PATH=$<TARGET_FILE_DIR:bvr_sim>/scenarios"
            "-DTARGET_PATH=${CMAKE_SOURCE_DIR}/scenarios"
            -P "${CMAKE_SOURCE_DIR}/cmake/CreateScenarioJunction.cmake"
    COMMENT "Linking scenarios/ into output directory")

# data/ and fonts/ load via cwd-relative paths too, but aren't hand-edited live
# -- copied, not linked.
add_custom_command(TARGET bvr_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/data" "$<TARGET_FILE_DIR:bvr_sim>/data"
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:bvr_sim>/fonts"
    COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_SOURCE_DIR}/deps/imgui/misc/fonts/Roboto-Medium.ttf"
            "${CMAKE_SOURCE_DIR}/deps/imgui/misc/fonts/Cousine-Regular.ttf"
            "$<TARGET_FILE_DIR:bvr_sim>/fonts"
    COMMENT "Copying data/ and fonts/ to output directory")

# --- Headless drivers and measurement tools: each links the whole sim core ---

# Parallel Monte Carlo driver: runs the sim pipeline over a caller-supplied run
# list. Threads::Threads for the std::thread worker pool. Run from repo root.
add_executable(bvr_batch src/batch/BatchMain.cpp)
find_package(Threads REQUIRED)
target_link_libraries(bvr_batch PRIVATE bvr_core Threads::Threads)
bvr_warnings(bvr_batch)

# Sim-side performance benchmark (SimLoop::Step over N aircraft). Prints timing
# + a behavior checksum; not a pass/fail gate. See tests/perf_benchmark.cpp.
add_executable(perf_benchmark tests/perf_benchmark.cpp)
target_link_libraries(perf_benchmark PRIVATE bvr_core)
bvr_warnings(perf_benchmark)
bvr_output_subdir(perf_benchmark tools)

# Sim vs PUBLISHED real-world F-16/AIM-120 figures. A measurement instrument,
# not a pass/fail gate. Run from repo root so JSBSim resolves aircraft data.
# See docs/VALIDATION.md.
add_executable(real_world_validation tests/real_world_validation.cpp)
target_link_libraries(real_world_validation PRIVATE bvr_core)
bvr_warnings(real_world_validation)
bvr_output_subdir(real_world_validation tools)

# F-16 pitch FCS design, step 1: identifies the bare airframe's short-period
# model over an altitude x Mach grid (CSV to stdout). tools/fit/fcs_design.py
# turns it into the gain tables in f16.xml. Run from repo root.
add_executable(fcs_identify tools/drivers/fcs_identify.cpp)
target_link_libraries(fcs_identify PRIVATE bvr_core)
bvr_warnings(fcs_identify)
bvr_output_subdir(fcs_identify tools)

# Engine thrust fit, step 1: samples the F-16's 1 g drag and alpha over
# Mach in level accelerations (CSV). tools/fit/thrust_fit.py fits the thrust
# tables to the flight manual's acceleration times with it. Flies the
# validation rig's aircraft, hence tests/ on the include path.
add_executable(drag_sample tools/drivers/drag_sample.cpp)
target_link_libraries(drag_sample PRIVATE bvr_core)
target_include_directories(drag_sample PRIVATE tests)
bvr_warnings(drag_sample)
bvr_output_subdir(drag_sample tools)

# Part-throttle fuel flow fit, step 1: the thrust and engine speed the
# model needs at each of a flight manual's steady cruise points (CSV).
# tools/fit/fuel_fit.py fits the part-throttle factor with it. Flies the
# validation rig's aircraft.
add_executable(cruise_sample tools/drivers/cruise_sample.cpp)
target_link_libraries(cruise_sample PRIVATE bvr_core)
target_include_directories(cruise_sample PRIVATE tests)
bvr_warnings(cruise_sample)
bvr_output_subdir(cruise_sample tools)

# Drag-due-to-lift check, step 1: Ps in hard level turns at a flight
# manual's sustained-turn load factors (CSV). tools/fit/lift_drag_fit.py fits
# the drag correction from it. Flies the validation rig's aircraft.
add_executable(turn_sample tools/drivers/turn_sample.cpp)
target_link_libraries(turn_sample PRIVATE bvr_core)
target_include_directories(turn_sample PRIVATE tests)
bvr_warnings(turn_sample)
bvr_output_subdir(turn_sample tools)

# Launch zone: flies a missile type over a grid of launch conditions for its
# maximum launch range against a non-maneuvering target, into
# data/missiles/<id>/fit/launch_zone.json. Threads::Threads for the worker
# pool. Run from repo root.
add_executable(launch_zone tools/drivers/launch_zone.cpp)
target_link_libraries(launch_zone PRIVATE bvr_core Threads::Threads)
bvr_warnings(launch_zone)
bvr_output_subdir(launch_zone tools)

# --- Tests (GoogleTest) ---
# gtest_force_shared_crt: GoogleTest defaults to MSVC's static CRT (/MT);
# everything else here uses the dynamic CRT (/MD), and the mismatch is the
# same class of LNK4098 warning gl3w.c's C/C++ split above avoids. IPO off
# -- LTCG buys nothing for a test binary and nothing links against
# googletest afterward, so it's never turned back on.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE OFF)
add_subdirectory(deps/googletest SYSTEM)

# gtest_discover_tests registers each individual TEST() as its own ctest
# entry, so one failing check reads as one failing ctest name. Discovery
# runs at ctest time, serially, not as a POST_BUILD step: CMake 4.4's
# post-build discovery names every target's temporary test-list JSON the
# same, so a parallel build races on it and fails with a JSON syntax error.
include(GoogleTest)
set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)

# Every test runs from the repo root so scenarios/ and data/jsbsim_data/
# resolve: tests/TestMain.cpp changes to it before running them. Not
# gtest_discover_tests' WORKING_DIRECTORY -- discovery would then run from
# the root too, and CMake writes its test-list JSON into <cwd>/CMakeFiles/.
function(bvr_add_test target)
    target_sources(${target} PRIVATE tests/TestMain.cpp)
    target_compile_definitions(${target} PRIVATE INTERCEPT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_link_libraries(${target} PRIVATE gtest)
    bvr_warnings(${target})
    bvr_output_subdir(${target} tests)
    gtest_discover_tests(${target})
endfunction()

# A deliberately minimal target, compiling only the sources it tests:
# unit_tests is pure logic with no JSBSim.
add_executable(unit_tests
    tests/unit/AircraftCatalogTest.cpp
    tests/unit/AtmosphereTest.cpp
    tests/unit/CameraFollowTest.cpp
    tests/unit/CameraTest.cpp
    tests/unit/ComponentStoreTest.cpp
    tests/unit/DragTableTest.cpp
    tests/unit/EventLogTest.cpp
    tests/unit/EventMarksTest.cpp
    tests/unit/FlightEnvelopeTest.cpp
    tests/unit/InterceptSystemTest.cpp
    tests/unit/LaunchZoneTest.cpp
    tests/unit/Mat4Test.cpp
    tests/unit/MissileCatalogTest.cpp
    tests/unit/MissileNormalForceTest.cpp
    tests/unit/NumericTest.cpp
    tests/unit/ParamRegistryTest.cpp
    tests/unit/RadarConeTierTest.cpp
    tests/unit/RadarTrackFilterTest.cpp
    tests/unit/RngTest.cpp
    tests/unit/SimClockTest.cpp
    tests/unit/TrackTableTest.cpp
    tests/unit/Vec3Test.cpp
    tests/unit/WindTest.cpp
    tests/unit/WorldTest.cpp
    src/aircraft/AircraftCatalog.cpp
    src/missile/LaunchZone.cpp
    src/missile/MissileCatalog.cpp
    src/missile/MissileNormalForce.cpp
    src/ecs/World.cpp
    src/render/Camera.cpp
    src/render/EventMarks.cpp
    src/render/RadarConeTier.cpp
    src/scenario/ParamRegistry.cpp
    src/sim/AircraftBounds.cpp
    src/sim/Atmosphere.cpp
    src/sim/DragTable.cpp
    src/sim/EventLog.cpp
    src/sim/RadarEquation.cpp
    src/sim/RadarTrackFilter.cpp
    src/sim/Environment.cpp
    src/systems/InterceptSystem.cpp
)
target_include_directories(unit_tests PRIVATE src)
target_include_directories(unit_tests SYSTEM PRIVATE deps/json)
# imgui (and through it GLFW) only to link Camera.cpp, whose input handling
# the tests never call.
target_link_libraries(unit_tests PRIVATE eigen imgui)
bvr_add_test(unit_tests)

# Whole-pipeline validation: physics and behavior checked against closed
# forms, independent re-derivations, and known answers -- guidance and
# missile physics, aircraft flight, radar, every teaching scenario, and
# scenario save/reset.
add_executable(validation_tests
    tests/validation/AircraftManeuversValidationTest.cpp
    tests/validation/BatchCliValidationTest.cpp
    tests/validation/EngagementFirstPrinciplesValidationTest.cpp
    tests/validation/FlightStabilityValidationTest.cpp
    tests/validation/GuidanceValidationTest.cpp
    tests/validation/LaunchFirstPrinciplesValidationTest.cpp
    tests/validation/MissileFirstPrinciplesValidationTest.cpp
    tests/validation/MissileManeuversValidationTest.cpp
    tests/validation/MissileSelfDestructValidationTest.cpp
    tests/validation/PhysicsValidationTest.cpp
    tests/validation/RadarValidationTest.cpp
    tests/validation/ScenarioLoaderValidationTest.cpp
    tests/validation/ScenarioSaverValidationTest.cpp
    tests/validation/ScenarioSmokeValidationTest.cpp
)
target_link_libraries(validation_tests PRIVATE bvr_core)
# BatchCliValidationTest runs the built bvr_batch of the same config.
add_dependencies(validation_tests bvr_batch)
target_compile_definitions(validation_tests PRIVATE BVR_BATCH_EXE="$<TARGET_FILE:bvr_batch>")
bvr_add_test(validation_tests)

# The Python tooling's own tests (tests/python/, stdlib unittest): the pointer
# helper, the batch_compare regression gate and clean.py's deletion rules.
# Cases that need pandas or a built bvr_batch skip without them.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME PythonTools
             COMMAND ${Python3_EXECUTABLE} -I -m unittest discover -s ${CMAKE_SOURCE_DIR}/tests/python
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_tests_properties(PythonTools PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
endif()

# Invoke with `ctest -C Release`. perf_benchmark and real_world_validation
# are measurement instruments (always exit 0), so they build but are not
# registered here.
enable_testing()
