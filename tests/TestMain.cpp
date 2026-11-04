// main for every GoogleTest binary: moves to the repo root first, so
// scenarios/ and data/ resolve wherever the binary is started. Done here
// rather than through ctest's WORKING_DIRECTORY, which would also run test
// discovery from the root and leave CMake's discovery JSON in
// <root>/CMakeFiles/. Trap: a test file must not read data at namespace
// scope -- static initialization runs before this.
#include <filesystem>

#include <gtest/gtest.h>

int main(int argc, char** argv) {
    std::filesystem::current_path(INTERCEPT_SOURCE_DIR);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
