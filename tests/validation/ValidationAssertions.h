#pragma once

#include <cmath>

#include <gtest/gtest.h>

// Shared percent/absolute tolerance checks for the validation suite,
// returned as an AssertionResult so EXPECT_TRUE/ASSERT_TRUE print the full
// diagnostic on failure and stay silent on success.
namespace validation {

inline ::testing::AssertionResult NearPct(double measured, double expected, double tolerancePct,
                                           const char* label = "") {
    double errorPct = expected != 0.0 ? 100.0 * (measured - expected) / expected : (measured == 0.0 ? 0.0 : 1e9);
    if (std::fabs(errorPct) <= tolerancePct) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << label << ": measured=" << measured << " expected=" << expected
                                          << " error=" << errorPct << "% exceeds tolerance " << tolerancePct << "%";
}

inline ::testing::AssertionResult NearAbs(double measured, double expected, double absTolerance,
                                           const char* label = "") {
    double delta = measured - expected;
    if (std::fabs(delta) <= absTolerance) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << label << ": measured=" << measured << " expected=" << expected
                                          << " delta=" << delta << " exceeds tolerance " << absTolerance;
}

}  // namespace validation
