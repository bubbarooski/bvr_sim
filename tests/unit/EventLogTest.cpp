#include <gtest/gtest.h>

#include "sim/EventLog.h"

TEST(EventLog, PrintAllListsEveryEventInOrder) {
    EventLog log;
    log.Record({1.5, "LAUNCH", 3, "range_nmi=20.0"});
    log.Record({12.25, "INTERCEPT", 4, ""});

    testing::internal::CaptureStdout();
    log.PrintAll();
    std::string printed = testing::internal::GetCapturedStdout();

    EXPECT_EQ(printed,
              "Event log (2 events):\n"
              "  [  1.500s] LAUNCH    entity=3   range_nmi=20.0\n"
              "  [ 12.250s] INTERCEPT entity=4   \n");

    log.Clear();
    EXPECT_TRUE(log.Events().empty());
}

// A viewer that caches the log cannot tell a cleared-and-refilled log from a
// continued one by its size alone (the new run may already be as long as the
// old when the viewer next looks), so Clear() starts a new generation.
TEST(EventLog, ClearStartsANewGenerationAndRecordingDoesNot) {
    EventLog log;
    const unsigned long long first = log.Generation();
    log.Record({1.0, "LAUNCH", 3, ""});
    log.Record({2.0, "MISS", 3, ""});
    EXPECT_EQ(log.Generation(), first);
    log.Clear();
    EXPECT_NE(log.Generation(), first);
    const unsigned long long second = log.Generation();
    log.Record({1.0, "LAUNCH", 3, ""});
    EXPECT_EQ(log.Generation(), second);
    log.Clear();
    EXPECT_NE(log.Generation(), second);
}
