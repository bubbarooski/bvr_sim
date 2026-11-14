#pragma once

#include <fstream>
#include <string>

#include "ecs/World.h"

struct SimContext;

// Writes the sim's changing state to a CSV, one sample per call: a row per
// entity that has a Transform, then a row per radar track. Long format with
// one fixed header; a cell that does not apply to a row's kind is empty.
// Static per-type data (radar power, motor impulse...) is left out -- it is
// in the type files. Columns are listed in docs/SCHEMA.md, "Tick recording".
//
// Shared by the GUI's Record button and bvr_batch --record, so the two
// files have the same columns.
class TickRecorder {
public:
    // Truncates `path` and writes the header. False, with `error` set, if the
    // file cannot be opened; the recorder then stays closed.
    bool Open(const std::string& path, std::string& error);
    void Close();
    bool IsOpen() const { return file_.is_open(); }
    const std::string& Path() const { return path_; }

    // `simTime` is the time the state refers to: call after SimLoop::Step,
    // with the simTime it advanced. Does nothing while closed.
    void Sample(World& world, const SimContext& simContext, double simTime);

private:
    std::ofstream file_;
    std::string path_;
    std::string line_;  // reused across rows so a sample allocates nothing once warm
};
