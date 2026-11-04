#pragma once

#include <cstdint>

class World;
struct SimContext;
struct ScenarioData;

// The one definition of the per-tick system call order. Every driver of
// the sim -- the GUI, the headless validation tests, the batch runner --
// goes through here, so the order cannot drift between them.
namespace SimLoop {

// Starts a run: seeds the one RNG stream and hands the systems the
// scenario's air. The same seed recreates the same run, and every driver
// gets it by seeding through here rather than seeding the RNG directly.
void BeginRun(ScenarioData& scenario, SimContext& simContext, uint64_t seed);

// Advances every system by exactly one tick of `dt` seconds.
//
// Ordering follows the data dependencies, and three of them are
// load-bearing:
//  - Guidance before Integration, so the integration step uses this
//    tick's freshly computed acceleration rather than a stale one.
//  - Launch after the radar systems, so its firing-solution gate reads
//    this tick's track tier. It runs after Integration so a missile
//    launched this tick is not also force-integrated the same tick.
//  - Datalink after the radar systems, so an update carries this tick's
//    track rather than the previous one's. Guidance reads the result next
//    tick, which is a real datalink latency.
// Intercept runs on the range that resulted from all of the above; only
// self-destruct runs after it.
//
// simTime advances mid-step, so the systems that timestamp events see an
// accurate "now".
void Step(World& world, SimContext& simContext, const ScenarioData& scenario, double dt, double& simTime);

}  // namespace SimLoop
