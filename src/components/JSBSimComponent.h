#pragma once

#include <memory>

#include "FGFDMExec.h"
#include "util/Vec3.h"

// Property-tree nodes for every JSBSim property read or written per tick,
// resolved once at Init rather than re-parsed from their string paths on
// every access -- that path walk runs thousands of times a second at
// 120Hz. The nodes are owned by the FGFDMExec's own property tree, the
// same lifetime as `fdm` below, so these raw pointers cannot dangle.
struct JSBSimPropertyNodes {
    SGPropertyNode* latGcDeg = nullptr;      // position/lat-gc-deg
    SGPropertyNode* longGcDeg = nullptr;     // position/long-gc-deg
    SGPropertyNode* hSlFt = nullptr;         // position/h-sl-ft
    SGPropertyNode* phiDeg = nullptr;        // attitude/phi-deg
    SGPropertyNode* thetaDeg = nullptr;      // attitude/theta-deg
    SGPropertyNode* psiDeg = nullptr;        // attitude/psi-deg
    SGPropertyNode* pRadSec = nullptr;       // velocities/p-rad_sec
    SGPropertyNode* qRadSec = nullptr;       // velocities/q-rad_sec
    // TRUE airspeed, not calibrated -- the initial condition is set as
    // true and the V-n envelope derives from vt-fps, so true airspeed is
    // this sim's speed convention everywhere it matters. Holding a
    // calibrated reading against a true target compares two different
    // physical quantities.
    SGPropertyNode* vtKts = nullptr;         // velocities/vtrue-kts
    SGPropertyNode* vtFps = nullptr;         // velocities/vt-fps
    SGPropertyNode* vNorthFps = nullptr;     // velocities/v-north-fps
    SGPropertyNode* vEastFps = nullptr;      // velocities/v-east-fps
    SGPropertyNode* vDownFps = nullptr;      // velocities/v-down-fps
    SGPropertyNode* nz = nullptr;            // accelerations/Nz
    SGPropertyNode* nx = nullptr;            // accelerations/Nx (body axial load factor, thrust included)
    SGPropertyNode* ny = nullptr;            // accelerations/Ny (body side load factor)
    SGPropertyNode* weightLbs = nullptr;     // inertia/weight-lbs
    SGPropertyNode* aileronCmd = nullptr;    // fcs/aileron-cmd-norm
    SGPropertyNode* elevatorCmd = nullptr;   // fcs/elevator-cmd-norm
    SGPropertyNode* throttleCmd = nullptr;   // fcs/throttle-cmd-norm[0]
    // aero/function/kCLmax-mach: the model's maximum lift at this Mach over
    // its low-speed value. Null for a model that does not define it, which
    // reads as 1 (no Mach dependence).
    SGPropertyNode* clMaxMachFactor = nullptr;
};

// One FGFDMExec per aircraft entity. FGFDMExec.h is included in full
// (not forward-declared) because unique_ptr's destructor needs a complete
// type wherever ComponentStore<JSBSimComponent> destroys an entry.
struct JSBSimComponent {
    std::unique_ptr<JSBSim::FGFDMExec> fdm;
    JSBSimPropertyNodes props;

    // The ISA temperature offset currently applied to this aircraft's own
    // JSBSim atmosphere, so EnvironmentSystem only touches it on a change.
    double appliedTemperatureOffsetC = 0.0;
    // Likewise the wind (world frame, ft/s). The first application is the
    // spawn wind, which also moves the aircraft's ground velocity -- see
    // FlightDynamicsSystem::ApplySpawnWind. It is applied on
    // EnvironmentSystem's first tick, the same path as any later change.
    Vec3 appliedWindFps;
    bool spawnWindApplied = false;

    // The trim throttle, captured once at Init, used as the feed-forward
    // baseline for ControlSystem's throttle loop. It is pure P, so it
    // settles at whatever speed error produces the throttle the aircraft
    // actually needs -- the closer the zero-error command is to real trim,
    // the smaller that residual. (Pitch needs no such baseline: the FCS
    // holds 1 G at neutral stick on its own.)
    double trimThrottle = 0.0;
};
