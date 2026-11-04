#pragma once

#include <cstdlib>

#include "FGJSBBase.h"

// Silences JSBSim's per-aircraft console banner so ctest output stays
// legible in binaries that spawn many aircraft. JSBSim reads an env var at
// init and also honors a direct debug_lvl assignment; setting both covers
// whichever path its startup code consults.
struct JsbSimDebugSilencer {
    JsbSimDebugSilencer() {
#ifdef _WIN32
        _putenv("JSBSIM_DEBUG=0");
#else
        setenv("JSBSIM_DEBUG", "0", 1);
#endif
        JSBSim::FGJSBBase::debug_lvl = 0;
    }
};
inline const JsbSimDebugSilencer kSilenceJsbSimDebug;
