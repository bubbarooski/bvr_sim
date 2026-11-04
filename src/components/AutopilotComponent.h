#pragma once

// Plain data -- the boundary an AI-trained controller could sit behind
// later. The three target_* fields are the whole command interface;
// turnDirection below is controller-owned state, not something an
// external controller sets.
struct AutopilotComponent {
    double targetHeadingDeg = 0.0;
    double targetAltitudeFt = 0.0;
    double targetSpeedKts = 0.0;

    // Which way we committed to turn through a near-180-degree reversal:
    // 0 = not latched, +1 = right, -1 = left. Owned and cleared entirely
    // by ControlSystem.
    //
    // Heading error is ambiguous at exactly 180 degrees and numerically
    // unstable near it: a hundredth of a degree of noise flips the wrapped
    // error between -179.99 and +179.99, swinging the commanded bank by
    // 360 degrees. Without the latch the aircraft dithers instead of
    // turning.
    int turnDirection = 0;
    // The heading command the latch last saw: a jump from it is a new
    // command, which breaks the latch (its commitment was to the old one).
    // Owned by ControlSystem.
    double lastTargetHeadingDeg = 0.0;

    // Integral of bank-angle error near the target bank, deg*s. Also owned
    // by ControlSystem. A proportional roll loop settles a few degrees
    // short, which near the ~84 deg bank of a 9 G level turn is the
    // difference between 9 G and 6.
    double bankErrorIntegralDegSec = 0.0;
};
