#pragma once

#include <cmath>

#include "util/Units.h"

// Everything in a radar's signal-to-noise budget except the target and the
// range: the terms of the SNR form of the radar equation,
//
//     SNR = Pavg G^2 lam^2 sigma T / ((4 pi)^3 R^4 k T0 F L)
//
// (average power and coherent time on target, rather than peak power and
// bandwidth, because a pulse-Doppler radar integrates its pulses). Kept
// apart from RadarEquation.h so the components that carry one stay plain
// data. See REFERENCE.md - Radar.
struct ReceiverBudget {
    double averagePowerW = 0.0;
    double antennaGain = 0.0;  // linear; the same antenna transmits and receives
    double wavelengthM = 0.0;
    double noiseFigureDb = 0.0;
    double integrationTimeS = 0.0;  // coherent time on target per look
    double systemLossDb = 0.0;      // everything else, fitted to the cited range
};

inline double WavelengthM(double frequencyHz) { return Units::kSpeedOfLightMPerS / frequencyHz; }

// The reference-range teaching knob, on the receive side: a sensor whose
// type detects its reference target at typeRangeNmi, asked to detect it at
// rangeNmi instead, gains 40 log10(rangeNmi / typeRangeNmi) dB (SNR ~
// 1/R^4), so every detection range scales by the same ratio while what it
// transmits -- what a radar warning receiver hears -- is unchanged. Zero
// for a type with no reference range (no fit file), and exactly zero at
// the type's own range.
inline double ReferenceRangeGainDb(double rangeNmi, double typeRangeNmi) {
    if (typeRangeNmi <= 0.0 || rangeNmi <= 0.0) return 0.0;
    return 40.0 * std::log10(rangeNmi / typeRangeNmi);
}
