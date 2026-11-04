"""Missile normal force: compute a type's two CN tables from a component build-up.

The simulator's normal force is CN(alpha, M) = a alpha + b alpha^2 on the drag
reference area, from two tables over Mach (REFERENCE.md 6.5):

  cn_alpha_per_deg_vs_mach   the slope a
  cn_max_vs_mach             CN at alpha_max; b pins the curve to it

For the AIM-120 both are DERIVED from a component build-up of published theory
(body potential lift + viscous crossflow + fins with interference). The
build-up's inputs are data/missiles/<id>/fit/normal_force_buildup.json:

  CN_max(M)   = the build-up's CN(alpha_max, M) at 25 000 ft, standard day
  CN_alpha(M) = with CN_max pinned, the least-squares slope over alpha
                0.5-10 deg, the band guidance flies in

Both are rescaled from the build-up's pi d^2 / 4 to the type's
reference_area_ft2, so the force is unchanged. See docs/REFERENCE.md 6.5.

Usage, from the repo root:
    python tools/fit/fit_normal_force.py --missile aim120                 # report only
    python tools/fit/fit_normal_force.py --missile aim120 --write         # also update the type file
"""

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
FIT_ALTITUDE_FT = 25000.0
SLOPE_BAND_DEG = np.arange(0.5, 10.0001, 0.5)
MACH_ROWS = [0.3, 0.5, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0]


class BuildUp:
    """CN(alpha, M, Re_d): the attached-flow table plus the body's viscous crossflow."""

    def __init__(self, inputs):
        table = inputs["cn_attached"]
        self.alpha = np.array(table["alpha_deg"])
        self.mach = np.array(table["mach"])
        self.cn_rows = np.array(table["cn"])
        cf = inputs["body_crossflow"]
        self.cdc_mc = cf["cdc_vs_crossflow_mach"]
        self.cdc_re = cf["cdc_vs_log10_reynolds"]
        self.fade_lo, self.fade_hi = cf["reynolds_fade_crossflow_mach"]
        self.eta_subsonic = cf["eta_subsonic"]
        self.planform_ratio = cf["planform_area_ratio"]
        self.alpha_max_deg = inputs["alpha_max_deg"]
        self.diameter_ft = inputs["body_diameter_in"] / 12.0

    @staticmethod
    def _interp(rows, key):
        return float(np.interp(key, [r[0] for r in rows], [r[1] for r in rows]))

    def cn(self, alpha_deg, mach, re_d):
        alpha_deg = min(abs(alpha_deg), self.alpha[-1])
        attached = float(np.interp(mach, self.mach, [np.interp(alpha_deg, self.alpha, row) for row in self.cn_rows]))
        s = math.sin(math.radians(alpha_deg))
        mc = mach * s
        cdc = self._interp(self.cdc_mc, mc)
        if mc < self.fade_hi:
            dropped = min(self._interp(self.cdc_re, math.log10(max(re_d * s, 1.0e4))), cdc)
            w = min(max((mc - self.fade_lo) / (self.fade_hi - self.fade_lo), 0.0), 1.0)
            cdc = dropped + w * (cdc - dropped)
        eta = 1.0 if mach > 1.0 else self.eta_subsonic
        return attached + eta * cdc * self.planform_ratio * s * s

    def area_ft2(self):
        return 0.25 * math.pi * self.diameter_ft ** 2


def reynolds_d(mach, altitude_ft, diameter_ft):
    """Reynolds number on body diameter, ISA standard day, Sutherland viscosity (SI inside)."""
    h = altitude_ft * 0.3048
    t = 288.15 - 0.0065 * h if h < 11000 else 216.65
    p = 101325 * (t / 288.15) ** 5.25588 if h < 11000 else 22632.06 * math.exp(-9.80665 * (h - 11000) / (287.053 * t))
    rho = p / (287.053 * t)
    mu = 1.458e-6 * t ** 1.5 / (t + 110.4)
    return rho * mach * math.sqrt(1.4 * 287.053 * t) * diameter_ft * 0.3048 / mu


def fit_row(build_up, mach):
    re_d = reynolds_d(mach, FIT_ALTITUDE_FT, build_up.diameter_ft)
    amax = build_up.alpha_max_deg
    cn_max = build_up.cn(amax, mach, re_d)
    cn = np.array([build_up.cn(a, mach, re_d) for a in SLOPE_BAND_DEG])
    # CN = a alpha + (cn_max - a amax)(alpha / amax)^2, linear in a.
    g = SLOPE_BAND_DEG - amax * (SLOPE_BAND_DEG / amax) ** 2
    f = cn - cn_max * (SLOPE_BAND_DEG / amax) ** 2
    return float(g @ f / (g @ g)), cn_max


def format_json(obj):
    """Indent like the hand-written type files, but keep arrays of numbers on one line
    (the same formatter as tools/fit/rcs_shape.py)."""
    text = json.dumps(obj, indent=4, ensure_ascii=False)
    number = r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
    pattern = re.compile(r"\[\s*(%s(?:\s*,\s*%s)*)\s*\]" % (number, number))
    return pattern.sub(lambda m: "[" + ", ".join(re.split(r"\s*,\s*", m.group(1).strip())) + "]", text) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--missile", required=True, help="missile type id (data/missiles/<id>)")
    parser.add_argument("--write", action="store_true", help="store the fitted tables in the type file")
    args = parser.parse_args()

    rel = "data/missiles/%s/missile.json" % args.missile
    inputs_rel = "data/missiles/%s/fit/normal_force_buildup.json" % args.missile
    build_up = BuildUp(json.loads((REPO / inputs_rel).read_text(encoding="utf-8")))
    path = REPO / rel
    doc = json.loads(path.read_text(encoding="utf-8"))
    scale = build_up.area_ft2() / doc["aerodynamics"]["reference_area_ft2"]

    slope, cn_max = [], []
    print("Fit at %.0f ft, alpha_max %g deg, area rescale %.6f" % (FIT_ALTITUDE_FT, build_up.alpha_max_deg, scale))
    print("  Mach  CN_alpha/deg  CN_max")
    for mach in MACH_ROWS:
        a, c = fit_row(build_up, mach)
        slope.append([mach, round(a * scale, 4)])
        cn_max.append([mach, round(c * scale, 3)])
        print("  %4.2f  %10.4f  %8.3f" % (mach, slope[-1][1], cn_max[-1][1]))
        if c < a * build_up.alpha_max_deg:
            raise SystemExit("CN_max below CN_alpha * alpha_max at M%g: the curve would not rise" % mach)

    if args.write:
        normal_force = {"alpha_max_deg": build_up.alpha_max_deg, "cn_alpha_per_deg_vs_mach": slope,
                        "cn_max_vs_mach": cn_max}
        # Only the aerodynamics block's normal_force text changes: a
        # whole-file dump would respell hand-written numbers elsewhere.
        block = format_json({"normal_force": normal_force}).strip()[1:-1].strip("\n")
        block = "\n".join("    " + line for line in block.split("\n"))
        text = path.read_text(encoding="utf-8")
        text, nf_count = re.subn(r'        "normal_force": \{.*?\n        \}(?=\n    \})', lambda m: block, text, count=1,
                                 flags=re.S)
        if nf_count != 1:
            raise SystemExit("no aerodynamics.normal_force block found in %s" % rel)
        written = json.loads(text)["aerodynamics"]["normal_force"]
        assert written == normal_force, "the rewritten block does not parse back to the fit"
        path.write_text(text, encoding="utf-8", newline="\n")
        print("wrote %s" % rel)


if __name__ == "__main__":
    main()
