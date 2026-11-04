"""Fit an aircraft type's drag at high lift to a flight manual's sustained turns.

A sustained turn is where full-power thrust just equals drag at that load
factor. With thrust fitted at 1 g (tools/fit/thrust_fit.py), the manual's
sustained-turn (Ps = 0) line therefore says what the drag coefficient is at
each lift coefficient it reaches:

    CD = T cos(alpha) / (q S)        at CL = (n W - T sin(alpha)) / (q S)

where the thrust's share of the force normal to the flight path comes off the
lift (alpha follows from CL, so CL is a fixed point). Inputs, under data/aircraft/<id>/:
  reference/turn_ps0.csv   the manual's Ps = 0 line: altitude, Mach, gross
                           weight, sustained turn rate. For f16c_block50,
                           T.O. GR1F-16CJ-1-1 Turn Performance charts (MAX
                           AB, drag index 0, 22 000 lb), digitised from the
                           PDF's vector paths.
  fit/turn_drag.csv        tools/drivers/turn_sample: the model's full-power thrust
                           and its drag / lift coefficients and alpha at
                           0.85, 1.0 and 1.15 x each point's load factor.

The model's drag at the manual's CL is interpolated (quadratic in CL across
the three pulls); the shortfall is the manual's CD less it. It is fitted as

    dCD = c(M) * max(0, |CL| - CL0)

with c(M) a table at Mach 0.5, 0.6, 0.7, 0.8 and 0.9 (free in sign: the
model is too high at some Machs and too low at others) falling to 0 at
Mach 1.05, where the manual shows no shortfall (the model is ~0.006 high
at CL 0.5, left alone -- that drag due to lift was fitted to NASA flight and
tunnel data). CL0 is not fitted: the turns reach CL 0.75-1.2 only, so they
cannot say where the shortfall starts, and it is held at CL0_FIXED, the top
of the flight-measured drag due to lift (NASA TP-3414, to CL ~0.65), below
which the term must not touch 1 g drag. A turn is where thrust equals drag, so the
shortfall does not depend on any error in the 1 g drag the thrust was fitted
with (thrust would carry it too): it measures only the drag that grows with
lift. The table is fitted to the current term plus the shortfall, so
re-running after --write converges rather than stacking.

Run from the repo root:
    build/Release/tools/turn_sample.exe --aircraft f16c_block50
    python tools/fit/lift_drag_fit.py --aircraft f16c_block50            # report
    python tools/fit/lift_drag_fit.py --aircraft f16c_block50 --write    # update the model
"""

import argparse
import csv
import math
import re
import sys
from collections import defaultdict

import numpy as np
from scipy.optimize import least_squares

from aircraft_data import AircraftData, add_aircraft_argument

G = 32.174
KNOTS = [0.5, 0.6, 0.7, 0.8, 0.9]
ZERO_AT_MACH = 1.05
CL0_FIXED = 0.65
BEGIN = "<!-- BEGIN lift_drag_fit.py: generated, do not edit by hand -->"
END = "<!-- END lift_drag_fit.py -->"


def isa(altitude_ft):
    """(density slug/ft3, speed of sound ft/s), US Standard Atmosphere 1976 troposphere / lower stratosphere."""
    if altitude_ft <= 36089.0:
        theta = 1.0 - 6.87559e-6 * altitude_ft
        return 0.0023769 * theta ** 4.25588, 1116.45 * math.sqrt(theta)
    return 0.00070612 * math.exp(-(altitude_ft - 36089.0) / 20805.8), 968.08


def amplitude(c, mach):
    """c(M): the table's knots, then zero at ZERO_AT_MACH (clamped below the first knot)."""
    return float(np.interp(mach, KNOTS + [ZERO_AT_MACH], list(c) + [0.0]))


def current_term(xml):
    """(c at KNOTS, CL0) of the term already in the model, or (zeros, 0.4) if there is none."""
    if BEGIN not in xml:
        return [0.0] * len(KNOTS), 0.4
    block = xml[xml.index(BEGIN):xml.index(END)]
    cl0 = float(re.search(r"CL0 = ([\d.eE+-]+) -->", block).group(1))
    rows = re.findall(r"^\s+([\d.]+)\s+([\d.eE+-]+)\s*$", block[block.index("<tableData>"):block.index("</tableData>")], re.M)
    table = {round(float(m), 4): float(v) for m, v in rows}
    return [table[round(k, 4)] for k in KNOTS], cl0


def term_xml(c, cl0):
    rows = "\n".join(f"       {k:.4f}  {v:.5f}" for k, v in zip(KNOTS, c))
    return f"""{BEGIN}
   <!-- CL0 = {cl0:.4f} -->
   <function name="aero/coefficient/CDturn">
    <description>Drag_above_the_polar_break: T.O. GR1F-16CJ-1-1 sustained turns, tools/fit/lift_drag_fit.py</description>
    <product>
     <property>aero/qbar-psf</property>
     <property>metrics/Sw-sqft</property>
     <table>
      <independentVar>velocities/mach</independentVar>
      <tableData>
{rows}
       {ZERO_AT_MACH:.4f}  0.00000
      </tableData>
     </table>
     <max>
      <value>0.0</value>
      <difference>
       <sqrt><property>aero/cl-squared</property></sqrt>
       <value>{cl0:.4f}</value>
      </difference>
     </max>
    </product>
   </function>
   {END}"""


def shortfalls(aircraft):
    wing_area = aircraft.type["geometry"]["wing_area_ft2"]
    manual = {}
    for r in csv.DictReader(open(aircraft.reference("turn_ps0.csv"))):
        manual[(float(r["altitude_ft"]), float(r["mach"]))] = (float(r["gross_weight_lb"]), float(r["rate_deg_s"]))
    samples = defaultdict(list)
    for r in csv.DictReader(open(aircraft.fit("turn_drag.csv"))):
        samples[(float(r["altitude_ft"]), float(r["point_mach"]))].append(r)
    out = []
    for key, rows in sorted(samples.items()):
        altitude, mach = key
        weight, rate = manual[key]
        rho, sound = isa(altitude)
        speed = mach * sound
        qs = 0.5 * rho * speed * speed * wing_area
        omega_v_over_g = math.radians(rate) * speed / G
        n = math.sqrt(omega_v_over_g ** 2 + 1.0)
        cl = np.array([float(r["lift_coefficient"]) for r in rows])
        cd = np.array([float(r["drag_coefficient"]) for r in rows])
        alpha = np.array([float(r["alpha_deg"]) for r in rows])
        thrust = float(rows[0]["thrust_lbf"])
        alpha_of_cl = np.polyfit(cl, alpha, 1)
        cl_manual = n * weight / qs
        for _ in range(100):
            alpha_manual = float(np.polyval(alpha_of_cl, cl_manual))
            cl_next = (n * weight - thrust * math.sin(math.radians(alpha_manual))) / qs
            if abs(cl_next - cl_manual) < 1e-9:
                break
            cl_manual = cl_next
        cd_model = float(np.polyval(np.polyfit(cl, cd, 2), cl_manual))
        alpha_manual = float(np.polyval(alpha_of_cl, cl_manual))
        cd_manual = thrust * math.cos(math.radians(alpha_manual)) / qs
        out.append(dict(altitude=altitude, mach=mach, n=n, cl=cl_manual, cd_model=cd_model, cd_manual=cd_manual,
                        shortfall=cd_manual - cd_model, alpha=alpha_manual, in_range=cl.min() <= cl_manual <= cl.max()))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_aircraft_argument(parser)
    parser.add_argument("--write", action="store_true", help="write the drag term into the type's model")
    args = parser.parse_args()

    aircraft = AircraftData(args.aircraft)
    xml = open(aircraft.model_xml, encoding="utf-8").read()
    c_now, cl0_now = current_term(xml)

    def term(c, cl0, cl, mach):
        return amplitude(c, mach) * max(0.0, cl - cl0)

    points = shortfalls(aircraft)
    print(" alt ft  Mach    n    CL    CD model  CD manual  shortfall  alpha")
    for p in points:
        print("%6.0f  %4.2f  %4.2f  %5.3f  %7.4f  %7.4f   %+7.4f   %4.1f%s" % (
            p["altitude"], p["mach"], p["n"], p["cl"], p["cd_model"], p["cd_manual"], p["shortfall"], p["alpha"],
            "" if p["in_range"] else "  (extrapolated)"))

    fit_points = [p for p in points if p["mach"] <= KNOTS[-1] + 1e-9]
    targets = np.array([term(c_now, cl0_now, p["cl"], p["mach"]) + p["shortfall"] for p in fit_points])
    k = len(KNOTS)

    def residuals(x):
        return np.array([term(x, CL0_FIXED, p["cl"], p["mach"]) for p in fit_points]) - targets

    start = list(c_now)
    fit = least_squares(residuals, start, bounds=([-0.3] * k, [0.6] * k))
    c, cl0 = list(fit.x), CL0_FIXED
    rms = math.sqrt(np.mean(residuals(fit.x) ** 2))
    print("\nfitted to %d subsonic points: c(M) = %s at Mach %s, CL0 = %.3f (fixed; model had %.3f); rms residual %.4f in CD" % (
        len(fit_points), " ".join("%+.4f" % v for v in c), " ".join("%.1f" % m for m in KNOTS), cl0, cl0_now, rms))

    if args.write:
        block = term_xml(c, cl0)
        if BEGIN in xml:
            xml = xml[:xml.index(BEGIN)] + block + xml[xml.index(END) + len(END):]
        else:
            # After the compressibility drag-due-to-lift term, its neighbour.
            anchor = xml.index('<function name="aero/coefficient/CDliftMach">')
            end = xml.index("</function>", anchor) + len("</function>")
            xml = xml[:end] + "\n   " + block + xml[end:]
        open(aircraft.model_xml, "w", encoding="utf-8", newline="").write(xml)
        print("wrote %s" % aircraft.model_xml)


if __name__ == "__main__":
    sys.exit(main())
