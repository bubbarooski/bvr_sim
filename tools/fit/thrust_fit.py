"""Fit an aircraft type's installed engine thrust tables to flight-test data.

Installed thrust at altitude is rarely published, but a flight manual's
level-acceleration tables measure thrust minus drag directly: in level
flight

    (W/g) dV/dt = T cos(alpha) - D

Inputs, under data/aircraft/<id>/:
  reference/flight_manual_accel.csv  MIL / MAX AB level accelerations: time
                                     and fuel from the first tabulated speed
                                     to each later one, per altitude and
                                     gross weight. For f16c_block50 these are
                                     T.O. GR1F-16CJ-1-1 (Hellenic Air Force
                                     F-16C/D supplemental flight manual,
                                     2003) Appendix A, F100-PW-229, drag
                                     index 0, "data basis flight test",
                                     parsed from the PDF.
  fit/drag_1g.csv                    the model's drag and alpha (drag_sample)
  fit/thrust_fit.json                weights to fit, extra Mach rows
The engine file rewritten is the one the type's JSBSim model names.

Drag and alpha are the model's, which for the F-16 rest on NASA TP-1538 /
TP-3355 / TP-3414 data. Thrust is the only
quantity fitted. For each altitude the thrust at the table's Mach rows is
chosen so that integrating the equation above, with weight falling by the
manual's own fuel burn, reproduces the manual's acceleration times (least
squares on segment times, in seconds -- the tables round to 1 s). Rows
outside the data keep the old table's shape, scaled to the nearest fitted
row. Above the data, thrust scales with pressure (corrected thrust is
constant in the isothermal stratosphere).

Several gross-weight columns are fitted together: thrust does not depend on
weight, so how well one thrust curve fits them all checks the model's drag
due to lift.

Run from the repo root:
    python tools/fit/thrust_fit.py --aircraft f16c_block50            # fit and report
    python tools/fit/thrust_fit.py --aircraft f16c_block50 --write    # also rewrite the engine tables
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
# Seconds of acceleration-time error worth a unit second difference in log
# thrust between neighbouring Mach rows (0.2 apart).
SMOOTHING = 20.0


def isa(altitude_ft):
    """US Standard Atmosphere 1976: (pressure ratio delta, speed of sound ft/s)."""
    h = altitude_ft * 0.3048
    if h <= 11000.0:
        theta = 1.0 - 2.25577e-5 * h
        delta = theta ** 5.25588
    elif h <= 20000.0:
        theta = 216.65 / 288.15
        delta = 0.223361 * math.exp(-(h - 11000.0) / 6341.62)
    else:
        theta = (216.65 + 0.001 * (h - 20000.0)) / 288.15
        delta = 0.0540330 * (1.0 + (h - 20000.0) / 216650.0) ** -34.1632
    return delta, 1116.45 * math.sqrt(theta)


# --- engine file ------------------------------------------------------------

def read_table(xml, name):
    block = re.search(r'<function name="%s">.*?<tableData>\n(.*?)\n\s*</tableData>' % name, xml, re.S).group(1)
    lines = block.splitlines()
    altitudes = [float(v) for v in lines[0].split()]
    machs, values = [], []
    for line in lines[1:]:
        v = [float(x) for x in line.split()]
        machs.append(v[0])
        values.append(v[1:])
    return np.array(machs), np.array(altitudes), np.array(values)


def format_table(machs, altitudes, values):
    out = ["    " + "".join("%9d" % a for a in altitudes)]
    for m, row in zip(machs, values):
        out.append("     %.1f " % m + "".join("%9.4f" % v for v in row))
    return "\n".join(out)


def write_table(xml, name, machs, altitudes, values):
    pattern = r'(<function name="%s">.*?<tableData>\n)(.*?)(\n\s*</tableData>)' % name
    return re.sub(pattern, lambda m: m.group(1) + format_table(machs, altitudes, values) + m.group(3), xml,
                  count=1, flags=re.S)


# --- model drag -------------------------------------------------------------

class Drag:
    """The model's 1 g drag and alpha at one altitude, as functions of Mach and weight.

    The sampler's altitude hold lets load factor wander (0.9-1.1 G), so at each
    Mach drag is regressed on lift squared across the six sampled weights,
    D = A + B L^2 (the model's own polar), and evaluated at L = W.
    """

    def __init__(self, rows):
        mach = np.array([float(r["mach"]) for r in rows])
        lift = np.array([float(r["nz"]) * float(r["weight_lb"]) for r in rows])
        drag = np.array([float(r["drag_lbf"]) for r in rows])
        alpha = np.array([float(r["alpha_deg"]) for r in rows])
        grid, a, b, a0, a1 = [], [], [], [], []
        for m in np.arange(0.25, 2.2, 0.01):
            near = np.abs(mach - m) < 0.015
            if near.sum() < 6 or np.ptp(lift[near]) < 4000.0:
                continue
            x = lift[near] ** 2
            b_, a_ = np.polyfit(x, drag[near], 1)
            s_, i_ = np.polyfit(lift[near], alpha[near], 1)
            grid.append(m)
            a.append(a_)
            b.append(b_)
            a0.append(i_)
            a1.append(s_)
        self.grid = np.array(grid)
        self.a, self.b = np.array(a), np.array(b)
        self.alpha0, self.alpha1 = np.array(a0), np.array(a1)
        self.top = self.grid.max() if len(grid) else 0.0
        self.bottom = self.grid.min() if len(grid) else 0.0

    def drag(self, mach, weight):
        return np.interp(mach, self.grid, self.a) + np.interp(mach, self.grid, self.b) * weight ** 2

    def alpha_rad(self, mach, weight):
        return math.radians(np.interp(mach, self.grid, self.alpha0) + np.interp(mach, self.grid, self.alpha1) * weight)


# --- level acceleration -----------------------------------------------------

def fly(thrust_of_mach, drag, altitude, gross, rows, dmach=0.002):
    """Integrates a level acceleration through the rows' Machs; returns the time at each."""
    _, sound = isa(altitude)
    t_rows = np.array([r["time"] for r in rows])
    f_rows = np.array([r["fuel"] for r in rows])
    times = [0.0]
    t = 0.0
    mach = rows[0]["mach"]
    for target in [r["mach"] for r in rows[1:]]:
        while mach < target - 1e-9:
            step = min(dmach, target - mach)
            m = mach + 0.5 * step
            weight = gross - np.interp(t, t_rows, f_rows)
            accel = G * (thrust_of_mach(m) * math.cos(drag.alpha_rad(m, weight)) - drag.drag(m, weight)) / weight
            if accel <= 0.05:
                return None  # cannot reach the next speed
            t += step * sound / accel
            mach += step
        times.append(t)
    return np.array(times)


class FitConfig:
    """Everything the fit reads about one aircraft type."""

    def __init__(self, aircraft):
        self.aircraft = aircraft
        self.manual_csv = aircraft.reference("flight_manual_accel.csv")
        self.drag_csv = aircraft.fit("drag_1g.csv")
        settings = aircraft.fit_settings("thrust_fit.json")
        self.weights = tuple(settings["fit_gross_weights_lb"])
        # Extra Mach rows for the afterburning table, where the data have
        # steeper features than the generic 0.2-Mach rows can follow.
        self.refine = {"AugThrust": tuple(settings.get("extra_aug_mach_rows", ()))}
        self.extra_lapse = float(settings.get("stratosphere_extra_lapse_exponent", 0.0))
        self.extra_lapse_mil = float(settings.get("stratosphere_extra_lapse_exponent_mil", 0.0))
        # Climb-envelope points (Ps at 1 g) that the 40 000 ft column is
        # fitted to, per power setting.
        self.envelope = defaultdict(list)
        if settings.get("fit_mil_to_climb_envelope", False):
            for r in csv.DictReader(open(aircraft.reference("climb_envelope.csv"))):
                if r["power"] == "MIL":
                    self.envelope["MIL"].append({"mach": float(r["mach"]), "altitude": float(r["altitude_ft"]),
                                                 "gross": float(r["gross_weight_lb"]),
                                                 "ps": float(r["climb_fpm"]) / 60.0})
        with open(aircraft.engine_xml, encoding="utf-8") as f:
            engine = f.read()
        self.max_thrust = float(re.search(r"<maxthrust>\s*([\d.]+)", engine).group(1))
        self.mil_thrust = float(re.search(r"<milthrust>\s*([\d.]+)", engine).group(1))


def load_manual(config, power):
    columns = defaultdict(list)
    for r in csv.DictReader(open(config.manual_csv)):
        if r["power"] != power or int(r["gross_weight_lb"]) not in config.weights:
            continue
        columns[(int(r["altitude_ft"]), int(r["gross_weight_lb"]))].append(
            {"mach": float(r["mach"]), "time": float(r["time_s"]), "fuel": float(r["fuel_lb"]), "row": r["row"]})
    for rows in columns.values():
        rows.sort(key=lambda r: r["time"])
    return columns


def load_drag(config):
    by_alt = defaultdict(list)
    for r in csv.DictReader(open(config.drag_csv)):
        by_alt[int(float(r["altitude_ft"]))].append(r)
    # The sampler flies sea level at 200 ft.
    return {(0 if a == 200 else a): Drag(rows) for a, rows in by_alt.items()}


# --- fit --------------------------------------------------------------------

# Aero-Matic's generic augmented-turbofan tables, as the engine file had them
# before this fit (JSBSim utils/aeromatic++/Systems/Propulsion.cpp). Used
# only for their SHAPE where the flight data does not reach: Mach rows
# outside the data, below sea level, and MIL from 30 000 to 40 000 ft.
GENERIC = {
    "MilThrust": """
          -10000       0   10000   20000   30000   40000   50000
     0.0   1.2600  1.0000  0.7400  0.5340  0.3720  0.2410  0.1490
     0.2   1.1710  0.9340  0.6970  0.5060  0.3550  0.2310  0.1430
     0.4   1.1500  0.9210  0.6920  0.5060  0.3570  0.2330  0.1450
     0.6   1.1810  0.9510  0.7210  0.5320  0.3780  0.2480  0.1540
     0.8   1.2580  1.0200  0.7820  0.5820  0.4170  0.2750  0.1700
     1.0   1.3690  1.1200  0.8710  0.6510  0.4750  0.3150  0.1950
     1.2   1.4850  1.2300  0.9750  0.7440  0.5450  0.3640  0.2250
     1.4   1.5941  1.3400  1.0860  0.8450  0.6280  0.4240  0.2630
""",
    "AugThrust": """
          -10000       0   10000   20000   30000   40000   50000
     0.0   1.1816  1.0000  0.8184  0.6627  0.5280  0.3756  0.2327
     0.2   1.1308  0.9599  0.7890  0.6406  0.5116  0.3645  0.2258
     0.4   1.1150  0.9474  0.7798  0.6340  0.5070  0.3615  0.2240
     0.6   1.1284  0.9589  0.7894  0.6420  0.5134  0.3661  0.2268
     0.8   1.1707  0.9942  0.8177  0.6647  0.5309  0.3784  0.2345
     1.0   1.2411  1.0529  0.8648  0.7017  0.5596  0.3983  0.2467
     1.2   1.3287  1.1254  0.9221  0.7462  0.5936  0.4219  0.2614
     1.4   1.4365  1.2149  0.9933  0.8021  0.6360  0.4509  0.2794
     1.6   1.5711  1.3260  1.0809  0.8700  0.6874  0.4860  0.3011
     1.8   1.7301  1.4579  1.1857  0.9512  0.7495  0.5289  0.3277
     2.0   1.8314  1.5700  1.3086  1.0474  0.8216  0.5786  0.3585
     2.2   1.9700  1.6900  1.4100  1.2400  0.9100  0.6359  0.3940
     2.4   2.0700  1.8000  1.5300  1.3400  1.0000  0.7200  0.4600
     2.6   2.2000  1.9200  1.6400  1.4400  1.1000  0.8000  0.5200
""",
}

# Combat speed, per the manual, is where 500 ft/min of climb potential
# remains at the thrust setting: Ps = 500/60 ft/s. Weighted so 1 ft/s of
# Ps error there costs like 1 s of acceleration-time error.
COMBAT_PS = 500.0 / 60.0
COMBAT_WEIGHT = 1.0


def refine(config, name, machs, values):
    """Adds the type's extra Mach rows to a table, linearly interpolated (as JSBSim reads it)."""
    extra = [m for m in config.refine.get(name, ()) if not np.any(np.isclose(machs, m))]
    if not extra:
        return machs, values
    new_machs = np.sort(np.concatenate([machs, extra]))
    new_values = np.array([[np.interp(m, machs, values[:, c]) for c in range(values.shape[1])] for m in new_machs])
    return new_machs, new_values


def generic(config, name):
    lines = GENERIC[name].strip("\n").splitlines()
    altitudes = [float(a) for a in lines[0].split()]
    machs = np.array([float(line.split()[0]) for line in lines[1:]])
    values = np.array([[float(v) for v in line.split()[1:]] for line in lines[1:]])
    return altitudes, refine(config, name, machs, values)[1]


def specific_power(thrust, drag, altitude, weight, mach):
    v = mach * isa(altitude)[1]
    return v * (thrust * math.cos(drag.alpha_rad(mach, weight)) - drag.drag(mach, weight)) / weight


def fit_altitude(altitude, columns, drag, machs, start_thrust, shape, weights):
    """Fits thrust (lbf) at the table's Mach rows for one altitude.

    start_thrust is the current table (the initial guess); shape is the
    generic table, which rows outside the data follow, scaled to the
    nearest fitted row.
    """
    timed, combat = {}, {}
    for w in weights:
        rows = [r for r in columns[(altitude, w)] if r["mach"] <= drag.top]
        timed[w] = [r for r in rows if r["row"] != "combat"]
        combat[w] = [r for r in rows if r["row"] == "combat"]
    lo = min(r["mach"] for rows in timed.values() for r in rows)
    hi = max(r["mach"] for w in weights for r in timed[w] + combat[w])
    # Rows within 0.1 Mach of the data (the table's rows are 0.2 apart):
    # further out a row would be set by a sliver of data and the smoothing.
    free = [i for i, m in enumerate(machs) if lo - 0.1 - 1e-9 <= m <= hi + 0.1 + 1e-9]

    def thrust_curve(x):
        values = np.empty(len(machs))
        values[free] = x
        for i in range(len(machs)):
            if i < free[0]:
                values[i] = shape[i] * x[0] / shape[free[0]]
            elif i > free[-1]:
                values[i] = shape[i] * x[-1] / shape[free[-1]]
        return values

    def residuals(x):
        values = thrust_curve(x)
        thrust = lambda m: np.interp(m, machs, values)
        res = []
        for w in weights:
            rows = timed[w]
            times = fly(thrust, drag, altitude, w, rows)
            if times is None:
                res.extend([100.0] * (len(rows) - 1))
            else:
                res.extend(np.diff(times) - np.diff([r["time"] for r in rows]))
            for r in combat[w]:
                weight = w - r["fuel"]
                res.append(COMBAT_WEIGHT * (specific_power(thrust(r["mach"]), drag, altitude, weight, r["mach"])
                                            - COMBAT_PS))
        # Thrust varies smoothly with Mach: penalise the curve's curvature,
        # so rows the data barely reaches follow their neighbours instead of
        # trading off against them. On log thrust, so it is scale-free and
        # does not depend on what table the fit started from.
        res.extend(SMOOTHING * np.diff(np.log(values[free[0]:free[-1] + 1]), 2))
        return np.array(res)

    x0 = start_thrust[free]
    fit = least_squares(residuals, x0, bounds=(0.2 * x0, 3.0 * x0), x_scale=x0, diff_step=1e-3)
    values = thrust_curve(fit.x)
    thrust = lambda m: np.interp(m, machs, values)
    report = []
    for w in weights:
        rows = timed[w]
        times = fly(thrust, drag, altitude, w, rows)
        combat_ps = [(r["mach"], specific_power(thrust(r["mach"]), drag, altitude, w - r["fuel"], r["mach"]))
                     for r in combat[w]]
        report.append((w, rows, times, np.array([r["time"] for r in rows]), combat_ps))
    skipped = sorted({r["mach"] for w in weights for r in columns[(altitude, w)] if r["mach"] > drag.top})
    return values, free, report, skipped


def fit_power(config, power, xml, drags, verbose):
    name = "MilThrust" if power == "MIL" else "AugThrust"
    machs, altitudes, table = read_table(xml, name)
    machs, table = refine(config, name, machs, table)
    idle_machs, _, idle = read_table(xml, "IdleThrust")
    generic_altitudes, generic_table = generic(config, name)
    columns = load_manual(config, power)
    fit_altitudes = sorted({a for a, _ in columns})

    def idle_at(col, m):
        return np.interp(m, idle_machs, idle[:, col])

    def to_thrust(col, normalized):
        if power == "MAXAB":
            return normalized * config.max_thrust
        # JSBSim FGTurbine at MIL: T = milthrust * (idle + (1 - idle) * table)
        return config.mil_thrust * (idle_at(col, machs) + (1.0 - idle_at(col, machs)) * normalized)

    def from_thrust(col, thrust):
        if power == "MAXAB":
            return thrust / config.max_thrust
        i = idle_at(col, machs)
        return (thrust / config.mil_thrust - i) / (1.0 - i)

    def generic_column(altitude):
        return generic_table[:, generic_altitudes.index(altitude)]

    new = table.copy()
    last_fitted = {}  # altitude -> highest Mach row the fit set
    print("\n=== %s ===" % name)
    for altitude in fit_altitudes:
        col = list(altitudes).index(altitude)
        start = to_thrust(col, table[:, col])
        shape = to_thrust(col, generic_column(altitude))
        values, free, report, skipped = fit_altitude(altitude, columns, drags[altitude], machs, start, shape,
                                                     config.weights)
        last_fitted[altitude] = free[-1]
        new[:, col] = from_thrust(col, values)
        print("\n%6d ft   Mach   generic lbf   fitted lbf   ratio" % altitude)
        for i, m in enumerate(machs):
            mark = "*" if i in free else " "
            print("          %s%.1f  %10.0f  %10.0f   %.3f" % (mark, m, shape[i], values[i], values[i] / shape[i]))
        for w, rows, times, manual, combat_ps in report:
            if times is None:
                print("   %d lb: does not reach the last row" % w)
                continue
            err = times - manual
            worst = np.argmax(np.abs(err))
            line = "   %d lb: rms %.1f s, worst %+.1f s at Mach %.2f (manual %.0f s)" % (
                w, math.sqrt(np.mean(np.diff(err) ** 2)), err[worst], rows[worst]["mach"], manual[worst])
            for m, ps in combat_ps:
                line += "; Ps %.1f ft/s at combat speed Mach %.2f (manual 8.3)" % (ps, m)
            print(line)
            if verbose:
                for r, t, mt in zip(rows, times, manual):
                    print("        M %.2f  model %6.1f  manual %4.0f  %s" % (r["mach"], t, mt, r["row"]))
        if skipped:
            print("   not yet fitted (beyond sampled drag, M %.2f): %s" % (drags[altitude].top, skipped))

    # Mach rows above an altitude's data follow the Mach shape of the next
    # higher altitude whose data reach further -- installed thrust's Mach
    # dependence above Mach 1.6 or so is mostly the inlet's shock recovery,
    # a function of Mach -- rather than the generic table's.
    for altitude in sorted(last_fitted, reverse=True):
        col, last = list(altitudes).index(altitude), last_fitted[altitude]
        higher = [a for a in last_fitted if a > altitude and last_fitted[a] > last]
        if not higher:
            continue
        ref = new[:, list(altitudes).index(min(higher))]
        for i in range(last + 1, len(machs)):
            new[i, col] = new[last, col] * ref[i] / ref[last]

    # Below sea level (cold days): the generic table's ratio to sea level.
    sl = list(altitudes).index(0)
    minus = list(altitudes).index(-10000)
    new[:, minus] = generic_column(-10000) / generic_column(0) * new[:, sl]
    # Above the highest fitted altitude: in the isothermal stratosphere
    # corrected thrust would be constant, so thrust would scale with
    # pressure, delta. It falls faster in practice: at the low Reynolds
    # numbers of very high altitude, component losses grow. No engine data
    # sizes that, so the extra lapse is one parameter, T ~ delta^(1 + k)
    # above the fitted altitudes, k calibrated in thrust_fit.json (see its
    # note for what it was fitted to). MIL's data stops at 30 000 ft, below
    # the tropopause: 40 000 ft takes the generic table's 40k/30k ratio
    # first.
    top = max(fit_altitudes)
    envelope = config.envelope.get(power, [])
    if envelope:
        top = fit_envelope_column(config, power, envelope, drags, machs, altitudes, new, top, to_thrust,
                                  from_thrust, generic_column)

    # Done in pounds of thrust, then converted back: MIL table values sit on
    # top of the idle term (FGTurbine), which must lapse with them.
    for col, altitude in enumerate(altitudes):
        if altitude <= top:
            continue
        below = col - 1
        thrust_below = to_thrust(below, new[:, below])
        if power == "MIL" and altitude == 40000:
            thrust = thrust_below * generic_column(40000) / generic_column(30000)
        else:
            ratio = isa(altitude)[0] / isa(altitudes[below])[0]
            extra = config.extra_lapse if power == "MAXAB" else config.extra_lapse_mil
            thrust = thrust_below * ratio ** (1.0 + extra)
        new[:, col] = from_thrust(col, thrust)
    return name, machs, altitudes, new


def fit_envelope_column(config, power, envelope, drags, machs, altitudes, new, top, to_thrust, from_thrust,
                        generic_column):
    """Fits the 40 000 ft column to climb-envelope points; returns the new top fitted altitude.

    The accelerations stop at 30 000 ft for MIL, and its 30 000 -> 40 000 ft
    step would otherwise be the generic table's ratio. A climb envelope (the
    altitude, at each Mach, where some climb rate remains at 1 g) gives
    thrust at altitude directly: T cos(alpha) = D + W Ps / V there. Thrust at
    each point's altitude is the 40 000 ft column scaled with pressure (and
    the setting's extra lapse); the column's rows within 0.1 Mach of the
    points are fitted, the rest keep the starting shape.
    """
    col = list(altitudes).index(40000)
    below = list(altitudes).index(top)
    start = to_thrust(below, new[:, below]) * generic_column(40000) / generic_column(top)
    lo = min(e["mach"] for e in envelope)
    hi = max(e["mach"] for e in envelope)
    free = [i for i, m in enumerate(machs) if lo - 0.1 - 1e-9 <= m <= hi + 0.1 + 1e-9]
    extra = config.extra_lapse if power == "MAXAB" else config.extra_lapse_mil

    def curve(x):
        values = start.copy()
        values[free] = x
        for i in range(len(machs)):
            if i < free[0]:
                values[i] = start[i] * x[0] / start[free[0]]
            elif i > free[-1]:
                values[i] = start[i] * x[-1] / start[free[-1]]
        return values

    def residuals(x):
        values = curve(x)
        res = []
        for e in envelope:
            ratio = (isa(e["altitude"])[0] / isa(40000)[0]) ** (1.0 + extra)
            thrust = np.interp(e["mach"], machs, values) * ratio
            drag = drags[int(e["altitude"])]
            res.append(specific_power(thrust, drag, e["altitude"], e["gross"], e["mach"]) - e["ps"])
        res.extend(SMOOTHING * np.diff(np.log(values[free[0]:free[-1] + 1]), 2))
        return np.array(res)

    x0 = start[free]
    fit = least_squares(residuals, x0, bounds=(0.2 * x0, 3.0 * x0), x_scale=x0, diff_step=1e-3)
    values = curve(fit.x)
    new[:, col] = from_thrust(col, values)
    print("\n 40000 ft fitted to the climb envelope (Ps at 1 g):")
    for e, r in zip(envelope, residuals(fit.x)):
        print("   Mach %.2f at %.0f ft: Ps %+.1f ft/s from the envelope's %.1f" % (e["mach"], e["altitude"], r, e["ps"]))
    for i, m in enumerate(machs):
        print("          %s%.1f  %10.0f  %10.0f" % ("*" if i in free else " ", m, start[i], values[i]))
    return 40000


def idle_table(xml):
    """The idle-thrust table: its sea-level column, scaled with pressure.

    Aero-Matic's generic idle table rises with altitude -- 0.049 x milthrust
    at sea level to 0.147 at 50 000 ft, 2 600 lbf there, more than the
    engine's whole military thrust at that height. Corrected idle thrust is
    constant like any other, so idle scales with pressure from its sea-level
    row (which keeps the windmilling drag, negative, at high Mach).
    """
    machs, altitudes, idle = read_table(xml, "IdleThrust")
    sl = list(altitudes).index(0)
    new = np.array([[idle[i, sl] * isa(a)[0] for a in altitudes] for i in range(len(machs))])
    return machs, altitudes, new


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_aircraft_argument(parser)
    parser.add_argument("--write", action="store_true", help="rewrite the engine file's Mil/AugThrust tables")
    parser.add_argument("--verbose", action="store_true", help="print every row")
    args = parser.parse_args()

    config = FitConfig(AircraftData(args.aircraft))
    engine = config.aircraft.engine_xml
    xml = open(engine, encoding="utf-8").read()
    xml = write_table(xml, "IdleThrust", *idle_table(xml))
    drags = load_drag(config)
    fits = [fit_power(config, "MAXAB", xml, drags, args.verbose),
            fit_power(config, "MIL", xml, drags, args.verbose)]
    if args.write:
        for name, machs, altitudes, values in fits:
            xml = write_table(xml, name, machs, altitudes, values)
        open(engine, "w", encoding="utf-8", newline="").write(xml)
        print("\nwrote %s" % engine)


if __name__ == "__main__":
    sys.exit(main())
