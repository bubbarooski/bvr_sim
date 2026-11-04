"""Fit an aircraft type's engine fuel flow to flight-test data.

JSBSim burns fuel as thrust x TSFC (military power) or thrust x ATSFC
(afterburner). The stock engine file gave both a constant, with only a
sqrt(temperature) factor on military power, which put afterburning fuel
23% low at sea level and 11% low at 10 000 ft. A real engine's specific
fuel consumption rises with Mach (ram temperature and a smaller share of
the thrust from the fan) and falls with air temperature; this fits both.

Inputs, under data/aircraft/<id>/:
  reference/fuel_flow.csv            the flight manual's fuel-flow chart (for
                                     f16c_block50, T.O. GR1F-16CJ-1-1
                                     Fig. A8-4, MIL and MAX AB, standard day,
                                     flight test, 1 000 ft to 60 000 ft):
                                     the chart's curve vertices, digitised
                                     from the PDF's vector paths
  reference/flight_manual_accel.csv  the acceleration tables' fuel columns:
                                     fuel to reach each tabulated speed. Not
                                     fitted; they check the fit (a separate
                                     measurement of the same engine)
  reference/cruise_fuel.csv          the manual's Subsonic Cruise tables: total
                                     fuel flow in steady level flight, by
                                     altitude, true airspeed and gross weight
  fit/cruise_thrust.csv              tools/drivers/cruise_sample: the model's thrust
                                     and engine speed at each of those points
  fit/fuel_fit.json                  highest altitude fitted, per power; the
                                     gross weight held out of the cruise fit
  the engine file                    the thrust tables (tools/fit/thrust_fit.py
                                     must have been run first: fuel flow is
                                     fuel per unit of the model's thrust)

For afterburning the fuel flow is

    ATSFC = c0 * theta^p * (1 + c1 * Mach)        lb/hr per lbf

with theta the static temperature ratio, three numbers. The shape is the
standard one for a turbofan (Raymer, Aircraft Design, installed engine
performance: TSFC rises with Mach and as the square root of temperature);
the exponent p is left free because an afterburning engine's fuel flow need
not follow sqrt(theta) exactly. c0, c1 and p are CALIBRATED to the chart: the
model's thrust x ATSFC is fitted to the chart's fuel flow by least squares on
log flow (weighted by the square root of the flow: the fuel burned is mostly
at the high-flow end), but only where the thrust tables are themselves
fitted to flight data: the Mach range the acceleration tables cover at each
altitude, up to the altitude set in fuel_fit.json. Outside it the thrust is
an extrapolation, so a fuel flow fitted there would put the thrust error
into the TSFC; the chart there checks thrust and fuel flow together instead.
Below the lowest fitted Mach the TSFC is held at its value there (a static
F100-PW-229 is published at about 0.74 military, 2.05 afterburning).

Military power is the same form at full power (n = 1) times a part-throttle
factor, and is fitted jointly to the chart and to the manual's steady cruise
tables, which measure fuel flow at part throttle (thrust = drag):

    TSFC = c0 theta^p (1 + c1 M) (1 + c6 u^2) (1 + c7 u v)        full power
         x (1 + a x + b x^2) (1 + c5 x u) (1 + c8 x v)            part throttle

with x = 1 - N2norm (0 at full power, so the chart fixes the first line),
u = M - 0.7 and v = theta - 0.85 (M held to 0.3-1.0 in the u terms): nine
numbers, all CALIBRATED. tools/drivers/cruise_sample flies each cruise point in the
model for the thrust and engine speed it needs; the fit is to the points at
every gross weight but the one in fuel_fit.json's cruise_holdout_gross_weight_lb,
which validation flies, so the validation rows are not fitted. The cruise
points carry the model's drag as well as its TSFC: where the two disagree
with the manual the fit cannot say which is wrong, and the part-throttle
factor absorbs it (REFERENCE.md, Fuel flow).

The acceleration tables' fuel, flown at five weights per altitude, is an
independent check of both.

Run from the repo root:
    python tools/fit/fuel_fit.py --aircraft f16c_block50            # fit and report
    python tools/fit/fuel_fit.py --aircraft f16c_block50 --write    # also rewrite the engine's TSFC / ATSFC
"""

import argparse
import csv
import math
import re
import sys
from collections import defaultdict

import numpy as np
from scipy.optimize import least_squares

import thrust_fit as tf
from aircraft_data import AircraftData, add_aircraft_argument

# JSBSim's FGTurbine: at part throttle TSFC is scaled by 0.84 + (1 - N2norm)^2.
PART_THROTTLE_BASE = 0.84

# Lowest Mach the fit has data at; below it TSFC is held (set from the data).
MACH_FLOOR = 0.3

START = {"MAXAB": [1.9, 0.2, 0.7]}
MIL_START = [0.6, 0.8, 0.6, -0.5, 2.3, 0.7, 0.0, 0.0, 0.0]
OUTSIDE_WEIGHT = 0.5
MIL_UPPER_MACH = 1.0  # the u terms are held to this Mach: the cruise data stop below it


def theta(altitude_ft):
    """Static temperature ratio, US Standard Atmosphere 1976."""
    a = tf.isa(altitude_ft)[1] / 1116.45
    return a * a


def tsfc(params, mach, altitude):
    """Afterburning TSFC (three numbers)."""
    c0, c1, p = params
    return c0 * theta(altitude) ** p * (1.0 + c1 * max(mach, MACH_FLOOR))


def _uv(mach, altitude):
    return min(max(mach, MACH_FLOOR), MIL_UPPER_MACH) - 0.7, theta(altitude) - 0.85


def mil_full(q, mach, altitude):
    """Military TSFC at full power (N2norm = 1)."""
    u, v = _uv(mach, altitude)
    return (q[0] * theta(altitude) ** q[2] * (1.0 + q[1] * max(mach, MACH_FLOOR)) * (1.0 + q[6] * u * u) *
            (1.0 + q[7] * u * v))


def mil_part(q, n2norm, mach, altitude):
    """The part-throttle factor on it."""
    u, v = _uv(mach, altitude)
    x = max(1.0 - n2norm, 0.0)
    return (1.0 + q[3] * x + q[4] * x * x) * (1.0 + q[5] * x * u) * (1.0 + q[8] * x * v)


class Thrust:
    """The engine file's thrust at full military power or full afterburner,
    as FGTurbine computes it, as a function of Mach and altitude."""

    def __init__(self, config, xml):
        self.config = config
        self.tables = {name: tf.read_table(xml, name) for name in ("IdleThrust", "MilThrust", "AugThrust")}

    @staticmethod
    def lookup(table, mach, altitude):
        machs, altitudes, values = table
        column = [np.interp(mach, machs, values[:, j]) for j in range(len(altitudes))]
        return float(np.interp(altitude, altitudes, column))

    def __call__(self, power, mach, altitude):
        if power == "MAXAB":
            return self.lookup(self.tables["AugThrust"], mach, altitude) * self.config.max_thrust
        idle = self.lookup(self.tables["IdleThrust"], mach, altitude)
        return self.config.mil_thrust * (idle + (1.0 - idle) * self.lookup(self.tables["MilThrust"], mach, altitude))


def load_chart(config):
    """power -> [(altitude, mach, fuel flow lb/hr)] from the fuel-flow chart."""
    chart = defaultdict(list)
    for r in csv.DictReader(open(config.aircraft.reference("fuel_flow.csv"))):
        chart[r["power"]].append((float(r["altitude_ft"]), float(r["mach"]), float(r["fuel_flow_pph"])))
    return chart


def load_columns(config):
    """(power, altitude, gross weight) -> [(mach, time, cumulative fuel)] from the first row."""
    columns = defaultdict(list)
    for r in csv.DictReader(open(config.manual_csv)):
        if r["row"] not in ("table", "limit"):
            continue
        columns[(r["power"], int(r["altitude_ft"]), int(r["gross_weight_lb"]))].append(
            (float(r["mach"]), float(r["time_s"]), float(r["fuel_lb"])))
    for rows in columns.values():
        rows.sort(key=lambda row: row[1])
    return columns


def data_region(columns):
    """(power, altitude) -> (lowest, highest) Mach of the acceleration tables there."""
    region = {}
    for (power, altitude, _), rows in columns.items():
        lo, hi = region.get((power, altitude), (9.0, 0.0))
        region[(power, altitude)] = (min(lo, min(r[0] for r in rows)), max(hi, max(r[0] for r in rows)))
    return region


def in_region(region, power, altitude, mach):
    """True where the thrust tables rest on flight data: between the Mach range of the
    nearest tabulated altitudes, interpolated."""
    altitudes = sorted(a for p, a in region if p == power)
    lo = np.interp(altitude, altitudes, [region[(power, a)][0] for a in altitudes])
    hi = np.interp(altitude, altitudes, [region[(power, a)][1] for a in altitudes])
    return lo - 0.01 <= mach <= hi + 0.01


def column_fuel(thrust, power, altitude, rows, tsfc_at):
    """The model's cumulative fuel at each row of one manual column, integrating
    thrust x TSFC over the manual's own segment times."""
    fuel = [0.0]
    for a, b in zip(rows, rows[1:]):
        samples = 4
        flow = 0.0
        for k in range(samples):
            m = a[0] + (b[0] - a[0]) * (k + 0.5) / samples
            flow += thrust(power, m, altitude) * tsfc_at(m, altitude)
        fuel.append(fuel[-1] + flow / samples * (b[1] - a[1]) / 3600.0)
    return fuel


def load_cruise(config, n2_idle, n2_max):
    """The manual's cruise points with the model's thrust and engine speed there:
    dicts of mach, altitude, gross, thrust, n2norm, flow (the manual's lb/hr)."""
    manual = {}
    for r in csv.DictReader(open(config.aircraft.reference("cruise_fuel.csv"))):
        manual[(int(r["altitude_ft"]), int(r["gross_weight_lb"]), int(r["ktas"]))] = float(r["fuel_flow_pph"])
    rows = []
    for r in csv.DictReader(open(config.aircraft.fit("cruise_thrust.csv"))):
        if r["steady"] != "1":
            continue
        key = (int(r["altitude_ft"]), int(r["gross_weight_lb"]), int(r["ktas"]))
        rows.append({"mach": float(r["mach"]), "altitude": float(key[0]), "gross": key[1], "ktas": key[2],
                     "thrust": float(r["thrust_lbf"]), "n2norm": (float(r["n2"]) - n2_idle) / (n2_max - n2_idle),
                     "flow": manual[key]})
    # Full military power (the chart's job) and the transonic climb in drag at
    # the speed limit are left out: the fit is for steady part-throttle cruise.
    return [c for c in rows if c["n2norm"] < 0.97 and c["mach"] < 0.96]


def chart_points(chart, power, fit_max_altitude, region):
    return [pt for pt in chart[power] if pt[0] <= fit_max_altitude and in_region(region, power, pt[0], pt[1])]


def report_chart(chart, power, fit_max_altitude, region, flow_ratio):
    """Model flow / chart flow by altitude, inside the fitted region and outside it."""
    print("    chart fuel flow, model vs chart, by altitude (rms / worst; fitted region | outside it):")
    for altitude in sorted({pt[0] for pt in chart[power]}):
        at = [pt for pt in chart[power] if pt[0] == altitude]
        parts = []
        for inside in (True, False):
            group = [pt for pt in at if (altitude <= fit_max_altitude and in_region(region, power, altitude, pt[1])) == inside]
            if not group:
                parts.append("      --      ")
                continue
            errors = [100.0 * (flow_ratio(h, m, f) - 1.0) for h, m, f in group]
            parts.append("%5.1f%% /%+5.1f%%" % (math.sqrt(np.mean(np.square(errors))), max(errors, key=abs)))
        print("      %6d ft  %s | %s" % (altitude, parts[0], parts[1]))


def report_columns(thrust, power, columns, tsfc_at, verbose):
    worst = 0.0
    for (p, altitude, weight), rows in sorted(columns.items()):
        if p != power:
            continue
        model = column_fuel(thrust, power, altitude, rows, tsfc_at)[-1]
        error = 100.0 * (model - rows[-1][2]) / rows[-1][2]
        worst = max(worst, abs(error))
        if verbose:
            print("    accel %5d ft  %5d lb  manual %5.0f lb  model %5.0f lb  %+5.1f%%" % (
                altitude, weight, rows[-1][2], model, error))
    print("    acceleration-table fuel (not fitted): worst column %.1f%%" % worst)


def fit_afterburner(thrust, chart, fit_max_altitude, columns, verbose):
    power = "MAXAB"
    region = data_region(columns)
    points = chart_points(chart, power, fit_max_altitude, region)

    def residuals(params, weighted):
        out = np.array([math.log(thrust(power, m, h) * tsfc(params, m, h) / flow) for h, m, flow in points])
        if weighted:
            # What matters to the fuel burned is the high-flow end of each
            # curve, which has few vertices.
            out = out * np.array([math.sqrt(flow / 10000.0) for _, _, flow in points])
        return out

    params = least_squares(residuals, START[power], args=(True,)).x
    rms = math.sqrt(np.mean(residuals(params, False) ** 2))
    print("%s: TSFC = %.4f theta^%.3f (1 + %.4f M)   rms %.1f%% of fuel flow, %d chart points in the thrust-data region" % (
        power, params[0], params[2], params[1], 100.0 * rms, len(points)))
    print("    TSFC at M0.3 sea level %.3f, M0.9 sea level %.3f, M1.5 40 000 ft %.3f" % (
        tsfc(params, 0.3, 0), tsfc(params, 0.9, 0), tsfc(params, 1.5, 40000)))
    report_chart(chart, power, fit_max_altitude, region,
                 lambda h, m, f: thrust(power, m, h) * tsfc(params, m, h) / f)
    report_columns(thrust, power, columns, lambda m, h: tsfc(params, m, h), verbose)
    return params


def fit_military(thrust, chart, fit_max_altitude, columns, cruise, holdout_gross, verbose):
    power = "MIL"
    region = data_region(columns)
    points = chart_points(chart, power, fit_max_altitude, region)
    fitted = [c for c in cruise if c["gross"] != holdout_gross]
    held = [c for c in cruise if c["gross"] == holdout_gross]

    def cruise_ratio(q, c):
        return (c["thrust"] * mil_full(q, c["mach"], c["altitude"]) *
                mil_part(q, c["n2norm"], c["mach"], c["altitude"]) / c["flow"])

    # Chart points at or below the fit altitude but outside the thrust-data region
    # (the thrust there is extrapolated) enter at half weight: they keep the
    # fit from running away at the Mach and altitude ends the data skip.
    outside = [pt for pt in chart[power] if pt[0] <= fit_max_altitude and pt not in points]

    def residuals(q):
        out = [thrust(power, m, h) * mil_full(q, m, h) / flow - 1.0 for h, m, flow in points]
        out += [OUTSIDE_WEIGHT * (thrust(power, m, h) * mil_full(q, m, h) / flow - 1.0) for h, m, flow in outside]
        out += [cruise_ratio(q, c) - 1.0 for c in fitted]
        return np.array(out)

    q = least_squares(residuals, MIL_START).x
    chart_rms = math.sqrt(np.mean([(thrust(power, m, h) * mil_full(q, m, h) / flow - 1.0) ** 2 for h, m, flow in points]))
    fit_errors = np.array([cruise_ratio(q, c) - 1.0 for c in fitted])
    print("%s: TSFC = %.4f theta^%.3f (1 + %.4f M)(1 %+.3f u^2)(1 %+.3f u v) x (1 %+.3f x %+.3f x^2)(1 %+.3f x u)(1 %+.3f x v)" % (
        power, q[0], q[2], q[1], q[6], q[7], q[3], q[4], q[5], q[8]))
    print("    chart %.1f%% rms over %d points in the thrust-data region; cruise %.1f%% rms over %d points (all weights but %d lb)" % (
        100.0 * chart_rms, len(points), 100.0 * math.sqrt(np.mean(fit_errors ** 2)), len(fitted), holdout_gross))
    if held:
        held_errors = np.array([cruise_ratio(q, c) - 1.0 for c in held])
        print("    held-out %d lb cruise points (not fitted): %.1f%% rms, worst %+.1f%%, %d of %d within 5%%" % (
            holdout_gross, 100.0 * math.sqrt(np.mean(held_errors ** 2)), 100.0 * held_errors[np.argmax(np.abs(held_errors))],
            int(np.sum(np.abs(held_errors) <= 0.05)), len(held)))
    print("    TSFC at full power: M0.3 sea level %.3f, M0.9 sea level %.3f, M0.9 35 000 ft %.3f" % (
        mil_full(q, 0.3, 0), mil_full(q, 0.9, 0), mil_full(q, 0.9, 35000)))
    report_chart(chart, power, fit_max_altitude, region,
                 lambda h, m, f: thrust(power, m, h) * mil_full(q, m, h) / f)
    report_columns(thrust, power, columns, lambda m, h: mil_full(q, m, h), verbose)
    return q


# --- the engine file ---------------------------------------------------------

def _fmt(v):
    return "%.4f" % v


def render(node, indent):
    """node: ('value', number) | ('property', name) | (tag, [children])."""
    pad = " " * indent
    tag, body = node
    if tag == "value":
        return "%s<value> %s </value>" % (pad, _fmt(body))
    if tag == "property":
        return "%s<property>%s</property>" % (pad, body)
    inner = "\n".join(render(child, indent + 1) for child in body)
    return "%s<%s>\n%s\n%s</%s>" % (pad, tag, inner, pad, tag)


def V(x):
    return ("value", x)


def P(name):
    return ("property", name)


def N(tag, *kids):
    return (tag, list(kids))


def sum1(*terms):
    return N("sum", V(1.0), *terms)


def afterburner_tree(params):
    c0, c1, p = params
    return N("product", V(c0), N("pow", P("atmosphere/theta"), V(p)),
             sum1(N("product", V(c1), N("max", V(MACH_FLOOR), P("velocities/mach")))))


def military_tree(q, n2_idle, n2_max):
    mach = N("min", V(MIL_UPPER_MACH), N("max", V(MACH_FLOOR), P("velocities/mach")))
    u = N("difference", mach, V(0.7))
    v = N("difference", P("atmosphere/theta"), V(0.85))
    n2norm = N("quotient", N("difference", P("propulsion/engine[#]/n2"), V(n2_idle)), V(n2_max - n2_idle))
    x = N("max", V(0.0), N("difference", V(1.0), n2norm))
    return N("product", V(q[0]), N("pow", P("atmosphere/theta"), V(q[2])),
             sum1(N("product", V(q[1]), N("max", V(MACH_FLOOR), P("velocities/mach")))),
             sum1(N("product", V(q[6]), u, u)),
             sum1(N("product", V(q[7]), u, v)),
             sum1(N("product", V(q[3]), x), N("product", V(q[4]), x, x)),
             sum1(N("product", V(q[5]), x, u)),
             sum1(N("product", V(q[8]), x, v)))


def write_fuel(xml, mil, aug):
    n2_idle = float(re.search(r"<idlen2>\s*([\d.]+)", xml).group(1))
    n2_max = float(re.search(r"<maxn2>\s*([\d.]+)", xml).group(1))
    for tag, tree in (("tsfc", military_tree(mil, n2_idle, n2_max)), ("atsfc", afterburner_tree(aug))):
        pattern = r"  <%s>.*?</%s>|  <%s>[^<\n]*</%s>" % (tag, tag, tag, tag)
        block = "  <%s>\n%s\n  </%s>" % (tag, render(tree, 3), tag)
        xml, count = re.subn(pattern, lambda _: block, xml, count=1, flags=re.S)
        if count != 1:
            raise SystemExit("engine file has no <%s> element" % tag)
    return xml


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_aircraft_argument(parser)
    parser.add_argument("--write", action="store_true", help="rewrite the engine file's TSFC and ATSFC")
    parser.add_argument("--verbose", action="store_true", help="print every column")
    args = parser.parse_args()

    config = tf.FitConfig(AircraftData(args.aircraft))
    engine = config.aircraft.engine_xml
    xml = open(engine, encoding="utf-8").read()
    thrust = Thrust(config, xml)
    chart = load_chart(config)
    columns = load_columns(config)
    settings = config.aircraft.fit_settings("fuel_fit.json")
    limits = settings["fit_max_altitude_ft"]
    n2_idle = float(re.search(r"<idlen2>\s*([\d.]+)", xml).group(1))
    n2_max = float(re.search(r"<maxn2>\s*([\d.]+)", xml).group(1))
    cruise = load_cruise(config, n2_idle, n2_max)
    mil = fit_military(thrust, chart, limits["MIL"], columns, cruise, settings["cruise_holdout_gross_weight_lb"],
                       args.verbose)
    aug = fit_afterburner(thrust, chart, limits["MAXAB"], columns, args.verbose)
    if args.write:
        open(engine, "w", encoding="utf-8", newline="").write(write_fuel(xml, mil, aug))
        print("\nwrote %s" % engine)


if __name__ == "__main__":
    sys.exit(main())
