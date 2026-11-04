"""Pitch FCS design, step 2: gains from the identified airframe.

Reads the bare-airframe short-period models that tools/drivers/fcs_identify.cpp
measured over an altitude x Mach grid (data/aircraft/<id>/fit/fcs_plant.csv),
designs the load-factor loop at each point by pole placement, checks it in a
linear simulation, and writes the four gain tables into the type's JSBSim
model (for f16c_block50, data/jsbsim_data/aircraft/f16/f16.xml) between the
'fcs_design.py' marker comments.

The control law (implemented by the model's pitch channel; the F-16's is the
first to carry it):

    e     = n_c - dn                       (n_c: prefiltered, AoA-limited command)
    delta = trim + Kff*n_c - Kp*(b*n_c - dn) - Ki*integral(e) + Kq*(q - g*dn/V)

delta is the normalized elevator (+1 = 25 deg trailing-edge down), dn the
load factor above 1 G, q the pitch rate (rad/s). The damping acts on the
pitch rate the current load factor does not account for (q = g*dn/V in a
steady pull-up), so in a sustained pull it carries no static elevator for
the integrator to hold -- with plain q it carried ~0.2 of elevator, which
the integrator could not track as the aircraft bled through transonic.
With n_c = 0 the law is state feedback on [alpha, q, integral(e)], so Kp,
Ki and Kq place all three closed-loop poles:
  * the short period at CAP = omega^2 / (n/alpha) = 1.0, damping 0.75 --
    inside MIL-F-8785C Level 1 for Category A (CAP 0.28-3.6, damping
    0.35-1.30). omega is clipped to 1.8-7 rad/s: the elevator rate limit
    sets the top, and the slowest corners would otherwise be sluggish.
  * the integrator pole at -min(omega, 3) rad/s.
The command path (feed-forward at half the static elevator-per-G, a 0.15 s
prefilter and setpoint weight b = 0.6) shapes the step response without
moving those poles: in the linear check every point overshoots <= ~6% and
reaches 90% in <= ~1.4 s (the tool prints the exact worst case).

Usage, from the repo root:
    build/Release/tools/fcs_identify.exe --aircraft f16c_block50
    python tools/fit/fcs_design.py --aircraft f16c_block50            # report only
    python tools/fit/fcs_design.py --aircraft f16c_block50 --write    # also update the model
"""

import argparse
import csv
import math

import numpy as np
from scipy.signal import place_poles

from aircraft_data import AircraftData, add_aircraft_argument

CAP = 1.0
DAMPING = 0.75
OMEGA_MIN, OMEGA_MAX = 1.8, 7.0
INTEGRATOR_POLE_MAX = 3.0
# A point is designed only if the identification input kept alpha within
# this of trim; beyond it the fit is not a small-perturbation model.
MAX_IDENTIFICATION_DALPHA_DEG = 5.0
FEEDFORWARD_FRACTION = 0.5
PREFILTER_TAU_S = 0.15
SETPOINT_WEIGHT = 0.6
GRAVITY = 32.174
DT = 1.0 / 120.0

BEGIN = "<!-- BEGIN fcs_design.py gain tables: generated, do not edit by hand -->"
END = "<!-- END fcs_design.py gain tables -->"


def design(p):
    """Gains for one identified point; p holds the CSV row as floats."""
    za, zq, zd = p["Za"], p["Zq"], p["Zd"]
    ma, mq, md = p["Ma"], p["Mq"], p["Md"]
    na, nq, nd = p["Na"], p["Nq"], p["Nd"]

    omega = float(np.clip(math.sqrt(CAP * na), OMEGA_MIN, OMEGA_MAX))
    sigma = DAMPING * omega
    omega_d = omega * math.sqrt(1.0 - DAMPING**2)
    poles = [complex(-sigma, omega_d), complex(-sigma, -omega_d), -min(omega, INTEGRATOR_POLE_MAX)]

    # States [alpha, q, z], z = integral of (-dn) with no command.
    a = np.array([[za, zq, 0.0], [ma, mq, 0.0], [-na, -nq, 0.0]])
    b = np.array([[zd], [md], [-nd]])
    k1, k2, k3 = place_poles(a, b, poles).gain_matrix[0]

    # delta = -K x  <=>  delta = Kp*dn + Kq*q - Ki*z, with dn depending on
    # delta through Nd (an algebraic loop, solved here).
    kp = -k1 / (na - k1 * nd)
    d = 1.0 - kp * nd
    kq = -k2 * d - kp * nq
    ki = k3 * d
    # The damping's -Kq*g/V*dn is extra load-factor feedback; fold it into
    # the tabulated Kp so the total dn feedback is still the placed one.
    kp += kq * GRAVITY / p["vt_fps"]

    # Static elevator per G in a steady pull-up (q = g*dn/V).
    q_per_g = GRAVITY / p["vt_fps"]
    alpha, delta = np.linalg.solve(np.array([[za, zd], [ma, md]]), -np.array([zq, mq]) * q_per_g)
    elevator_per_g = delta / (na * alpha + nq * q_per_g + nd * delta)

    return {"omega": omega, "kp": kp, "ki": ki, "kq": kq, "kff": FEEDFORWARD_FRACTION * elevator_per_g,
            "g_over_v": GRAVITY / p["vt_fps"]}


def step_response(p, g, seconds=4.0, actuator_tau_s=0.05):
    """Unit G step through the full law, with a one-tick delay and a
    first-order actuator. Returns (overshoot fraction, time to 90%)."""
    za, zq, zd = p["Za"], p["Zq"], p["Zd"]
    ma, mq, md = p["Ma"], p["Mq"], p["Md"]
    na, nq, nd = p["Na"], p["Nq"], p["Nd"]
    alpha = q = z = delta = n_c = 0.0
    pending = 0.0
    peak, t90 = 0.0, None
    for k in range(int(seconds / DT)):
        n_c += (1.0 - n_c) * DT / PREFILTER_TAU_S
        dn = na * alpha + nq * q + nd * delta
        command = (g["kff"] * n_c - g["kp"] * (SETPOINT_WEIGHT * n_c - dn) - g["ki"] * z +
                   g["kq"] * (q - g["g_over_v"] * dn))
        delta += (pending - delta) * DT / actuator_tau_s
        pending = command
        alpha += (za * alpha + zq * q + zd * delta) * DT
        q += (ma * alpha + mq * q + md * delta) * DT
        z += (n_c - dn) * DT
        peak = max(peak, dn)
        if t90 is None and dn >= 0.9:
            t90 = k * DT
    return peak - 1.0, t90 if t90 is not None else float("inf")


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: (float(v) if v not in ("", None) else None) for k, v in r.items()})
    return rows


def fill_untrimmable(grid, altitudes, machs):
    """Corners the aircraft cannot trim at (too slow high up, too fast low
    down) take the nearest trimmed Mach at the same altitude, so JSBSim's
    table interpolation never reaches into an undefined cell."""
    for alt in altitudes:
        trimmed = [m for m in machs if grid.get((alt, m)) is not None]
        for m in machs:
            if grid.get((alt, m)) is None:
                nearest = min(trimmed, key=lambda t: abs(t - m))
                grid[(alt, m)] = grid[(alt, nearest)]


def table_xml(name, key, grid, altitudes, machs, note):
    lines = [f'   <!-- {note} -->', f'   <fcs_function name="fcs/{name}">', "    <function>", "     <table>",
             '      <independentVar lookup="row">velocities/mach</independentVar>',
             '      <independentVar lookup="column">position/h-sl-ft</independentVar>', "      <tableData>",
             "             " + "".join(f"{a:>10.0f}" for a in altitudes)]
    for m in machs:
        lines.append(f"        {m:5.2f}" + "".join(f"{grid[(a, m)][key]:10.5f}" for a in altitudes))
    lines += ["      </tableData>", "     </table>", "    </function>", "   </fcs_function>"]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_aircraft_argument(parser)
    parser.add_argument("--write", action="store_true", help="update the gain tables in the type's model")
    args = parser.parse_args()

    aircraft = AircraftData(args.aircraft)
    xml_path = aircraft.model_xml
    rows = load(aircraft.fit("fcs_plant.csv"))
    altitudes = sorted({r["alt_ft"] for r in rows})
    machs = sorted({r["mach"] for r in rows})
    grid = {}
    worst_overshoot, worst_t90 = (0.0, None), (0.0, None)
    print(f"{'alt ft':>7} {'Mach':>5} {'n/a':>6} {'omega':>6} {'CAP':>5} {'Kp':>8} {'Ki':>8} {'Kq':>8} "
          f"{'Kff':>8} {'over':>6} {'t90':>5}")
    for r in rows:
        key = (r["alt_ft"], r["mach"])
        if r["trimmed"] != 1.0:
            grid[key] = None
            print(f"{r['alt_ft']:7.0f} {r['mach']:5.2f}   not trimmable")
            continue
        if r["max_dalpha_deg"] > MAX_IDENTIFICATION_DALPHA_DEG:
            # Trimmed at the edge of the envelope, where the input swings
            # alpha far beyond a small perturbation: the fitted linear model
            # does not describe the aircraft there (at 50 000 ft / M0.5 it
            # makes the placement singular). Filled like an untrimmable
            # corner.
            grid[key] = None
            print(f"{r['alt_ft']:7.0f} {r['mach']:5.2f}   identification not small-perturbation "
                  f"(alpha swung {r['max_dalpha_deg']:.1f} deg)")
            continue
        g = design(r)
        overshoot, t90 = step_response(r, g)
        grid[key] = g
        worst_overshoot = max(worst_overshoot, (overshoot, key))
        worst_t90 = max(worst_t90, (t90, key))
        print(f"{r['alt_ft']:7.0f} {r['mach']:5.2f} {r['Na']:6.1f} {g['omega']:6.2f} {g['omega']**2 / r['Na']:5.2f} "
              f"{g['kp']:8.4f} {g['ki']:8.4f} {g['kq']:8.4f} {g['kff']:8.4f} {100 * overshoot:5.1f}% {t90:5.2f}")
    print(f"\nworst overshoot {100 * worst_overshoot[0]:.1f}% at {worst_overshoot[1]}; "
          f"slowest 90% {worst_t90[0]:.2f} s at {worst_t90[1]}")

    if not args.write:
        return
    fill_untrimmable(grid, altitudes, machs)
    tables = "\n\n".join([
        table_xml("g-kp", "kp", grid, altitudes, machs, "Kp: elevator per G of measured load factor"),
        table_xml("g-ki", "ki", grid, altitudes, machs, "Ki: elevator per G-second of error"),
        table_xml("g-kq", "kq", grid, altitudes, machs, "Kq: elevator per rad/s of pitch rate beyond g*dn/V"),
        table_xml("g-kff", "kff", grid, altitudes, machs, "Kff: feed-forward elevator per G commanded"),
    ])
    with open(xml_path, encoding="utf-8") as f:
        xml = f.read()
    start, end = xml.index(BEGIN) + len(BEGIN), xml.index(END)
    with open(xml_path, "w", encoding="utf-8", newline="") as f:
        f.write(xml[:start] + "\n" + tables + "\n   " + xml[end:])
    print(f"wrote gain tables to {xml_path}")


if __name__ == "__main__":
    main()
