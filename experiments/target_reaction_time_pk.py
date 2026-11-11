"""How long can the target wait before it turns and runs?

A launch zone's Rmax is the farthest shot that reaches a target flying
straight (REFERENCE.md 12.8). A target that sees the shot coming can turn
around and run, and a missile that burned its motor in the first few
seconds then has to catch it from behind on its remaining speed. Whether
that works depends on when the target turns: turn early and the missile
falls short; turn late and the missile is too close and too fast to
outrun. Somewhere between, Pk climbs from 0 to 1. The later that climb
starts, the more time the target has to see the shot and react.

This moves the Target's turn-and-run point along its route in
1_core_04_loft, so that the turn starts a set number of seconds after
the launch, and plots Pk against that reaction time for three shots:

  long shot, lofted  -- the Pursuer fires on its first confirmed track
  long shot, flat    -- the same shot with loft off (REFERENCE.md 6.10)
  short shot, lofted -- the Pursuer holds fire to half its Rmax
                        (launch_range_fraction 0.5)

Before its turn the Target flies straight and level at constant speed with
no wind, so the turn can be placed to the second: a calibration batch
finds each seed's launch time, and that seed's turn is set from it.

Measured 2026-11-10 (49 reaction times x 40 seeds per shot):

  shot             time of flight   Pk 0.5 when the     i.e. this long
                   (straight target) target turns at    before arrival
  45 nmi, lofted        91 s             60 s               31 s
  45 nmi, flat         105 s            103 s                2 s
  28 nmi, lofted        51 s             14 s               37 s

- Lofted, the 45 nmi shot is beaten by a turn in its first minute. Flat,
  the same shot is beaten by a turn almost up to the moment it arrives:
  it has cruised in thicker air, and arrives too slow to follow the
  target round.
- Holding fire to 28 nmi cuts the target's window from a minute to 14 s.
  The price is the shot itself: the Pursuer has waited 17 nmi to take it.
- Each rise is spread by the dice. When the radar confirms its track
  decides the launch range (38-47 nmi for the "45 nmi" shot), and the
  shots that win against a mid-flight turn are the shorter ones. The
  28 nmi shot always waits for the same range, so its rise is sharp.

The numbers illustrate the physics of this model; they do not predict a
real missile's performance.

Writes the figure to experiments/results/ (or to --figure PATH); --replot
redraws it from the last sweep's summary without running anything.
Requires matplotlib as well as pandas.

Run from anywhere:  python experiments/target_reaction_time_pk.py
"""

import argparse
import json
import logging
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pandas as pd

from _framework import RESULTS_DIR, REPO_ROOT, dot_to_pointer, grid_runs, load_results, run_batch

SCENARIO = "scenarios/1_core_04_loft.json"

# label: overrides. The labels are replaced by each shot's measured launch range.
SHOTS = {
    "long, lofted": {"missile_doctrine.launch_range_fraction": 1.0, "missile_doctrine.loft": True},
    "long, flat": {"missile_doctrine.launch_range_fraction": 1.0, "missile_doctrine.loft": False},
    "short, lofted": {"missile_doctrine.launch_range_fraction": 0.5, "missile_doctrine.loft": True},
}

REACTION_S = [2.5 * i for i in range(49)]  # 0 to 120 s
SEEDS_PER_LEVEL = 40
SEED_BASE = 80_000

SUMMARY = RESULTS_DIR / "target_reaction_time_pk_summary.csv"

FT_PER_M = 1.0 / 0.3048
FT_PER_S_PER_KT = 1852.0 / 3600.0 * FT_PER_M


def target_route(scenario):
    """The Target's index, start x (ft), ground speed (ft/s), route and arrival radius (ft).

    The route must open with a westbound straight leg (heading 270, the
    Target on z = 0) to a turn point, then the run-out: the reaction time
    arithmetic below holds only for that shape.
    """
    doc = json.loads((REPO_ROOT / scenario).read_text())
    env = doc.get("environment", {})
    if env.get("wind_speed_kts", 0.0) != 0.0:
        raise SystemExit("needs a still-air scenario: ground speed is taken as airspeed")
    index = next(i for i, e in enumerate(doc["entities"]) if e.get("name") == "Target")
    target = doc["entities"][index]
    jsbsim, route = target["components"]["jsbsim"], target["components"]["waypoints"]
    if target["transform"]["heading"] != 270.0 or target["transform"]["pos"][2] != 0.0:
        raise SystemExit("the Target must start westbound on z = 0")
    return {
        "index": index,
        "x0_ft": target["transform"]["pos"][0],
        "speed_ft_s": jsbsim["airspeed_kts"] * FT_PER_S_PER_KT,
        "points": route["points"],
        "arrival_ft": route.get("arrival_radius_m", 150.0) * FT_PER_M,
    }


def turn_overrides(route, turn_start_x_ft):
    """Move the turn so the Target reaches it at x = turn_start_x_ft.

    The waypoint counts as reached at its arrival radius, so the point
    itself sits that far further on; the second point keeps its offset
    from the first, so the turn's shape does not change.
    """
    wp0_x = turn_start_x_ft - route["arrival_ft"]
    leg_x = route["points"][1][0] - route["points"][0][0]
    base = f"entities[{route['index']}].components.waypoints.points"
    return {dot_to_pointer(f"{base}[0][0]"): wp0_x, dot_to_pointer(f"{base}[1][0]"): wp0_x + leg_x}


def shot_overrides(shot):
    return {dot_to_pointer(k): v for k, v in SHOTS[shot].items()}


def launch_times(route, shot):
    """Each seed's launch time, with the turn moved far down-route so it cannot precede the launch.

    When the radar confirms its track is a dice roll, so a shot fires up to
    tens of seconds earlier or later from one seed to the next. Nothing
    before the launch depends on where the turn is, so a seed launches at
    the same moment at every level: placing each seed's turn from its own
    launch time makes every run's reaction time exactly its level's.
    """
    overrides = {**shot_overrides(shot), **turn_overrides(route, route["x0_ft"] - 1_000_000.0)}
    runs, _ = grid_runs([(shot, overrides)], SEEDS_PER_LEVEL, seed_of=lambda c, i: SEED_BASE + i)
    df = load_results(run_batch(SCENARIO, runs, name="reaction_calibration"))
    if not df.launched.all():
        raise SystemExit(f"{shot}: {int((1 - df.launched).sum())} calibration runs did not launch")
    return dict(zip(df.seed, df.launch_time_s))


def sweep(route, shot, launch_time_of_seed):
    runs = []
    for level in REACTION_S:
        for seed, launch_time in launch_time_of_seed.items():
            turn_x = route["x0_ft"] - route["speed_ft_s"] * (launch_time + level)
            runs.append({"run_id": len(runs), "seed": int(seed), "level_s": level,
                         "overrides": {**shot_overrides(shot), **turn_overrides(route, turn_x)}})
    level_of_run = {r["run_id"]: r.pop("level_s") for r in runs}
    df = load_results(run_batch(SCENARIO, runs, name=f"reaction_{shot.replace(', ', '_')}"))
    df["level_s"] = df.run_id.map(level_of_run)

    wp0_pointer = dot_to_pointer(f"entities[{route['index']}].components.waypoints.points[0][0]")
    wp0_x = df.run_id.map({r["run_id"]: r["overrides"][wp0_pointer] for r in runs})
    turn_time = (route["x0_ft"] - (wp0_x + route["arrival_ft"])) / route["speed_ft_s"]
    df["reaction_s"] = turn_time - df.launch_time_s
    drift = (df.reaction_s - df.level_s).abs().max()
    if drift > 0.1:
        raise SystemExit(f"{shot}: a run launched {drift:.1f} s away from its calibrated time")
    return df


def summarise(df):
    g = df.groupby("level_s")
    out = g.agg(reaction_s=("reaction_s", "median"), runs=("hit", "size"),
                launched=("launched", "mean"), pk=("hit", "mean"))
    out["se"] = (out.pk * (1 - out.pk) / out.runs).map(math.sqrt)
    # At the latest level the Target is still flying straight when the
    # missile arrives: that level's time of flight is the shot's own.
    last = df[(df.level_s == df.level_s.max()) & (df.hit == 1)]
    out["flight_time_s"] = (last.resolution_time_s - last.launch_time_s).median()
    return out.reset_index()


def plot(curves, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MultipleLocator, PercentFormatter

    # Reference categorical slots 1-3 (validated as a set), light surface.
    surface, ink, ink_2, ink_3, rule = "#fcfcfb", "#1a1a19", "#52514e", "#8a8984", "#e6e5e1"
    colours = ["#2a78d6", "#eb6834", "#1baf7a"]
    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)  # a missing fallback font is fine
    plt.rcParams["font.family"] = ["Segoe UI", "Helvetica Neue", "Arial", "DejaVu Sans"]

    fig, ax = plt.subplots(figsize=(9.0, 5.2), dpi=200)
    fig.patch.set_facecolor(surface)
    ax.set_facecolor(surface)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.76, bottom=0.15)

    handles = []
    for (label, s), colour in zip(curves.items(), colours):
        flight = s.flight_time_s.iloc[0]
        before = s[s.reaction_s <= flight]
        if len(before) and before.pk.iloc[-1] == 1.0:
            # Already certain by the time it arrives: a later turn changes nothing, so the line ends there.
            s = pd.concat([before, pd.DataFrame({"reaction_s": [flight], "pk": [1.0], "se": [0.0]})],
                          ignore_index=True)
        x, pk, se = s.reaction_s, s.pk, s.se
        ax.fill_between(x, (pk - se).clip(0, 1), (pk + se).clip(0, 1), color=colour, alpha=0.12, lw=0)
        ax.plot(x, pk, color=colour, lw=2.2, solid_capstyle="round", solid_joinstyle="round")
        ax.plot(flight, 1.06, marker="v", ms=7, color=colour, mec=surface, mew=1,
                clip_on=False, zorder=5)
        handles.append(Line2D([], [], color=colour, lw=2.2, label=label))

        # Direct label beside the curve where it crosses 50 %, on the side with room.
        i = int((pk >= 0.5).to_numpy().argmax())
        x50 = x[i] if i == 0 else x[i - 1] + (0.5 - pk[i - 1]) / (pk[i] - pk[i - 1]) * (x[i] - x[i - 1])
        left = x50 > 30
        ax.annotate(label, (x50, 0.5), xytext=(-10 if left else 10, 0), textcoords="offset points",
                    ha="right" if left else "left", va="center", fontsize=9.5, color=ink)

    first_arrival = min(s.flight_time_s.iloc[0] for s in curves.values())
    ax.annotate("missile arrives, if the target never turns", (first_arrival, 1.06), xytext=(9, 0),
                textcoords="offset points", ha="left", va="center", fontsize=8.5, color=ink_3,
                annotation_clip=False)

    ax.set_xlim(0, max(REACTION_S))
    ax.set_ylim(-0.02, 1.02)
    ax.margins(x=0)
    ax.xaxis.set_major_locator(MultipleLocator(20))
    ax.yaxis.set_major_locator(MultipleLocator(0.25))
    ax.yaxis.set_major_formatter(PercentFormatter(1.0, decimals=0))
    ax.set_xlabel("Seconds after launch when the target turns and runs", color=ink_2, fontsize=10, labelpad=8)
    ax.set_ylabel("Kill probability", color=ink_2, fontsize=10, labelpad=8)
    ax.grid(axis="y", color=rule, lw=0.8)
    ax.axhline(0, color=ink_3, lw=0.8, zorder=0.5)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.tick_params(colors=ink_2, labelsize=9, length=0, pad=6)

    fig.text(0.08, 0.95, "The later the target turns, the surer the kill", fontsize=14, fontweight="semibold",
             color=ink, ha="left", va="top")
    fig.text(0.08, 0.895, "Loft and a shorter shot both shrink the time the target has to get away",
             fontsize=10.5, color=ink_2, ha="left", va="top")
    fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.075, 0.855), ncol=len(handles),
               frameon=False, fontsize=9, labelcolor=ink_2, handlelength=1.8, columnspacing=1.6,
               handletextpad=0.6)
    fig.text(0.08, 0.025, f"1_core_04_loft, {SEEDS_PER_LEVEL} seeds per point; shading is ±1 standard "
             "error. Illustrates this model's physics, not real-world performance.",
             fontsize=8, color=ink_3, ha="left", va="bottom")
    fig.savefig(path, facecolor=surface)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--figure", type=Path, default=RESULTS_DIR / "target_reaction_time_pk.png",
                        help="where to write the PNG")
    parser.add_argument("--replot", action="store_true",
                        help="redraw the figure from the last sweep's summary, without running anything")
    args = parser.parse_args()
    args.figure.parent.mkdir(parents=True, exist_ok=True)

    if args.replot:
        summary = pd.read_csv(SUMMARY)
        plot({label: g.reset_index(drop=True) for label, g in summary.groupby("shot", sort=False)}, args.figure)
        print(f"figure: {args.figure}")
        return

    route = target_route(SCENARIO)
    print(f"{len(SHOTS)} shots x {len(REACTION_S)} reaction times x {SEEDS_PER_LEVEL} seeds "
          f"= {len(SHOTS) * len(REACTION_S) * SEEDS_PER_LEVEL} runs, plus "
          f"{len(SHOTS) * SEEDS_PER_LEVEL} to find each seed's launch time")

    curves = {}
    for shot in SHOTS:
        df = sweep(route, shot, launch_times(route, shot))
        launched = df[df.launched == 1]
        label = f"{launched.launch_range_m.median() / 1852.0:.0f} nmi, {shot.split(', ')[1]}"
        s = summarise(df)
        curves[label] = s

        print(f"\n{label}")
        print(f"{'reaction s':>10} {'runs':>5} {'launched':>9} {'Pk':>6} {'+/-':>6}")
        for row in s.itertuples():
            print(f"{row.reaction_s:>10.1f} {row.runs:>5} {row.launched:>8.0%} {row.pk:>6.2f} {row.se:>6.2f}")
        print(f"time of flight if the Target never turns: {s.flight_time_s.iloc[0]:.1f} s")
        rng = launched.launch_range_m / 1852.0
        print(f"launch range: median {rng.median():.1f} nmi, {rng.min():.1f}-{rng.max():.1f} across seeds")

    pd.concat([s.assign(shot=label) for label, s in curves.items()]).to_csv(SUMMARY, index=False)
    plot(curves, args.figure)
    print(f"\nfigure: {args.figure}")
    print("reaction s = median seconds from launch to the Target starting its turn")
    print("Pk         = kills / runs; a run with no launch counts as no kill")


if __name__ == "__main__":
    main()
