"""What does a more sensitive launch radar buy the missile?

During midcourse the missile steers to a position the launcher radioed to
it, not to truth. That position is the launcher's radar track, whose
dominant error is angular: an angular error becomes a cross-range position
error proportional to range.

The radar's angle error per look is beamwidth / (km sqrt(2 SNR)) -- see
REFERENCE.md, Track measurement and filtering. This sweeps the pursuer
radar's reference range, the teaching knob that sets its sensitivity:
every detection range scales with it, so its SNR at any given range rises
as reference_range^4 (40 log10 dB). Two things then move together, and
the sweep reports both:

  launch range -- the launch waits for the firing tier, so a more
                  sensitive radar fires from further out
  track error  -- at a given range a more sensitive radar measures the
                  angle better, but it now hands over a longer shot

It runs on two geometries, head-on (1_core_01_first_shot) and a crossing
target that breaks away (2_combined_04_crossing_break).

Measured 2026-11-10 (5 levels x 120 seeds per geometry; TWS frames, loft):

  ref nmi          25     35     50     70    100
  head-on  launch 22.3   33.1   44.8   46.8   46.8 nmi  err p50 311 467 372 278 188 m
  crossing launch 32.0   33.3   33.3   33.3   33.3 nmi  err p50 403 266 212 120  84 m

- Head-on, sensitivity buys launch range almost in proportion until the
  launch zone caps the shot near 47 nmi; only past the cap does it go into
  the track.
- Crossing, the launch range is capped from 35 nmi up, so the extra
  sensitivity goes into the track instead: the handoff error falls
  403 -> 84 m.
- Every launched run acquired, at every level, and Pk does not move with
  sensitivity: 1.00 head-on, 0.00 crossing, where the break turns the shot
  into a tail chase the burned-out missile cannot win. A midcourse missile
  steers on the datalink with no gimbal limit (REFERENCE.md 8.1), so track
  errors of this size never decide the shot. What radar sensitivity buys
  is an earlier shot or a better track, not a higher Pk.
- The errors are several times the 54-73 m a once-a-second track update
  would give: the track updates once per 4.4 s scan frame.

Run from anywhere:  python experiments/datalink_quality_pk.py
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

REPO_ROOT = Path(__file__).resolve().parent.parent

from _framework import dot_to_pointer, grid_runs, load_results, run_batch

SEEDS_PER_LEVEL = 120

SCENARIOS = {
    "head-on": "scenarios/1_core_01_first_shot.json",
    # The target crosses, then breaks away and dives.
    "crossing": "scenarios/2_combined_04_crossing_break.json",
}

# The launcher radar's reference range, nmi: where one look at a 5 m2
# target is a coin flip. The F-16 type's is 50 (its fit/radar_fit.json);
# halving it costs 12 dB everywhere, doubling it gains 12.
REFERENCE_RANGE_NMI = [25.0, 35.0, 50.0, 70.0, 100.0]


def pursuer_reference_pointer(scenario):
    """The pursuer radar's reference range, as a JSON Pointer override."""
    entities = json.loads((REPO_ROOT / scenario).read_text())["entities"]
    index = next(i for i, e in enumerate(entities) if e.get("name") == "Pursuer")
    return dot_to_pointer(f"entities[{index}].components.airborne_radar.reference_range_nmi")


def sweep(name, scenario):
    pointer = pursuer_reference_pointer(scenario)
    runs, level_of_run = grid_runs(
        [(nmi, {pointer: nmi}) for nmi in REFERENCE_RANGE_NMI],
        SEEDS_PER_LEVEL,
        seed_of=lambda level, i: 90_000 + level * 1_000 + i,
    )

    df = load_results(run_batch(scenario, runs, name=f"datalink_{name.replace('-', '_')}"))
    df["reference_nmi"] = df.run_id.map(level_of_run)
    df = df[df.launched == 1]

    print(f"\n{name}  ({scenario.split('/')[-1]})")
    print(f"{'ref nmi':>7} {'runs':>5} {'launch nmi':>11} {'dl err p50':>11} {'p90':>8} {'acquired':>9} "
          f"{'Pk':>7} {'Pk|acq':>8}")
    print("-" * 72)
    for nmi in REFERENCE_RANGE_NMI:
        g = df[df.reference_nmi == nmi]
        acq = g[g.seeker_active_time_s.notna()]
        pk_acq = f"{acq.hit.mean():.3f}" if len(acq) else "--"
        print(f"{nmi:>7.0f} {len(g):>5} {g.launch_range_m.median() / 1852.0:>11.1f} "
              f"{g.datalink_error_at_handoff_m.median():>11.1f} "
              f"{g.datalink_error_at_handoff_m.quantile(0.90):>8.1f} "
              f"{g.seeker_active_time_s.notna().mean():>8.1%} {g.hit.mean():>7.3f} {pk_acq:>8}")
    return df


def main():
    print(f"{len(REFERENCE_RANGE_NMI)} reference ranges x {SEEDS_PER_LEVEL} seeds "
          f"x {len(SCENARIOS)} geometries = {len(REFERENCE_RANGE_NMI) * SEEDS_PER_LEVEL * len(SCENARIOS)} runs")
    for name, scenario in SCENARIOS.items():
        sweep(name, scenario)

    print()
    print("launch   = median launch range")
    print("dl err   = metres the datalinked target position was wrong at seeker handoff")
    print("acquired = fraction of launched runs whose seeker found the target")
    print("Pk|acq   = kill probability among runs that DID acquire")


if __name__ == "__main__":
    main()
