"""TEMPLATE: copy this file to start a new Monte Carlo experiment.

An experiment answers one question by running the sim many times with one
parameter varied and comparing the outcomes. This file runs end to end as
written -- a small sweep of the missile's kill radius against
1_core_01_first_shot.json -- so you can execute it once to see the whole
pipeline work, then edit the marked sections to ask your own question.

Before running:
  1. Build the headless batch runner in Release (Debug is ~10x slower):
       cmake --build build --config Release --target intercept_batch
  2. pip install -r requirements.txt   (this needs pandas)

Run from anywhere:  python experiments/_template.py

A real, fuller example once this one makes sense:
  datalink_quality_pk.py  -- the same sweep run across two geometries
  target_reaction_time_pk.py -- a sweep that moves a route point, and plots Pk
"""

import sys
from pathlib import Path

# _framework.py lives in this same directory but isn't an installed
# package -- this puts the directory on sys.path so the import below
# works no matter where you run this script FROM.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _framework import dot_to_pointer, grid_runs, load_results, run_batch

# ============================================================================
# 1. WHAT YOU'RE SWEEPING -- the only section most new experiments need to
#    change to ask a completely different question.
# ============================================================================

# Any scenario file -- the numbered teaching set under scenarios/, or your
# own under scenarios/custom/. 1_core_01_first_shot.json is a reasonable
# default: a plain head-on F-16-vs-F-16 intercept with the full
# radar/missile chain.
SCENARIO = "scenarios/1_core_01_first_shot.json"

# How many random seeds to run at EACH level of the swept parameter. Every
# run at a given level is otherwise identical -- the seed is the only
# source of randomness (sensor noise, RCS fluctuation, sweep-timing
# jitter) -- so more seeds means a tighter estimate of the true kill
# probability there, at the cost of more runs. 50 is enough to see a
# trend; 250 here gives a Pk estimate good to about +/-3 points per level.
SEEDS = 250

# The parameter to vary, and the values to try. This example sweeps the
# missile's kill_radius_m -- the proximity-fuze radius lethality is
# evaluated at. Pk is a logistic curve centered on this value (see
# REFERENCE.md, InterceptSystem's Pk(missDistance, killRadius,
# falloffWidth)), so widening it should produce a clean, monotonically
# increasing Pk with no surprises -- a safe first sweep to learn the
# pipeline on. The scenario's own default is 12.0.
#
# HOW TO FIND YOUR OWN PARAMETER PATH:
#   - docs/SCHEMA.md is the authoring reference: every scenario JSON key,
#     its unit, and its default.
#   - Write the path the way you'd address it in the JSON, with dots
#     between levels: "missile_doctrine.kill_radius_m" reaches
#     {"missile_doctrine": {"kill_radius_m": ...}}.
#   - Per-entity fields need an array index, e.g.
#     "entities[1].components.jsbsim.airspeed_kts" for the
#     SECOND entity in the scenario's "entities" array (0-indexed) --
#     open the scenario file itself to see which index is which entity.
#   - dot_to_pointer() below converts that dotted path into the JSON
#     Pointer the batch runner actually reads. A value here is patched
#     directly into a copy of the scenario JSON before it loads, so any
#     path that already exists in the scenario works -- it doesn't have
#     to be a field the GUI exposes. A SCHEMA.md key the file leaves at
#     its default works too. Anything else (a typo) stops the batch with
#     an error before any run starts.
LEVELS = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]
PARAM_POINTER = dot_to_pointer("missile_doctrine.kill_radius_m")


# ============================================================================
# 2. BUILD THE RUN LIST -- rarely needs to change; this shape works for
#    any single-parameter sweep.
# ============================================================================
def build_runs():
    """One dict per simulated run: {run_id, seed, overrides}.

    - run_id: any unique integer per run in this batch. It's just how a
      result row gets mapped back to which (level, seed) produced it.
    - seed: seeds the sim's RNG for that run. MUST be unique across the
      batch, or two "different" runs are bit-identical. The offset
      (20_000 here) is just a convention the other experiments use to
      keep different scripts' seed ranges from colliding -- pick any
      base and stay off 0.
    - overrides: {json_pointer: value}. Every key here is patched into a
      fresh copy of SCENARIO's JSON before that run loads; anything not
      listed keeps the scenario file's own value.
    """
    # grid_runs makes SEEDS runs per (label, overrides) cell. Here the
    # label is just the level itself, so the results can be grouped by it.
    return grid_runs(
        [(level, {PARAM_POINTER: level}) for level in LEVELS],
        SEEDS,
        seed_of=lambda level_index, i: 20_000 + level_index * SEEDS + i,
    )


# ============================================================================
# 3. RUN IT AND READ THE RESULTS
# ============================================================================
def main():
    runs, _ = build_runs()
    print(f"{len(LEVELS)} levels x {SEEDS} seeds = {len(runs)} runs")

    # run_batch writes a spec JSON, invokes the Release intercept_batch exe
    # against it, and returns the path to the results CSV it wrote.
    # name= just controls the output filename under experiments/results/
    # (gitignored -- delete it or regenerate anytime by rerunning this).
    results_path = run_batch(SCENARIO, runs, name="template_kill_radius")

    # load_results reads that CSV into a pandas DataFrame, one row per
    # run. Columns that always exist (src/batch/BatchMain.cpp's
    # kScalarColumnsTail is the authoritative list):
    #
    #   launched                     did the launcher ever fire a missile
    #   launch_time_s                sim time of launch, if it happened
    #   first_detection_time_s       when the radar first saw the target
    #   firing_solution_time_s       when the launcher's lock was confirmed for firing
    #   seeker_active_time_s         when the missile's own seeker acquired
    #   seeker_lost_time_s           when/if the seeker lost track
    #   datalink_error_at_handoff_m  launcher track error at handoff
    #   resolved                     did the engagement reach an outcome
    #   hit                          1 if the missile scored a kill, else 0
    #   armed                        did the warhead reach its arming range
    #   resolution_time_s            sim time the engagement resolved at
    #   miss_distance_m              closest approach, if not a hit
    #   impact_region                where on the airframe (if hit)
    #   error                        exception text, if the run raised in C++
    #
    # Plus one column per override key you swept, named after its last
    # path segment ("kill_radius_m" here).
    df = load_results(results_path)

    # Bail loudly on a bad sweep rather than silently computing Pk over
    # nothing -- a non-empty "error" column means a run's scenario failed
    # to load (e.g. an override value outside what the loader accepts).
    failed = df[df.error.notna()]
    if len(failed):
        raise SystemExit(f"{len(failed)} run(s) failed, e.g.: {failed.error.iloc[0]}")

    # Most outcome columns only mean something for a run that actually
    # launched a missile -- filter to those before computing Pk.
    df = df[df.launched == 1]

    # The actual analysis: kill probability at each level. This is the
    # part every experiment is really about -- everything above is just
    # getting a DataFrame to do this with. Group by whatever you swept;
    # print or plot whatever answers your question.
    print(f"\n{'kill_radius_m':>13} {'runs':>5} {'Pk':>7}")
    for level in LEVELS:
        cell = df[df.kill_radius_m == level]
        pk = cell.hit.mean() if len(cell) else float("nan")
        print(f"{level:>13.1f} {len(cell):>5} {pk:>7.3f}")

    # For anything the wide table doesn't carry -- WHY a run ended a
    # particular way, not just that it did -- load_events() reads the
    # long-format per-event log written alongside the results CSV (one
    # row per EventLog entry: run_id, sim time, event type, entity, a
    # free-text "data" field). A number can be pulled out of that "data"
    # field with a regex:
    #
    #   from _framework import load_events
    #   events = load_events(results_path)
    #   acquisitions = events[events.type == "SEEKER_ACTIVE"]
    #   link_err_ft = acquisitions.data.str.extract(r"datalink_error_ft=([-\d.]+)")[0].astype(float)


if __name__ == "__main__":
    main()
