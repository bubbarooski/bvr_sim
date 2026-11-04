"""Reusable Monte Carlo plumbing for intercept_sim experiments.

An experiment script builds a list of runs, calls run_batch, and analyzes
the DataFrame that load_results hands back. Everything specific to a
question -- which parameter to sweep, how many seeds, what to plot --
belongs in the experiment script, not here.

    from _framework import run_batch, load_results, dot_to_pointer

    runs = [
        {"run_id": i, "seed": 1000 + i,
         "overrides": {dot_to_pointer("missile_doctrine.nav_gain"): gain}}
        for i, gain in enumerate(...)
    ]
    df = load_results(run_batch("scenarios/1_core_01_first_shot.json", runs))
    print(df.groupby("nav_gain")["hit"].mean())

Results are CSV: a wide one-row-per-run table, plus a long-format event
table alongside it that load_events reads and that joins back on run_id.
Both open directly in a spreadsheet.

Requires pandas, and a Release build of intercept_batch.
"""

import json
import subprocess
import sys
import time
from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
# Multi-config generators (Visual Studio, Xcode) nest under the config name;
# single-config ones (Makefiles, Ninja) put the binary straight in build/.
_EXE_NAME = "intercept_batch.exe" if sys.platform == "win32" else "intercept_batch"
_EXE_CANDIDATES = [REPO_ROOT / "build" / "Release" / _EXE_NAME, REPO_ROOT / "build" / _EXE_NAME]
BATCH_EXE = next((p for p in _EXE_CANDIDATES if p.exists()), _EXE_CANDIDATES[0])
RESULTS_DIR = Path(__file__).resolve().parent / "results"
"""Where specs, results CSVs, and recorded trajectories land. Gitignored."""

def dot_to_pointer(path):
    """Convert a readable dotted path to the JSON Pointer the runner wants.

    "entities[1].components.signature.rcs_scale"
        -> "/entities/1/components/signature/rcs_scale"

    A key that itself contains "~" or "/" is escaped the way RFC 6901
    requires. An empty segment ("a..b", a trailing dot) is an error rather
    than a pointer to a key named "".
    """
    segments = path.replace("[", ".").replace("]", "").split(".")
    if path.startswith("[") and segments[0] == "":
        segments = segments[1:]  # a leading index: "[0].a"
    if not segments or any(segment == "" for segment in segments):
        raise ValueError(f"not a dotted path: {path!r}")
    return "/" + "/".join(s.replace("~", "~0").replace("/", "~1") for s in segments)


def grid_runs(cells, seeds_per_cell, seed_of):
    """Build a run list: `seeds_per_cell` runs for each cell of a sweep.

    `cells` is a list of (label, overrides) pairs -- one per level of
    whatever is being swept, overrides keyed by JSON Pointer. `seed_of(cell
    index, seed index)` returns each run's seed; keep it stable, since it
    is what makes a sweep reproducible. Seeds must be unique across the
    batch unless deliberately shared (common random numbers across levels).

    Returns (runs, label_of_run): the list run_batch takes, and a dict
    from run_id to its cell's label for tagging the results afterwards.
    """
    runs, label_of_run = [], {}
    for c, (label, overrides) in enumerate(cells):
        for i in range(seeds_per_cell):
            run_id = len(runs)
            label_of_run[run_id] = label
            runs.append({"run_id": run_id, "seed": seed_of(c, i), "overrides": dict(overrides)})
    return runs, label_of_run


def run_batch(scenario, runs, jobs=None, record=False, max_sim_time_s=400.0, name=None):
    """Execute `runs` against `scenario` and return the results CSV path.

    `runs` is a list of {"run_id", "seed", "overrides"} dicts -- overrides
    keyed by JSON Pointer (see dot_to_pointer). `jobs` defaults to the
    runner's own choice of hardware concurrency.
    """
    if not BATCH_EXE.exists():
        raise SystemExit(
            f"intercept_batch not found at {BATCH_EXE}\n"
            "Build it first:\n"
            '    cmake --build build --config Release --target intercept_batch\n'
            "Release specifically -- a Debug build is ~10x slower and would turn a\n"
            "few minutes of sweep into an hour."
        )

    RESULTS_DIR.mkdir(exist_ok=True)
    stem = name or Path(sys.argv[0]).stem or "batch"
    stamp = time.strftime("%Y%m%d_%H%M%S")
    spec_path = RESULTS_DIR / f"{stem}_{stamp}_spec.json"
    out_path = RESULTS_DIR / f"{stem}_{stamp}.csv"

    spec_path.write_text(json.dumps(
        {"scenario": scenario, "max_sim_time_s": max_sim_time_s, "runs": runs}, indent=2))

    cmd = [str(BATCH_EXE), "--spec", str(spec_path), "--out", str(out_path)]
    if jobs:
        cmd += ["--jobs", str(jobs)]
    if record:
        cmd += ["--record", str(RESULTS_DIR / f"{stem}_{stamp}_traj")]

    # cwd=REPO_ROOT because the runner resolves scenario and JSBSim data
    # paths relative to the working directory.
    status = subprocess.run(cmd, cwd=REPO_ROOT).returncode
    # Exit 1 with a results file means some runs failed: each has its own
    # row with the "error" column set, which the caller reads (a sweep that
    # lost one run in a hundred should still be analysable). Anything else --
    # a usage error, a crash, or a runner that wrote nothing because the
    # spec or an override was bad -- is an error here.
    if status not in (0, 1) or not out_path.exists():
        raise RuntimeError(
            f"intercept_batch exited with status {status} and wrote no results to {out_path.name}; "
            "its output is above"
        )
    if status == 1:
        print("intercept_batch: some runs failed; see the 'error' column of the results", file=sys.stderr)
    return out_path


def load_results(path):
    """Read a results CSV into a DataFrame, sorted by run_id.

    Each override is already its own column, named after the last segment
    of its pointer, so a sweep groups by the parameter name directly:
    df.groupby("nav_sigma_m"). Booleans arrive as 1/0, so df.hit.mean() is
    the empirical Pk. Runs that raised in C++ have a populated "error"
    column and empty outcome columns.
    """
    return pd.read_csv(path).sort_values("run_id").reset_index(drop=True)


def load_events(path):
    """Read the event table written alongside a results CSV.

    Long format -- one row per event -- so it joins back onto the results
    frame on run_id rather than nesting inside it. `path` may be either
    the results CSV or the event CSV itself.
    """
    path = Path(path)
    if not path.name.endswith("_events.csv"):
        path = path.with_name(path.stem + "_events.csv")
    return pd.read_csv(path)
