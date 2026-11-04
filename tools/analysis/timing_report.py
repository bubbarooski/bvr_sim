"""Time the headless batch runner and write docs/timing_report.md.

One fixed engagement, tests/fixtures/scenarios/scenario_timing_5min.json,
is flown three ways: a single run five times back to back, a 1000-run
Monte Carlo at one thread, and the same sweep on every logical core. The
two sweeps are then checked to give identical per-seed results. The
report states each figure with the machine it was measured on, then the PDF
is rebuilt.

    cmake --build build --config Release --target intercept_batch
    python tools/analysis/timing_report.py                # ~20 min, mostly the 1-thread sweep
    python tools/analysis/timing_report.py --runs 50 --no-pdf   # quick check of the script

The scenario is a frozen benchmark input, not a teaching scenario: keep it
unchanged so figures from different dates stay like for like.

Requires a Release build of intercept_batch (see experiments/_framework.py,
whose run_batch this uses) and, for the PDF, what build_reference_pdf.py
requires.
"""

import argparse
import csv
import os
import statistics
import subprocess
import sys
import time
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "experiments"))

from _framework import run_batch  # noqa: E402

SCENARIO = "tests/fixtures/scenarios/scenario_timing_5min.json"
REPORT = REPO_ROOT / "docs" / "timing_report.md"
PDF = REPO_ROOT / "docs" / "timing_report.pdf"
PDF_BUILDER = REPO_ROOT / "tools" / "analysis" / "build_reference_pdf.py"

NOMINAL_SEED = 12345  # the scenario file's own seed
SINGLE_REPEATS = 5
MAX_SIM_TIME_S = 400.0

COMPARE_SKIP = {"wall_ms"}  # the only results column allowed to differ between thread counts


def machine():
    """(cpu name, physical cores or None, logical threads)."""
    name, cores = None, None
    if sys.platform == "win32":
        out = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "$p = Get-CimInstance Win32_Processor | Select-Object -First 1; "
             "$p.Name.Trim(); $p.NumberOfCores"],
            capture_output=True, text=True).stdout.split("\n")
        lines = [line.strip() for line in out if line.strip()]
        if len(lines) >= 2:
            name, cores = lines[0], int(lines[1])
    return name or "unknown CPU", cores, os.cpu_count()


def timed_batch(name, seeds, jobs):
    """Run the scenario over `seeds`; returns (wall seconds, results CSV path)."""
    runs = [{"run_id": i, "seed": s, "overrides": {}} for i, s in enumerate(seeds)]
    start = time.perf_counter()
    out = run_batch(SCENARIO, runs, jobs=jobs, max_sim_time_s=MAX_SIM_TIME_S, name=name)
    return time.perf_counter() - start, out


def read_csv(path):
    """Rows sorted by run_id (the runner already writes them in that order).
    The sort is stable, keeping each run's events in time order."""
    with open(path, newline="") as f:
        return sorted(csv.DictReader(f), key=lambda row: int(row["run_id"]))


def events_path(results):
    return results.with_name(results.stem + "_events.csv")


def identical(a, b):
    """Per-seed results (every column but wall_ms) and every event row match."""
    rows_a, rows_b = read_csv(a), read_csv(b)
    if len(rows_a) != len(rows_b):
        return False
    for ra, rb in zip(rows_a, rows_b):
        if any(ra[k] != rb.get(k) for k in ra if k not in COMPARE_SKIP):
            return False
    return read_csv(events_path(a)) == read_csv(events_path(b))


def outcomes(results):
    rows = read_csv(results)
    errors = [r for r in rows if r["error"]]
    launched = sum(r["launched"] == "1" for r in rows)
    hits = sum(r["hit"] == "1" for r in rows)
    res = [float(r["resolution_time_s"]) for r in rows if r["resolved"] == "1"]
    miss = [float(r["miss_distance_m"]) for r in rows if r["miss_distance_m"]]
    return {
        "runs": len(rows), "errors": len(errors), "launched": launched, "hits": hits,
        "timed_out": sum(r["timed_out"] == "1" for r in rows),
        "res_mean": statistics.mean(res), "res_sd": statistics.pstdev(res),
        "res_min": min(res), "res_max": max(res),
        "direct": sum(d == 0.0 for d in miss),
        "miss_median": statistics.median(miss) if miss else 0.0,
        "miss_max": max(miss) if miss else 0.0,
        "self_destructs": sum(r["impact_region"] == "self_destruct" for r in rows),
    }


def fmt_duration(s):
    if s < 60:
        return f"{s:.1f} s"
    return f"{s:.1f} s ({int(s // 60)} min {int(round(s % 60))} s)"


def build_report(m, single, timeline, resolution_s, sweep, n_runs, same, out):
    cpu, cores, threads = m
    single_mean = statistics.mean(single)
    rtf = resolution_s / single_mean
    s1, sn = sweep[1], sweep[threads]
    speedup = s1 / sn
    core_text = f"{cores} physical cores / {threads} logical threads" if cores else f"{threads} logical threads"
    scale_note = ""
    if cores:
        scale_note = (f" The ceiling is set by the {cores} physical cores: the logical threads share "
                      "them, and JSBSim is compute- and memory-bound, so hyper-threading adds little.")
    pk = out["hits"] / out["runs"]
    miss_text = ("0.00 m on every run" if out["miss_max"] == 0.0
                 else f"0.00 m on {out['direct']} runs; median {out['miss_median']:.2f} m, "
                      f"largest {out['miss_max']:.0f} m")

    lines = []
    w = lines.append
    w("# Timing report")
    w("")
    w("A headless timing exercise, not a physics study. One fixed air-to-air "
      "engagement is flown once by itself, then "
      f"{n_runs} times as a Monte Carlo sweep, and the measurement is wall-clock seconds.")
    w("")
    w("All runs use the headless `intercept_batch` driver (no window, no "
      "rendering), back to back. Simulated time advances at a fixed 120 Hz "
      "regardless of wall clock.")
    w("")
    w(f"**Machine:** {cpu}, {core_text}, Windows, Release build with "
      "link-time optimization (LTCG).  ")
    w(f"**Measured:** {date.today().isoformat()}. Generated by "
      "`tools/analysis/timing_report.py`.")
    w("")
    w("## The scenario")
    w("")
    w(f"`{SCENARIO}`: a head-on intercept whose start is far enough out that "
      "the whole engagement runs about five minutes of simulated time.")
    w("")
    w("| Element | Value |")
    w("|---|---|")
    w("| Geometry | Head-on, co-altitude, closing along the same line |")
    w("| Aircraft | Two JSBSim F-16s (full 6-DOF flight models) |")
    w("| Altitude / speed | 25,000 ft, 400 KTAS each (about 800 kt closure) |")
    w("| Initial separation | 525,000 ft (about 86 nmi) |")
    w("| Pursuer sensor | Airborne pulse-Doppler radar |")
    w("| Weapon | AIM-120 type, 12 m kill radius |")
    w("")
    w(f"**Timeline** (seed {NOMINAL_SEED}):")
    w("")
    w("| Event | Sim time |")
    w("|---|---|")
    for t, kind in timeline:
        w(f"| {kind} | {t:.1f} s |")
    w("")
    w(f"The run ends the instant the engagement resolves, at **{resolution_s:.1f} s**. "
      "The real-time factor below divides by this engagement length.")
    w("")
    w("## Test 1: single headless run")
    w("")
    w("One run timed end to end (process start, JSON load, JSBSim load and trim "
      "of both aircraft, then the fixed-step loop to the intercept), five times "
      "back to back.")
    w("")
    w("| Run | Wall time |")
    w("|---|---|")
    for i, new_s in enumerate(single, 1):
        w(f"| {i} | {new_s:.2f} s |")
    w(f"| **Mean** | **{single_mean:.2f} s** |")
    w(f"| Real-time factor | {rtf:.0f}x |")
    w("")
    w("The cost is dominated by JSBSim: two full flight models stepped every tick.")
    w("")
    w(f"## Test 2: {n_runs}-run Monte Carlo")
    w("")
    w("The same scenario, one run per seed (seeds 1 to "
      f"{n_runs}). Only the seed changes between runs; it reshuffles the "
      "radar's scan-to-scan RCS fluctuation, the datalink and seeker noise, "
      "and the warhead's Pk draw.")
    w("")
    w("| Configuration | Wall time | Runs/s |")
    w("|---|---|---|")
    w(f"| 1 thread | {fmt_duration(s1)} | {n_runs / s1:.1f} |")
    w(f"| {threads} threads | {fmt_duration(sn)} | {n_runs / sn:.1f} |")
    w(f"| Speedup | {speedup:.1f}x | |")
    w("")
    w("The win is doing many runs at once, not each run getting faster: per-run "
      "wall time rises under load as the workers contend for memory bandwidth."
      + scale_note)
    w("")
    w("## Test 3: same results at any thread count")
    w("")
    if same:
        w("**Identical.** Every run's results row (every column but its own wall "
          "time) and every event row match between the 1-thread and the "
          f"{threads}-thread sweep. Each run owns a random stream seeded from its "
          "own seed, so thread count changes only speed, never results.")
    else:
        w("**NOT identical.** The 1-thread and the "
          f"{threads}-thread sweeps disagree on at least one run. This is a bug: "
          "thread count must never change results.")
    w("")
    w(f"## What the {n_runs} runs found")
    w("")
    w("The outcomes are a byproduct, but they confirm the sweep did varied work.")
    w("")
    w("| Metric | Value |")
    w("|---|---|")
    w(f"| Runs launched | {out['launched']} / {out['runs']} |")
    w(f"| Hits (Pk) | {out['hits']} ({pk:.3f}) |")
    w(f"| Miss distance | {miss_text} |")
    w(f"| Resolution time, mean (sd) | {out['res_mean']:.1f} s ({out['res_sd']:.1f} s) |")
    w(f"| Resolution time, range | {out['res_min']:.1f} to {out['res_max']:.1f} s |")
    misses = out["runs"] - out["hits"]
    if misses:
        w(f"| Misses: self-destruct / other | {out['self_destructs']} / {misses - out['self_destructs']} |")
    if out["errors"] or out["timed_out"]:
        w(f"| Errors / timed out | {out['errors']} / {out['timed_out']} |")
    w("")
    w("Pk is L(d)/L(0), 1 at 0 m ([REFERENCE 12.4](REFERENCE.md#124-probabilistic-lethality)), "
      "so a direct hit always kills.")
    w("")
    w("## Summary")
    w("")
    w("| Workload | Wall time |")
    w("|---|---|")
    w(f"| 1 run | {single_mean:.2f} s |")
    w(f"| {n_runs} runs, 1 thread | {fmt_duration(s1)} |")
    w(f"| {n_runs} runs, {threads} threads | {fmt_duration(sn)} |")
    w("")
    w("*Method: `intercept_batch --spec <spec.json> --out <results.csv>`, "
      f"`--jobs 1` and `--jobs {threads}`, via "
      "`python tools/analysis/timing_report.py`. Same Release binary throughout.*")
    w("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--runs", type=int, default=1000, help="runs per sweep (the published report uses 1000)")
    parser.add_argument("--no-pdf", action="store_true", help="skip rebuilding docs/timing_report.pdf")
    args = parser.parse_args()

    m = machine()
    threads = m[2]
    print(f"timing_report: {m[0]}, {threads} threads")

    single, single_out = [], None
    for i in range(SINGLE_REPEATS):
        wall, single_out = timed_batch(f"timing_single{i}", [NOMINAL_SEED], 1)
        single.append(wall)
    row = read_csv(single_out)[0]
    resolution_s = float(row["resolution_time_s"])
    timeline = [(float(e["sim_time_s"]), e["type"]) for e in read_csv(events_path(single_out))]

    seeds = list(range(1, args.runs + 1))
    sweep, sweep_out = {}, {}
    for jobs in (1, threads):
        sweep[jobs], sweep_out[jobs] = timed_batch(f"timing_sweep_j{jobs}", seeds, jobs)
        print(f"timing_report: {args.runs} runs at {jobs} thread(s): {sweep[jobs]:.1f} s")

    same = identical(sweep_out[1], sweep_out[threads])
    out = outcomes(sweep_out[threads])
    REPORT.write_text(build_report(m, single, timeline, resolution_s, sweep, args.runs, same, out),
                      encoding="utf-8", newline="\n")
    print(f"timing_report: wrote {REPORT.relative_to(REPO_ROOT)} (thread-count check: "
          f"{'identical' if same else 'DIFFERENT'})")

    if not args.no_pdf:
        subprocess.run([sys.executable, str(PDF_BUILDER), "--input", str(REPORT), "--output", str(PDF)],
                       cwd=REPO_ROOT, check=True)
    return 0 if same else 1


if __name__ == "__main__":
    sys.exit(main())
