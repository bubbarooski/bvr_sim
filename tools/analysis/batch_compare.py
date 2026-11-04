"""Compare missile-scenario outcomes before and after a model change.

Every physics change shifts engagement outcomes, and the question each
time is whether a shift is the physics or just the RNG stream moving (a
change in draw order re-rolls every later Pk draw). This flies every
missile scenario over a fixed seed range, keeps the results under a
label, and compares two labels run by run.

    python tools/analysis/batch_compare.py run base          # at the commit before the change
    (make the change, rebuild intercept_batch Release)
    python tools/analysis/batch_compare.py run new
    python tools/analysis/batch_compare.py diff base new
    python tools/analysis/batch_compare.py run stt --set /missile_doctrine/launch_mode=STT

`run` flies every teaching scenario (the numbered files directly under
scenarios/) and every file under scenarios/custom/ whose missile_doctrine
names a missile type (custom/_template.json included: it is a complete
head-on engagement), seeds 1-20 by default, with any --set knob overrides
applied to every run (a sensitivity), and writes a manifest
experiments/results/compare/<label>.json naming each scenario's results
CSV and the commit it was run at. `diff` reports, per scenario:

  hits          hit count, A -> B
  flips         runs whose hit/miss changed, split into Pk rolls (the
                missile reached the kill radius in both runs with pk
                within 0.05, so the flip is a different draw against the
                same probability -- the RNG, not the guidance) and the rest
  resolution    runs whose resolution time moved by more than 0.05 s
  miss median   median miss distance, m

then lists every non-Pk-roll flip. `diff --identical` instead checks the
two labels are bit-identical (every results cell except wall_ms, compared as
the text the runner wrote, and every event row in its original order), the
gate for a behaviour-preserving change. It fails if the labels were run with
different seeds or overrides, if either has a scenario the other lacks, or
if they share none.

Requires pandas and a Release build of intercept_batch (see
experiments/_framework.py, whose run_batch this uses).
"""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "experiments"))

import pandas as pd  # noqa: E402

from _framework import RESULTS_DIR, load_results, run_batch  # noqa: E402

COMPARE_DIR = RESULTS_DIR / "compare"
SCENARIO_DIRS = [REPO_ROOT / "scenarios", REPO_ROOT / "scenarios" / "custom"]
RESOLUTION_TOL_S = 0.05


def missile_scenarios():
    """Every scenario file that launches a typed missile, repo-relative."""
    found = []
    for directory in SCENARIO_DIRS:
        for path in sorted(directory.glob("*.json")):
            doctrine = json.loads(path.read_text(encoding="utf-8-sig")).get("missile_doctrine") or {}
            if "missile" in doctrine:
                found.append(path.relative_to(REPO_ROOT).as_posix())
    return found


def parse_seeds(text):
    """'1-20', '1,4,9' or '1-3,7' -> a list of non-negative ints, in order.

    A range is inclusive. A malformed or empty piece, a reversed range or a
    duplicate seed is an error: it would otherwise be a silently shorter or
    double-counted sweep."""
    seeds = []
    for piece in text.split(","):
        lo, dash, hi = piece.strip().partition("-")
        try:
            first, last = int(lo), int(hi) if dash else int(lo)
        except ValueError:
            raise SystemExit(f"--seeds: '{piece}' is not a seed or a lo-hi range") from None
        if first < 0 or last < first:
            raise SystemExit(f"--seeds: '{piece}' must be non-negative and not reversed")
        seeds.extend(range(first, last + 1))
    if len(set(seeds)) != len(seeds):
        raise SystemExit(f"--seeds: '{text}' repeats a seed")
    return seeds


def label_name(text):
    """argparse type for a label: it becomes part of file names."""
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", text):
        raise argparse.ArgumentTypeError(f"label '{text}' may use letters, digits, '.', '_' and '-' only")
    return text


def git_head():
    def git(*args):
        return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True).stdout.strip()
    return git("rev-parse", "--short", "HEAD") + (" (dirty)" if git("status", "--porcelain") else "")


def parse_overrides(pairs):
    """['/a/b=STT', '/c=0.5'] -> {'/a/b': 'STT', '/c': 0.5}; a value that is
    not JSON is taken as a string."""
    overrides = {}
    for pair in pairs or []:
        pointer, sep, text = pair.partition("=")
        if not sep or not pointer.startswith("/"):
            raise SystemExit(f"--set needs /json/pointer=value, got '{pair}'")
        try:
            overrides[pointer] = json.loads(text)
        except json.JSONDecodeError:
            overrides[pointer] = text
    return overrides


def cmd_run(args):
    seeds = parse_seeds(args.seeds)
    scenarios = args.scenarios or missile_scenarios()
    overrides = parse_overrides(args.set)
    runs = [{"run_id": i, "seed": seed, "overrides": dict(overrides)} for i, seed in enumerate(seeds)]
    manifest = {"label": args.label, "commit": git_head(), "seeds": seeds, "overrides": overrides, "results": {}}
    for scenario in scenarios:
        print(f"[{args.label}] {scenario} x {len(seeds)} seeds", flush=True)
        name = f"compare_{args.label}_{Path(scenario).stem}"
        out = run_batch(scenario, runs, jobs=args.jobs, max_sim_time_s=args.max_sim_time_s, name=name)
        manifest["results"][scenario] = str(out.relative_to(REPO_ROOT).as_posix())
    COMPARE_DIR.mkdir(parents=True, exist_ok=True)
    path = COMPARE_DIR / f"{args.label}.json"
    path.write_text(json.dumps(manifest, indent=2))
    print(f"wrote {path.relative_to(REPO_ROOT).as_posix()} ({manifest['commit']})")


def load_manifest(label):
    path = COMPARE_DIR / f"{label}.json"
    if not path.exists():
        raise SystemExit(f"no run labelled '{label}' ({path}); run `batch_compare.py run {label}` first")
    return json.loads(path.read_text())


def paired(a, b):
    """The two results frames joined run by run (columns suffixed _a/_b).

    Both labels fly the same run list, so a run is its run_id and seed
    together; joining on seed alone would cross-multiply any shared seeds."""
    return a.merge(b, on=["run_id", "seed"], suffixes=("_a", "_b"))


def resolution_moved(p):
    """The paired runs whose resolution time moved by more than the
    tolerance, including a run that resolved in one label and timed out
    (no resolution time) in the other."""
    unresolved_a, unresolved_b = p.resolution_time_s_a.isna(), p.resolution_time_s_b.isna()
    return p[(unresolved_a != unresolved_b) | ((p.resolution_time_s_a - p.resolution_time_s_b).abs() > RESOLUTION_TOL_S)]


# Two Pk values this close count as "the same probability": 0.05 is about
# 1 m of miss distance on the lethality sigmoid's steep side (5 m falloff).
PK_ROLL_TOLERANCE = 0.05


def is_pk_roll(row):
    """A flip drawn against the same probability both times -- not one
    whose Pk itself moved (a hit at 0.92 becoming a miss at 0.30 is the
    guidance, not the RNG). Both runs must have resolved with a non-zero Pk."""
    return row.pk_a > 0 and row.pk_b > 0 and abs(row.pk_a - row.pk_b) <= PK_ROLL_TOLERANCE


def cmd_diff(args):
    ma, mb = load_manifest(args.a), load_manifest(args.b)
    print(f"A = {args.a} @ {ma['commit']}    B = {args.b} @ {mb['commit']}")
    common = [s for s in ma["results"] if s in mb["results"]]
    skipped = sorted(set(ma["results"]) ^ set(mb["results"]))
    for s in skipped:
        print(f"  (only in one label, skipped: {s})")
    if args.identical:
        return diff_identical(ma, mb, common, skipped)
    if not common:
        print("the two labels share no scenario: nothing to compare")
        return 1

    rows, details = [], []
    for scenario in common:
        a = load_results(REPO_ROOT / ma["results"][scenario])
        b = load_results(REPO_ROOT / mb["results"][scenario])
        p = paired(a, b)
        flips = p[p.hit_a != p.hit_b]
        rolls = flips[flips.apply(is_pk_roll, axis=1)]
        moved = resolution_moved(p)
        errors = int(a.error.notna().sum() + b.error.notna().sum())
        rows.append({
            "scenario": Path(scenario).stem.removeprefix("scenario_"),
            "runs": len(p),
            "hits": f"{int(a.hit.sum())} -> {int(b.hit.sum())}",
            "flips": len(flips),
            "pk_rolls": len(rolls),
            "res_moved": len(moved),
            "miss_med_m": f"{a.miss_distance_m.median():.1f} -> {b.miss_distance_m.median():.1f}",
            "errors": errors,
        })
        for _, r in flips.drop(rolls.index).iterrows():
            details.append(f"  {rows[-1]['scenario']} seed {int(r.seed)}: "
                           f"{'hit' if r.hit_a else 'miss'} -> {'hit' if r.hit_b else 'miss'}, "
                           f"miss {r.miss_distance_m_a:.1f} -> {r.miss_distance_m_b:.1f} m, "
                           f"pk {r.pk_a:.2f} -> {r.pk_b:.2f}, "
                           f"t {r.resolution_time_s_a:.1f} -> {r.resolution_time_s_b:.1f} s")

    table = pd.DataFrame(rows)
    print(table.to_string(index=False))
    hits_a = sum(int(r["hits"].split(" -> ")[0]) for r in rows)
    hits_b = sum(int(r["hits"].split(" -> ")[1]) for r in rows)
    print(f"\ntotal hits {hits_a} -> {hits_b}; flips {table.flips.sum()} "
          f"({table.pk_rolls.sum()} Pk rolls); resolution moved in {table.res_moved.sum()} runs")
    if details:
        print("\nflips that are not Pk rolls:")
        print("\n".join(details))


def read_text_table(path):
    """A runner CSV as the exact text it holds, one string per cell (no float
    parsing, which would call a one-ulp change identical), in run order."""
    frame = pd.read_csv(path, dtype=str, keep_default_na=False)
    order = frame["run_id"].astype(int).argsort(kind="stable")  # stable: events keep their order within a run
    return frame.iloc[order].reset_index(drop=True)


def diff_identical(ma, mb, common, skipped=()):
    same = True
    # A comparison of different work proves nothing about the code.
    if not common:
        print("  the two labels share no scenario: nothing to compare")
        same = False
    if skipped:
        print(f"  {len(skipped)} scenario(s) in only one label")
        same = False
    for field in ("seeds", "overrides"):
        if ma.get(field) != mb.get(field):
            print(f"  the labels were run with different {field}: {ma.get(field)} vs {mb.get(field)}")
            same = False
    for scenario in common:
        a_path, b_path = REPO_ROOT / ma["results"][scenario], REPO_ROOT / mb["results"][scenario]
        a = read_text_table(a_path).drop(columns="wall_ms")
        b = read_text_table(b_path).drop(columns="wall_ms")
        # A column one side lacks (a removed or new output) is reported, and
        # the rest compared, so dropping an output can still be bit-identical.
        only_a, only_b = sorted(set(a.columns) - set(b.columns)), sorted(set(b.columns) - set(a.columns))
        if only_a or only_b:
            print(f"  columns  {scenario}: only in A {only_a}, only in B {only_b}")
            shared = [c for c in a.columns if c in b.columns]
            a, b = a[shared], b[shared]
        ea = read_text_table(a_path.with_name(a_path.stem + "_events.csv"))
        eb = read_text_table(b_path.with_name(b_path.stem + "_events.csv"))
        results_same = a.shape == b.shape and a.equals(b)
        events_same = ea.shape == eb.shape and ea.equals(eb)
        if not (results_same and events_same):
            same = False
            what = ", ".join(w for w, ok in (("results", results_same), ("events", events_same)) if not ok)
            print(f"  DIFFERS  {scenario}: {what}")
        else:
            print(f"  same     {scenario}")
    print("bit-identical" if same else "NOT bit-identical")
    return 0 if same else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="fly every missile scenario and save the results under a label")
    run.add_argument("label", type=label_name)
    run.add_argument("--seeds", default="1-20", help="'1-20' or '1,4,9' (default 1-20)")
    run.add_argument("--scenarios", nargs="*", help="repo-relative scenario paths (default: every missile scenario)")
    run.add_argument("--jobs", type=int, help="worker threads (default: the runner's choice)")
    run.add_argument("--max-sim-time-s", type=float, default=400.0)
    run.add_argument("--set", action="append", metavar="POINTER=VALUE",
                     help="a knob override for every run, e.g. /missile_doctrine/launch_mode=STT (repeatable)")
    diff = sub.add_parser("diff", help="compare two labels run by run")
    diff.add_argument("a", type=label_name)
    diff.add_argument("b", type=label_name)
    diff.add_argument("--identical", action="store_true", help="check bit-identical results and events instead")
    args = parser.parse_args()
    if args.command == "run":
        cmd_run(args)
    else:
        sys.exit(cmd_diff(args) or 0)


if __name__ == "__main__":
    main()
