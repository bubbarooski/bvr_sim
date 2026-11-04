#!/usr/bin/env python3
"""Remove generated files the repo no longer needs.

Lists every path it would delete (the dry run is exactly what --yes removes)
and deletes nothing unless --yes is given. Nothing git tracks is ever deleted:
a rule that reaches a tracked file stops the run.

    python tools/clean.py                  # dry run: what is stale
    python tools/clean.py --yes            # delete it
    python tools/clean.py --results base new --yes
                                           # also prune experiments/results
                                           # to these batch_compare labels
    python tools/clean.py --build --yes    # delete the whole build/ folder
                                           # (next build starts from scratch)

What counts as stale:
  - Python caches (__pycache__/) anywhere outside deps/.
  - Litter in the repo root from running things there: imgui.ini, CMakeFiles/,
    Testing/, *.log.
  - In build/: anything belonging to a target that CMakeLists.txt no longer
    defines (CMake never removes these), and any file a person or script left
    at the top level. The allowed top-level entries are the ones a fresh
    Visual Studio or single-config configure creates.
  - In each build/<config>/: executables and libraries of removed targets,
    and files in the data/ copy that are gone from data/ (the build copies
    data/ but never deletes from the copy).
  - With --results: every file under experiments/results/ not belonging to
    one of the named batch_compare labels.

Also reports untracked files git does not ignore -- probably meant to be
committed or deleted -- without touching them. Run from anywhere.
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD = REPO_ROOT / "build"
RESULTS = REPO_ROOT / "experiments" / "results"
CONFIGS = ["Debug", "Release", "MinSizeRel", "RelWithDebInfo"]

# Targets the Visual Studio generator adds to every project.
GENERATOR_TARGETS = {"ALL_BUILD", "ZERO_CHECK", "INSTALL", "PACKAGE", "RUN_TESTS"}

# Top-level build/ entries a configure + build + ctest creates, other than
# per-target files. lib/ is GoogleTest's own output folder; x64/ is MSBuild's
# scratch space for the generator targets; coverage/ is
# tools/analysis/coverage.py's report; asan/ is the AddressSanitizer tree
# (README - Tests).
BUILD_TOP_LEVEL = {
    "CMakeCache.txt", "CMakeFiles", "Testing", "deps", "lib", "x64", "tests", "tools", "coverage", "asan",
    "cmake_install.cmake", "CTestTestfile.cmake", "CPackConfig.cmake",
    "CPackSourceConfig.cmake", "DartConfiguration.tcl", "Makefile", "build.ninja",
    ".ninja_deps", ".ninja_log", "compile_commands.json", *CONFIGS,
}

# What the build puts in build/<config>/ besides target outputs.
CONFIG_KEEP = {"data", "fonts", "scenarios", "imgui.ini", "tests", "tools", "lib"}
BINARY_SUFFIXES = {".exe", ".pdb", ".lib", ".ilk", ".exp", ".a", ""}


def current_targets():
    text = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    return set(re.findall(r"^\s*add_(?:executable|library)\((\w+)", text, re.MULTILINE))


def target_of(name):
    """The target a per-target build/ entry belongs to, or None."""
    m = re.match(r"^(\w+?)(?:\.dir|\.vcxproj(?:\.filters)?|_[0-9a-f]{8}_.*\.cmake)$", name)
    return m.group(1) if m else None


def stale_build(targets):
    if not BUILD.is_dir():
        return []
    known = targets | GENERATOR_TARGETS
    stale = []
    for entry in BUILD.iterdir():
        name = entry.name
        if name in BUILD_TOP_LEVEL or name.endswith((".sln", ".slnx")):
            continue
        owner = target_of(name)
        if owner is None or owner not in known:
            stale.append(entry)
    for config in [BUILD / c for c in CONFIGS] + [BUILD]:
        stale += stale_outputs(config, targets)
    return stale


def stale_outputs(config_dir, targets):
    """Removed targets' binaries in an output folder, and the data/ copy's
    files that data/ no longer has."""
    stale = []
    if not config_dir.is_dir():
        return stale
    # build/lib is GoogleTest's own output folder, not a target of ours;
    # build/ itself is only an output folder for single-config generators.
    folders = [config_dir / sub for sub in ("tests", "tools") + (("lib",) if config_dir != BUILD else ())]
    if config_dir != BUILD:
        folders.append(config_dir)
    for folder in folders:
        if not folder.is_dir():
            continue
        for f in folder.iterdir():
            if f.name in CONFIG_KEEP and folder == config_dir:
                continue
            if f.is_file() and f.suffix in BINARY_SUFFIXES and f.stem not in targets:
                stale.append(f)
            elif folder == config_dir and f.is_dir():
                stale.append(f)
    copy = config_dir / "data"
    if copy.is_dir() and config_dir != BUILD:
        for f in copy.rglob("*"):
            if f.is_file() and not (REPO_ROOT / "data" / f.relative_to(copy)).exists():
                stale.append(f)
    return stale


def root_litter():
    names = ["imgui.ini", "CMakeFiles", "Testing"]
    found = [REPO_ROOT / n for n in names if (REPO_ROOT / n).exists()]
    return found + sorted(REPO_ROOT.glob("*.log"))


def python_caches():
    return [p for p in REPO_ROOT.rglob("__pycache__")
            if "deps" not in p.relative_to(REPO_ROOT).parts and "build" not in p.relative_to(REPO_ROOT).parts]


def stale_results(keep_labels):
    compare = RESULTS / "compare"
    keep = set()
    for label in keep_labels:
        manifest = compare / f"{label}.json"
        if not manifest.is_file():
            sys.exit(f"no batch_compare label {label!r} ({manifest.relative_to(REPO_ROOT)} missing)")
        keep.add(manifest.resolve())
        for result in json.loads(manifest.read_text())["results"].values():
            csv = (REPO_ROOT / result).resolve()
            # A run's CSV, its _events.csv, _spec.json and any _traj/ folder.
            keep.update(p.resolve() for p in csv.parent.glob(csv.stem + "*"))
    stale = []
    for p in RESULTS.rglob("*"):
        if p.is_file() and p.resolve() not in keep and not any(a in keep for a in p.resolve().parents):
            stale.append(p)
    return stale


def untracked():
    out = subprocess.run(["git", "status", "--porcelain", "--untracked-files=normal"], cwd=REPO_ROOT,
                         capture_output=True, text=True).stdout
    return [line[3:] for line in out.splitlines() if line.startswith("??")]


def size_of(path):
    if path.is_file():
        return path.stat().st_size
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def remove(path):
    if path.is_dir():
        shutil.rmtree(path)
    elif path.exists():
        path.unlink()


def prune_empty_dirs(root, deleted):
    """Removes the folders that `deleted` files left empty, up to `root`;
    a folder that was already empty before this run is not ours to remove."""
    candidates = {parent for path in deleted for parent in path.parents if root in parent.parents}
    for d in sorted(candidates, key=lambda d: -len(d.parts)):
        if d.is_dir() and not any(d.iterdir()):
            d.rmdir()


def tracked_among(paths):
    """The paths (files, or folders holding a file) git tracks. Nothing
    tracked is ever deleted: a rule that reaches one is a bug in the rule."""
    out = subprocess.run(["git", "ls-files", "-z"], cwd=REPO_ROOT, capture_output=True, text=True).stdout
    tracked = {(REPO_ROOT / name).resolve() for name in out.split("\0") if name}
    hits = []
    for path in paths:
        resolved = path.resolve()
        if resolved in tracked or any(resolved in t.parents for t in tracked):
            hits.append(path)
    return hits


def report(groups, delete):
    """Prints every path that will be (or is being) deleted, one per line,
    and returns the total size in bytes. The dry run lists exactly what
    --yes deletes, nothing abbreviated."""
    total = 0
    for title, paths in groups:
        paths = sorted(set(paths))
        if not paths:
            continue
        group_size = sum(size_of(p) for p in paths)
        total += group_size
        print(f"{title}: {len(paths)} item(s), {group_size / 1e6:.1f} MB")
        for p in paths:
            print(f"  {p.relative_to(REPO_ROOT).as_posix()}")
    return total


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--yes", action="store_true", help="delete (default: list only)")
    parser.add_argument("--results", nargs="+", metavar="LABEL",
                        help="prune experiments/results to these batch_compare labels")
    parser.add_argument("--build", action="store_true", help="delete the whole build/ folder")
    args = parser.parse_args()

    groups = [("Python caches", python_caches()), ("Repo-root litter", root_litter())]
    if args.build:
        groups.append(("Build folder", [BUILD] if BUILD.exists() else []))
    else:
        groups.append(("Stale build files", stale_build(current_targets())))
    if args.results:
        groups.append(("Old experiment results", stale_results(args.results)))

    total = report(groups, args.yes)
    everything = sorted({p for _, paths in groups for p in paths})
    refused = tracked_among(everything)
    if refused:
        sys.exit("refusing to delete tracked files (a cleaning rule is wrong): " +
                 ", ".join(p.relative_to(REPO_ROOT).as_posix() for p in refused))
    if args.yes:
        for p in everything:
            remove(p)
        if args.results:
            prune_empty_dirs(RESULTS, everything)

    leftovers = untracked()
    if leftovers:
        print("Untracked, not ignored (commit or delete by hand):")
        for f in leftovers:
            print(f"  {f}")

    if total == 0:
        print("Nothing stale.")
    elif args.yes:
        print(f"Deleted {total / 1e6:.1f} MB.")
    else:
        print(f"{total / 1e6:.1f} MB would be deleted. Rerun with --yes to delete.")


if __name__ == "__main__":
    main()
