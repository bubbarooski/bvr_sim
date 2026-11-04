"""Line coverage of the sim core by the test suite.

Runs both test binaries (Debug: optimised code misattributes lines) under
OpenCppCoverage, merges them, writes an HTML report and prints line
coverage per source folder for the core. Child processes are covered too:
the batch CLI test runs the built intercept_batch.

    cmake --build build --config Debug --target unit_tests validation_tests
    python tools/analysis/coverage.py              # summary + build/coverage/html
    python tools/analysis/coverage.py --files      # also every file, worst first
    python tools/analysis/coverage.py --report-only  # re-read the last run

The core is everything under src/ except the GUI: src/app and src/render,
other than the render math that has no GL in it (Camera, Mat4). The GUI is
checked on screen instead. A core .cpp that no test binary compiles has no
line data at all; those are listed separately, not counted as 0%.

Requires OpenCppCoverage (winget install OpenCppCoverage.OpenCppCoverage).
"""

import argparse
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
OUT_DIR = REPO_ROOT / "build" / "coverage"
TEST_BINARIES = ["unit_tests", "validation_tests"]
DEFAULT_TOOL = Path(r"C:\Program Files\OpenCppCoverage\OpenCppCoverage.exe")

GUI_DIRS = {"app", "render"}
GUI_FILES_IN_CORE = {"render/Camera.cpp", "render/Camera.h", "render/Mat4.h"}


def in_core(rel_path):
    """rel_path is relative to src/, forward slashes."""
    if rel_path in GUI_FILES_IN_CORE:
        return True
    return rel_path.split("/")[0] not in GUI_DIRS


def find_tool():
    found = shutil.which("OpenCppCoverage")
    if found:
        return Path(found)
    if DEFAULT_TOOL.exists():
        return DEFAULT_TOOL
    sys.exit("OpenCppCoverage not found (winget install OpenCppCoverage.OpenCppCoverage)")


def run_coverage(config):
    tool = find_tool()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    binaries = [REPO_ROOT / "build" / config / "tests" / f"{name}.exe" for name in TEST_BINARIES]
    for binary in binaries:
        if not binary.exists():
            sys.exit(f"missing {binary}: build the {config} test targets first")

    # Each binary's run feeds the next through binary coverage files; the
    # last one also writes the reports.
    previous = None
    for index, binary in enumerate(binaries):
        last = index == len(binaries) - 1
        command = [str(tool), "--quiet", "--cover_children", "--sources", str(SRC_DIR),
                   "--working_dir", str(REPO_ROOT)]
        if previous:
            command += ["--input_coverage", str(previous)]
        if last:
            command += ["--export_type", f"html:{OUT_DIR / 'html'}",
                        "--export_type", f"cobertura:{OUT_DIR / 'coverage.xml'}"]
        else:
            previous = OUT_DIR / f"{binary.stem}.cov"
            command += ["--export_type", f"binary:{previous}"]
        command += ["--", str(binary)]
        print(f"running {binary.name} under coverage...", flush=True)
        # A failing test still leaves usable coverage; report it and go on.
        result = subprocess.run(command, cwd=REPO_ROOT)
        if result.returncode != 0:
            print(f"warning: {binary.name} exited {result.returncode} (test failure?)", file=sys.stderr)


def read_cobertura(path):
    """{src-relative path: (lines covered, lines measured)}"""
    tree = ET.parse(path)
    # OpenCppCoverage's source root is the bare drive ("C:"); joined with
    # Path that would be drive-relative, so join as text.
    source = tree.find("sources/source")
    root_prefix = source.text.strip().rstrip("\\") + "\\" if source is not None and source.text else ""
    src_root = SRC_DIR.resolve()
    # A file compiled into both binaries appears once per binary (package):
    # a line is covered if either run hit it.
    line_hits = defaultdict(dict)
    for cls in tree.iter("class"):
        try:
            rel = Path(root_prefix + cls.get("filename")).resolve().relative_to(src_root).as_posix()
        except ValueError:
            continue
        hits = line_hits[rel]
        for line in cls.iter("line"):
            number = int(line.get("number"))
            hits[number] = hits.get(number, False) or int(line.get("hits")) > 0
    return {rel: (sum(hits.values()), len(hits)) for rel, hits in line_hits.items()}


def report(show_files):
    xml_path = OUT_DIR / "coverage.xml"
    if not xml_path.exists():
        sys.exit(f"no {xml_path}: run without --report-only first")
    files = {rel: counts for rel, counts in read_cobertura(xml_path).items() if in_core(rel)}

    by_dir = defaultdict(lambda: [0, 0])
    for rel, (covered, total) in files.items():
        folder = rel.split("/")[0]
        by_dir[folder][0] += covered
        by_dir[folder][1] += total

    def pct(covered, total):
        return 100.0 * covered / total if total else 100.0

    print(f"\n{'folder':<12}{'covered':>9}{'lines':>8}{'%':>8}")
    for folder in sorted(by_dir):
        covered, total = by_dir[folder]
        print(f"{folder:<12}{covered:>9}{total:>8}{pct(covered, total):>7.1f}%")
    all_covered = sum(c for c, _ in files.values())
    all_total = sum(t for _, t in files.values())
    print(f"{'CORE':<12}{all_covered:>9}{all_total:>8}{pct(all_covered, all_total):>7.1f}%")

    if show_files:
        print(f"\n{'file':<48}{'covered':>9}{'lines':>8}{'%':>8}")
        for rel, (covered, total) in sorted(files.items(), key=lambda kv: pct(*kv[1])):
            print(f"{rel:<48}{covered:>9}{total:>8}{pct(covered, total):>7.1f}%")

    untested = sorted(
        rel for rel in (p.relative_to(SRC_DIR).as_posix() for p in SRC_DIR.rglob("*.cpp"))
        if in_core(rel) and rel not in files)
    if untested:
        print("\ncore .cpp files no test binary compiles (no line data):")
        for rel in untested:
            print(f"  {rel}")
    print(f"\nHTML report: {OUT_DIR / 'html' / 'index.html'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default="Debug", help="build configuration of the test binaries")
    parser.add_argument("--files", action="store_true", help="list every core file, worst covered first")
    parser.add_argument("--report-only", action="store_true", help="summarise the last run without rerunning")
    args = parser.parse_args()
    if not args.report_only:
        run_coverage(args.config)
    report(args.files)


if __name__ == "__main__":
    main()
