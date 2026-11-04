"""tools/analysis/batch_compare.py: the regression gate. Its `--identical`
mode is what proves a behaviour-preserving change changed nothing, so each
way it could say "identical" wrongly is pinned here. Needs pandas."""

import contextlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from argparse import ArgumentTypeError, Namespace
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

try:
    import pandas as pd

    spec = importlib.util.spec_from_file_location("batch_compare", REPO / "tools" / "analysis" / "batch_compare.py")
    bc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bc)
except ImportError:  # pandas missing
    pd = bc = None

RESULT_HEADER = "run_id,seed,hit,miss_distance_m,wall_ms,error"
EVENT_HEADER = "run_id,sim_time_s,type,entity,data"


@unittest.skipIf(bc is None, "pandas is not installed")
class ParseSeeds(unittest.TestCase):
    def test_ranges_lists_and_mixtures(self):
        self.assertEqual(bc.parse_seeds("1-3"), [1, 2, 3])
        self.assertEqual(bc.parse_seeds("1,4,9"), [1, 4, 9])
        self.assertEqual(bc.parse_seeds("1-3,7"), [1, 2, 3, 7])
        self.assertEqual(bc.parse_seeds("5"), [5])

    def test_a_malformed_reversed_or_repeated_seed_list_is_refused(self):
        for bad in ("", "a", "5-1", "1,1", "1-3,2", "-3", "1,,2"):
            with self.assertRaises(SystemExit, msg=repr(bad)):
                bc.parse_seeds(bad)


@unittest.skipIf(bc is None, "pandas is not installed")
class Labels(unittest.TestCase):
    def test_a_label_becomes_part_of_file_names_so_it_is_restricted(self):
        self.assertEqual(bc.label_name("base-2.1_x"), "base-2.1_x")
        for bad in ("", "../x", "a/b", "a b", "-lead", "a:b"):
            with self.assertRaises(ArgumentTypeError, msg=repr(bad)):
                bc.label_name(bad)


@unittest.skipIf(bc is None, "pandas is not installed")
class PairingAndMovement(unittest.TestCase):
    def test_runs_are_paired_by_run_id_and_seed_so_shared_seeds_do_not_multiply(self):
        a = pd.DataFrame({"run_id": [0, 1], "seed": [7, 7], "hit": [1, 0]})
        b = pd.DataFrame({"run_id": [0, 1], "seed": [7, 7], "hit": [1, 1]})
        self.assertEqual(len(bc.paired(a, b)), 2)

    def test_a_resolution_that_appears_or_vanishes_counts_as_moved(self):
        nan = float("nan")
        p = pd.DataFrame({"resolution_time_s_a": [nan, 5.0, 1.0, 1.0, nan],
                          "resolution_time_s_b": [5.0, nan, 1.01, 1.2, nan]})
        moved = bc.resolution_moved(p)
        self.assertEqual(list(moved.index), [0, 1, 3])  # a resolved/unresolved flip, or more than 0.05 s


@unittest.skipIf(bc is None, "pandas is not installed")
class IdenticalGate(unittest.TestCase):
    def setUp(self):
        self._dir = tempfile.TemporaryDirectory()
        self.dir = Path(self._dir.name)

    def tearDown(self):
        self._dir.cleanup()

    def label(self, name, results, events, seeds=(1, 2), overrides=None, scenario="scenarios/s.json"):
        """A manifest for one label: writes its results and event CSVs."""
        csv = self.dir / f"{name}.csv"
        csv.write_text(results)
        (self.dir / f"{name}_events.csv").write_text(events)
        return {"label": name, "commit": "test", "seeds": list(seeds), "overrides": overrides or {},
                "results": {scenario: str(csv)}}

    BASE_RESULTS = RESULT_HEADER + "\n0,1,1,1.2345678901234567,10.5,\n1,2,0,40.0,11.5,\n"
    BASE_EVENTS = EVENT_HEADER + "\n0,1.0,LAUNCH,3,x\n0,1.0,RADAR_LOCK,3,y\n1,2.0,LAUNCH,3,x\n"

    def run_gate(self, ma, mb):
        common = [s for s in ma["results"] if s in mb["results"]]
        skipped = sorted(set(ma["results"]) ^ set(mb["results"]))
        with contextlib.redirect_stdout(io.StringIO()):
            return bc.diff_identical(ma, mb, common, skipped)

    def test_identical_apart_from_wall_clock_passes_whatever_the_row_order(self):
        a = self.label("a", self.BASE_RESULTS, self.BASE_EVENTS)
        reordered = RESULT_HEADER + "\n1,2,0,40.0,99.0,\n0,1,1,1.2345678901234567,98.0,\n"
        b = self.label("b", reordered, self.BASE_EVENTS)
        self.assertEqual(self.run_gate(a, b), 0)

    def test_a_one_ulp_change_in_a_cell_is_not_identical(self):
        a = self.label("a", self.BASE_RESULTS, self.BASE_EVENTS)
        b = self.label("b", self.BASE_RESULTS.replace("1.2345678901234567", "1.2345678901234569"), self.BASE_EVENTS)
        self.assertEqual(self.run_gate(a, b), 1)

    def test_events_in_a_different_order_within_a_run_are_not_identical(self):
        a = self.label("a", self.BASE_RESULTS, self.BASE_EVENTS)
        swapped = EVENT_HEADER + "\n0,1.0,RADAR_LOCK,3,y\n0,1.0,LAUNCH,3,x\n1,2.0,LAUNCH,3,x\n"
        b = self.label("b", self.BASE_RESULTS, swapped)
        self.assertEqual(self.run_gate(a, b), 1)

    def test_a_scenario_in_only_one_label_is_not_identical(self):
        a = self.label("a", self.BASE_RESULTS, self.BASE_EVENTS)
        b = self.label("b", self.BASE_RESULTS, self.BASE_EVENTS)
        a["results"]["scenarios/extra.json"] = a["results"]["scenarios/s.json"]
        self.assertEqual(self.run_gate(a, b), 1)

    def test_labels_sharing_no_scenario_are_not_identical(self):
        a = self.label("a", self.BASE_RESULTS, self.BASE_EVENTS, scenario="scenarios/one.json")
        b = self.label("b", self.BASE_RESULTS, self.BASE_EVENTS, scenario="scenarios/two.json")
        self.assertEqual(self.run_gate(a, b), 1)

    def test_labels_run_with_different_seeds_or_overrides_are_not_identical(self):
        a = self.label("a", self.BASE_RESULTS, self.BASE_EVENTS, seeds=(1, 2))
        b = self.label("b", self.BASE_RESULTS, self.BASE_EVENTS, seeds=(1, 3))
        self.assertEqual(self.run_gate(a, b), 1)
        c = self.label("c", self.BASE_RESULTS, self.BASE_EVENTS, overrides={"/x": 1})
        self.assertEqual(self.run_gate(a, c), 1)

    def test_diffing_labels_with_nothing_in_common_reports_instead_of_crashing(self):
        saved = bc.COMPARE_DIR
        bc.COMPARE_DIR = self.dir
        try:
            for name, scenario in (("x", "scenarios/one.json"), ("y", "scenarios/two.json")):
                manifest = self.label(name, self.BASE_RESULTS, self.BASE_EVENTS, scenario=scenario)
                (self.dir / f"{name}.json").write_text(json.dumps(manifest))
            with contextlib.redirect_stdout(io.StringIO()) as out:
                status = bc.cmd_diff(Namespace(a="x", b="y", identical=False))
            self.assertEqual(status, 1)
            self.assertIn("share no scenario", out.getvalue())
        finally:
            bc.COMPARE_DIR = saved


if __name__ == "__main__":
    unittest.main()
