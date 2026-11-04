"""experiments/_framework.py: the pointer helper, and what run_batch does when
runs fail or the runner cannot start. Needs pandas; the run_batch tests also
need a built Release intercept_batch and skip without one."""

import importlib
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments"))

try:
    framework = importlib.import_module("_framework")
except ImportError:  # pandas missing
    framework = None


@unittest.skipIf(framework is None, "pandas is not installed")
class DotToPointer(unittest.TestCase):
    def test_dotted_paths_and_indices(self):
        self.assertEqual(framework.dot_to_pointer("missile_doctrine.nav_gain"), "/missile_doctrine/nav_gain")
        self.assertEqual(framework.dot_to_pointer("entities[1].components.signature.rcs_scale"),
                         "/entities/1/components/signature/rcs_scale")
        self.assertEqual(framework.dot_to_pointer("a[0][2].b"), "/a/0/2/b")

    def test_a_leading_index_is_not_an_empty_segment(self):
        self.assertEqual(framework.dot_to_pointer("[0].a"), "/0/a")

    def test_tilde_and_slash_are_escaped_as_rfc_6901_requires(self):
        self.assertEqual(framework.dot_to_pointer("a~b.c/d"), "/a~0b/c~1d")

    def test_an_empty_segment_is_an_error_not_a_key_named_nothing(self):
        for bad in ("", ".", "a..b", "a.", ".a"):
            with self.assertRaises(ValueError, msg=repr(bad)):
                framework.dot_to_pointer(bad)


@unittest.skipIf(framework is None, "pandas is not installed")
@unittest.skipUnless(framework is not None and framework.BATCH_EXE.exists(), "intercept_batch is not built")
class RunBatch(unittest.TestCase):
    SCENARIO = "scenarios/1_core_01_first_shot.json"

    def setUp(self):
        self._results = tempfile.TemporaryDirectory()
        self._saved = framework.RESULTS_DIR
        framework.RESULTS_DIR = Path(self._results.name)

    def tearDown(self):
        framework.RESULTS_DIR = self._saved
        self._results.cleanup()

    def test_a_failed_run_is_a_row_with_an_error_not_an_exception(self):
        runs = [
            {"run_id": 0, "seed": 1},
            {"run_id": 1, "seed": 1, "overrides": {"/missile_doctrine/launch_mode": "BOGUS"}},
        ]
        path = framework.run_batch(self.SCENARIO, runs, max_sim_time_s=1.0, name="unittest_failed_run")
        df = framework.load_results(path)
        self.assertEqual(len(df), 2)
        self.assertTrue(df.error.isna().iloc[0])
        self.assertIn("launch_mode", df.error.iloc[1])

    def test_a_runner_that_wrote_nothing_is_an_error(self):
        with self.assertRaises(RuntimeError):
            framework.run_batch("scenarios/no_such_scenario.json", [{"run_id": 0, "seed": 1}],
                                max_sim_time_s=1.0, name="unittest_bad_scenario")


if __name__ == "__main__":
    unittest.main()
