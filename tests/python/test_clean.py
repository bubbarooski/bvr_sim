"""tools/clean.py deletes files, so what it will delete is pinned: the dry run
lists exactly what --yes removes, nothing tracked by git is ever removed, and
an empty folder it did not empty is left alone. Needs git for the tracked-file
test."""

import contextlib
import importlib.util
import io
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("clean", REPO / "tools" / "clean.py")
clean = importlib.util.module_from_spec(spec)
spec.loader.exec_module(clean)


class Patched(unittest.TestCase):
    """Points clean.py's repo constants at a scratch tree."""

    def setUp(self):
        self._dir = tempfile.TemporaryDirectory()
        self.root = Path(self._dir.name).resolve()
        self._saved = (clean.REPO_ROOT, clean.BUILD, clean.RESULTS)
        clean.REPO_ROOT, clean.BUILD, clean.RESULTS = self.root, self.root / "build", self.root / "experiments" / "results"

    def tearDown(self):
        clean.REPO_ROOT, clean.BUILD, clean.RESULTS = self._saved
        self._dir.cleanup()


class DryRun(Patched):
    def test_the_dry_run_lists_every_path_it_would_delete(self):
        files = []
        for i in range(40):
            f = self.root / f"stale_{i:02d}.log"
            f.write_text("x")
            files.append(f)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            clean.report([("Logs", files)], delete=False)
        listed = [line.strip() for line in out.getvalue().splitlines() if line.startswith("  ")]
        self.assertEqual(len(listed), 40, "the listing must be exactly what --yes deletes")
        self.assertNotIn("more", out.getvalue())


@unittest.skipIf(shutil.which("git") is None, "git is not installed")
class NeverDeletesTrackedFiles(Patched):
    def git(self, *args):
        subprocess.run(["git", "-c", "user.email=t@t", "-c", "user.name=t", *args], cwd=self.root, check=True,
                       capture_output=True)

    def test_a_tracked_file_or_a_folder_holding_one_is_refused(self):
        (self.root / "keep").mkdir()
        (self.root / "keep" / "tracked.txt").write_text("x")
        (self.root / "loose.txt").write_text("x")
        self.git("init", "-q")
        self.git("add", "keep/tracked.txt")
        self.git("commit", "-q", "-m", "t")
        refused = clean.tracked_among([self.root / "keep" / "tracked.txt", self.root / "keep", self.root / "loose.txt"])
        self.assertEqual({p.name for p in refused}, {"tracked.txt", "keep"})


class Pruning(Patched):
    def test_only_folders_the_deletion_emptied_are_removed(self):
        results = clean.RESULTS
        (results / "was_empty").mkdir(parents=True)
        (results / "emptied").mkdir()
        gone = results / "emptied" / "old.csv"
        gone.write_text("x")
        (results / "kept").mkdir()
        (results / "kept" / "new.csv").write_text("x")
        gone.unlink()
        clean.prune_empty_dirs(results, [gone])
        self.assertFalse((results / "emptied").exists())
        self.assertTrue((results / "was_empty").exists(), "an already-empty folder is not ours to remove")
        self.assertTrue((results / "kept" / "new.csv").exists())


class BuildFolder(Patched):
    def test_googletests_own_lib_folder_is_not_stale_in_a_single_config_tree(self):
        lib = clean.BUILD / "lib"
        lib.mkdir(parents=True)
        (lib / "libgtest.a").write_text("x")
        self.assertEqual(clean.stale_outputs(clean.BUILD, {"intercept_batch"}), [])


if __name__ == "__main__":
    unittest.main()
