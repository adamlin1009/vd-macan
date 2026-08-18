import re
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
PLAN_URL = "https://adaml.in/log/2026-08-05-macan-instrumentation-plan"
DAY1_URL = "https://adaml.in/log/2026-08-16-six-runs-two-damper-maps"


def read(path):
    return (REPO / path).read_text()


class PublicCredibilityContractTests(unittest.TestCase):
    def test_readme_uses_live_posts_and_root_safe_hash_commands(self):
        readme = read("README.md")
        self.assertIn(PLAN_URL, readme)
        self.assertIn(DAY1_URL, readme)
        self.assertNotIn("github.com/adamlin1009/website", readme)
        self.assertIn(
            "(cd data/20260815_afternoon && shasum -a 256 -c SHA256SUMS)",
            readme,
        )
        self.assertIn(
            "(cd data/sd_dump_20260816 && shasum -a 256 -c SHA256SUMS)",
            readme,
        )

    def test_public_text_has_no_honest_or_tap_test_terms(self):
        paths = [REPO / "README.md", REPO / "CLAUDE.md", REPO / "requirements.txt"]
        for directory, suffixes in (
            (REPO / "docs", {".md"}),
            (REPO / "data", {".md"}),
            (REPO / "tools", {".py"}),
            (REPO / "matlab", {".m", ".md"}),
        ):
            paths.extend(
                path for path in directory.rglob("*") if path.suffix in suffixes
            )
        for path in paths:
            text = path.read_text()
            with self.subTest(path=path.relative_to(REPO)):
                self.assertIsNone(re.search(r"\bhonest(?:ly)?\b", text, re.I))
                self.assertIsNone(re.search(r"\btap[-_ ](?:test|check)\b", text, re.I))

    def test_scope_is_one_day_with_controlled_work_only_as_future_study(self):
        readme = read("README.md").lower()
        shakedown = read("docs/shakedown.md").lower()
        self.assertIn("one autocross day", readme)
        self.assertIn("future studies", readme)
        self.assertIn("only after a suitable venue exists", readme)
        self.assertNotIn("day 2", shakedown)
        self.assertNotIn("every future event day", read("docs/day1-thread.md").lower())

    def test_thread_reports_six_figures_and_keeps_raw_grip_primary(self):
        thread = read("docs/day1-thread.md").lower()
        self.assertIn("all six figures", thread)
        self.assertIn("raw roof measurement: 0.97 g sustained, 1.14 g peak", thread)
        self.assertRegex(
            thread,
            r"exploratory roll correction[^\n]*0\.93 g sustained[^\n]*1\.09 g peak",
        )

    def test_generator_and_time_figure_keep_public_precision_and_scope(self):
        generator = read("tools/day1_analysis.py")
        figure = read("figures/day1/fig02_run_times.svg")
        self.assertIn('POST_TEMPLATE = r"""', generator)
        self.assertIn("GPS virtual-gate calibration residual ~0.11 s", generator)
        self.assertIn("runs[i]['time']:.1f", generator)
        self.assertNotIn("runs[i]['time']:.2f", generator)
        self.assertIn("## Future studies, only after a suitable venue exists", generator)
        self.assertIn("CALIBRATION RMS 0.11 S", figure)
        self.assertIn(
            'aria-label="GPS virtual-gate estimates for three unmatched '
            'competition runs per PASM mode"',
            figure,
        )
        self.assertIn(">53.1</text>", figure)
        self.assertNotIn(">53.11</text>", figure)

    def test_generated_day_one_copy_keeps_scope_without_canned_slogans(self):
        generator = read("tools/day1_analysis.py")
        for phrase in (
            "The data had other ideas",
            "the whole reason this project exists",
            "the grip prediction is dead. Good.",
            "the clock that lied",
            "Wringing the dataset",
            "my favorite trick",
            "The graveyard, with causes of death",
            "Negative results are results",
            "## Steal these",
        ):
            with self.subTest(phrase=phrase):
                self.assertNotIn(phrase.lower(), generator.lower())

        self.assertIn(
            "| run | PASM | GPS virtual-gate estimate [s] |",
            generator,
        )
        self.assertRegex(
            generator,
            r"three unmatched\s+competition runs per mode",
        )
        self.assertRegex(
            generator,
            r"neither the time gap nor the\s+response differences can be"
            r"\s+attributed to PASM",
        )
        self.assertRegex(
            generator,
            r"raw\s+roof-mounted values remain the\s+primary grip result",
        )
        self.assertRegex(generator, r"exploratory\s+sensitivity check")
        self.assertIn("The current campaign was one autocross day. It is complete.", generator)
        self.assertIn("None is scheduled.", generator)

    def test_instrument_characterization_names_and_impulse_status(self):
        self.assertTrue((REPO / "tools/imu_characterize.py").is_file())
        self.assertFalse((REPO / "tools/tap_check.py").exists())
        self.assertTrue((REPO / "matlab/characterize_imu.m").is_file())
        self.assertFalse((REPO / "matlab/tap_test.m").exists())
        tool = read("tools/imu_characterize.py").lower()
        self.assertIn("impulse observations are informational", tool)
        self.assertNotIn("logger cleared for the ride block", tool)

    def test_python_is_authoritative_and_matlab_status_is_scoped(self):
        readme = read("README.md")
        self.assertIn("Python is the authoritative analysis", readme)
        self.assertIn("[matlab/README.md](matlab/README.md)", readme)
        self.assertNotIn("## MATLAB pipeline (status)", readme)
        matlab = read("matlab/README.md").lower()
        self.assertIn("unverified future work", matlab)
        self.assertIn("none of these `.m` files has been executed", matlab)


if __name__ == "__main__":
    unittest.main()
