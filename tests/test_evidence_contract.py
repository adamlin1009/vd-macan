import datetime as dt
import hashlib
import json
import math
import stat
import struct
import sys
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
TOOLS = REPO / "tools"
sys.path.insert(0, str(TOOLS))

import day1_analysis  # noqa: E402
import imu_characterize  # noqa: E402


SESSION = REPO / "data" / "20260815_afternoon"
PROCESSED = SESSION / "processed"
FIGURES = REPO / "figures" / "day1"


def manifest_entries(manifest):
    entries = {}
    for line in manifest.read_text().splitlines():
        if not line.strip():
            continue
        digest, name = line.split(maxsplit=1)
        name = name.lstrip("*")
        path = (manifest.parent / name).resolve()
        path.relative_to(manifest.parent.resolve())
        entries[name] = (digest, path)
    return entries


def make_flag61_frame(when, values):
    milliseconds = when.microsecond // 1000
    stamp = bytes([
        when.year - 2000,
        when.month,
        when.day,
        when.hour,
        when.minute,
        when.second,
        milliseconds & 0xFF,
        milliseconds >> 8,
    ])
    return b"\x55\x61" + struct.pack("<9h", *values) + stamp


class RawEvidenceContractTests(unittest.TestCase):
    def assert_manifest_valid(self, manifest, expected_names):
        entries = manifest_entries(manifest)
        self.assertEqual(set(entries), set(expected_names))
        for name, (expected_digest, path) in entries.items():
            with self.subTest(manifest=manifest, file=name):
                self.assertTrue(path.is_file())
                actual_digest = hashlib.sha256(path.read_bytes()).hexdigest()
                self.assertEqual(actual_digest, expected_digest)

    def test_session_raw_files_are_complete_and_match_manifest(self):
        expected = {"racebox.csv"}
        expected.update(
            str(path.relative_to(SESSION))
            for directory in (SESSION / "imu_sd", SESSION / "app_capture")
            for path in directory.iterdir()
            if path.is_file()
        )
        self.assert_manifest_valid(SESSION / "SHA256SUMS", expected)

    def test_full_sd_dump_is_complete_and_matches_manifest(self):
        root = REPO / "data" / "sd_dump_20260816"
        expected = {
            path.name for path in root.iterdir()
            if path.is_file() and path.name != "SHA256SUMS"
        }
        self.assert_manifest_valid(root / "SHA256SUMS", expected)

    def test_raw_data_is_not_executable_and_tools_are(self):
        executable_bits = stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
        raw_paths = {
            path
            for manifest in (
                SESSION / "SHA256SUMS",
                REPO / "data" / "sd_dump_20260816" / "SHA256SUMS",
            )
            for _, path in manifest_entries(manifest).values()
        }
        for path in raw_paths:
            with self.subTest(raw_file=path.relative_to(REPO)):
                self.assertFalse(path.stat().st_mode & executable_bits)
        for path in TOOLS.glob("*.py"):
            with self.subTest(tool=path.relative_to(REPO)):
                self.assertTrue(path.stat().st_mode & executable_bits)


class ImuParserCharacterizationTests(unittest.TestCase):
    def test_flag61_frames_decode_signed_channels_and_five_ms_clock(self):
        start = dt.datetime(2026, 8, 15, 14, 52, 7)
        frames = []
        for index in range(12):
            when = start + dt.timedelta(milliseconds=5 * index)
            values = (
                -32768 + index,
                index,
                32767 - index,
                -1000 - index,
                1000 + index,
                index * 2,
                -90,
                0,
                90,
            )
            frames.append(make_flag61_frame(when, values))

        rows = imu_characterize.parse_flag61(b"".join(frames))

        self.assertIsNotNone(rows)
        self.assertEqual(len(rows), 12)
        self.assertEqual(rows[0]["acc"], [-32768, 0, 32767])
        self.assertEqual(rows[-1]["gyr"], [-1011, 1011, 22])
        intervals = [
            (right["t"] - left["t"]).total_seconds()
            for left, right in zip(rows, rows[1:])
        ]
        self.assertTrue(all(value == 0.005 for value in intervals))

    def test_dedupe_keeps_a_frame_when_acceleration_or_gyro_changes(self):
        acc = day1_analysis.np.array([
            [1, 2, 3],
            [1, 2, 3],
            [1, 2, 4],
            [1, 2, 4],
        ])
        gyr = day1_analysis.np.array([
            [4, 5, 6],
            [4, 5, 6],
            [4, 5, 6],
            [4, 5, 7],
        ])
        self.assertEqual(
            day1_analysis.dedupe(acc, gyr).tolist(),
            [True, False, True, True],
        )


class Day1AnalysisContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = day1_analysis.analyze()

    def test_run_detection_has_six_ordered_valid_runs(self):
        runs = self.result["runs"]
        self.assertEqual(len(runs), 6)
        self.assertEqual(day1_analysis.MODES, [
            "Normal", "Sport+", "Normal", "Sport+", "Normal", "Sport+",
        ])
        for index, run in enumerate(runs):
            with self.subTest(run=index + 1):
                self.assertLess(run["a"], run["b"])
                self.assertLess(run["t_s"], run["t_f"])
                self.assertGreater(run["time"], 50.0)
                self.assertLess(run["time"], 54.0)
                if index:
                    self.assertLess(runs[index - 1]["t_f"], run["t_s"])
        for gate in (self.result["gate_start"], self.result["gate_finish"]):
            self.assertAlmostEqual(float(day1_analysis.np.linalg.norm(gate[1])), 1.0)

    def test_analysis_invariants_keep_sources_and_samples_defensible(self):
        imu = self.result["imu"]
        self.assertEqual(imu["file"], [
            "WIT39.TXT", "WIT39.TXT", "WIT39.TXT",
            "WIT39.TXT", "WIT40.TXT", "WIT40.TXT",
        ])
        self.assertTrue(all(value >= 0.80 for value in imu["xc"]))
        self.assertTrue(all(value > 0.0 for value in imu["corr_ay"]))
        self.assertEqual(len(self.result["rg_samples"]), 2617)
        self.assertGreater(
            min(self.result["rg_slopes"][index] for index in day1_analysis.IDX_N),
            max(self.result["rg_slopes"][index] for index in day1_analysis.IDX_S),
        )

    def test_day1_headlines_match_published_precision(self):
        result = self.result
        imu = result["imu"]
        self.assertEqual(
            [f"{run['time']:.2f}" for run in result["runs"]],
            ["53.11", "51.98", "52.12", "52.33", "51.90", "51.21"],
        )
        self.assertEqual(
            [f"{value:.2f}" for value in imu["roll"]],
            ["4.32", "4.81", "4.28", "4.59", "4.55", "4.95"],
        )
        self.assertEqual(
            [f"{result['roll_n']:.2f}", f"{result['roll_s']:.2f}"],
            ["4.38", "4.79"],
        )
        self.assertEqual(
            [round(imu["off_first"]), round(imu["off_last"])],
            [131, 182],
        )
        self.assertEqual(
            [f"{min(imu['xc']):.2f}", f"{max(imu['xc']):.2f}"],
            ["0.84", "0.90"],
        )
        self.assertEqual(
            [f"{min(imu['corr_ay']):+.2f}", f"{max(imu['corr_ay']):+.2f}"],
            ["+0.79", "+0.90"],
        )
        self.assertEqual(
            [f"{imu['norm_n']:.2f}", f"{imu['norm_s']:.2f}"],
            ["3.24", "3.28"],
        )
        self.assertEqual(
            [f"{result['lat_p95']:.2f}", f"{result['lat_max']:.2f}"],
            ["0.97", "1.14"],
        )
        self.assertEqual(
            [f"{value:+.2f}" for value in result["rg_slopes"]],
            ["+2.65", "+1.94", "+3.03", "+2.26", "+3.12", "+2.09"],
        )
        self.assertEqual(
            [f"{result['rgn']:.2f}", f"{result['rgs']:.2f}"],
            ["2.93", "2.10"],
        )
        self.assertEqual(
            [f"{result['lat_p95_corr']:.2f}", f"{result['lat_max_corr']:.2f}"],
            ["0.93", "1.09"],
        )

    def assert_json_close(self, expected, actual, path="summary"):
        self.assertEqual(type(expected), type(actual), path)
        if isinstance(expected, dict):
            self.assertEqual(set(expected), set(actual), path)
            for key in expected:
                self.assert_json_close(expected[key], actual[key], f"{path}.{key}")
        elif isinstance(expected, list):
            self.assertEqual(len(expected), len(actual), path)
            for index, (left, right) in enumerate(zip(expected, actual)):
                self.assert_json_close(left, right, f"{path}[{index}]")
        elif isinstance(expected, float):
            self.assertTrue(
                math.isclose(expected, actual, rel_tol=0.0, abs_tol=1e-12),
                f"{path}: {expected!r} != {actual!r}",
            )
        else:
            self.assertEqual(expected, actual, path)

    def test_generated_outputs_are_repeatable_and_match_committed_artifacts(self):
        with tempfile.TemporaryDirectory() as first, \
                tempfile.TemporaryDirectory() as second:
            roots = []
            for root_name in (first, second):
                root = Path(root_name)
                processed = root / "processed"
                figures = root / "figures"
                day1_analysis.write_processed(self.result, processed)
                day1_analysis.write_figures(self.result, figures)
                roots.append((processed, figures))

            for first_root, second_root in zip(roots[0], roots[1]):
                first_files = {
                    path.relative_to(first_root): path
                    for path in first_root.rglob("*") if path.is_file()
                }
                second_files = {
                    path.relative_to(second_root): path
                    for path in second_root.rglob("*") if path.is_file()
                }
                self.assertEqual(set(first_files), set(second_files))
                for name in first_files:
                    with self.subTest(repeatable=name):
                        self.assertEqual(
                            first_files[name].read_bytes(),
                            second_files[name].read_bytes(),
                        )

            generated_processed, generated_figures = roots[0]
            for committed_root, generated_root in (
                (PROCESSED, generated_processed),
                (FIGURES, generated_figures),
            ):
                committed_files = {
                    path.relative_to(committed_root): path
                    for path in committed_root.rglob("*") if path.is_file()
                }
                generated_files = {
                    path.relative_to(generated_root): path
                    for path in generated_root.rglob("*") if path.is_file()
                }
                self.assertEqual(set(committed_files), set(generated_files))
                for name in committed_files:
                    if name == Path("summary.json"):
                        self.assert_json_close(
                            json.loads(committed_files[name].read_text()),
                            json.loads(generated_files[name].read_text()),
                        )
                    else:
                        with self.subTest(committed=name):
                            self.assertEqual(
                                committed_files[name].read_bytes(),
                                generated_files[name].read_bytes(),
                            )


if __name__ == "__main__":
    unittest.main()
