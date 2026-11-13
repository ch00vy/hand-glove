"""Verify benchmark aggregation and isolation of sequential offline scenarios."""

import contextlib
import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from myohand.ik_retargeting import load_landmarks, model_points
from myohand.model_inspector import default_model_path
from scripts.benchmark_ik import ROOT, run, summarize


class BenchmarkTests(unittest.TestCase):
    def test_summary_uses_frame_means_and_total_compute_for_fps(self):
        rows = [dict(scenario="A", max_nfev=8, tip_mean_error_mm=error,
                     ik_compute_ms=ms, optimizer_success=success,
                     optimizer_status=status, optimizer_nfev=nfev)
                for error, ms, success, status, nfev in
                [(1, 10, True, 1, 3), (3, 30, False, 0, 8)]]
        result = summarize(rows)
        self.assertEqual(result["tip_mean_error_mm"], 2)
        self.assertAlmostEqual(result["tip_p95_error_mm"], 2.9)
        self.assertEqual(result["optimizer_success_pct"], 50)
        self.assertEqual(result["max_nfev_reached_pct"], 50)
        self.assertEqual(result["mean_nfev"], 5.5)
        self.assertEqual(result["ik_compute_mean_ms"], 20)
        self.assertEqual(result["ik_compute_median_ms"], 20)
        self.assertAlmostEqual(result["ik_compute_p95_ms"], 29)
        self.assertEqual(result["effective_ik_fps"], 50)

    @unittest.skipIf(default_model_path() is None, "MyoSuite model unavailable")
    def test_all_frames_same_targets_and_reset_independent_of_scenario_order(self):
        model = mujoco.MjModel.from_xml_path(str(default_model_path()))
        landmarks_path = ROOT / "config/myohand_landmarks.json"
        reference, landmarks = load_landmarks(model, landmarks_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        neutral = model_points(model, data, reference, landmarks)
        data.qpos[model.joint("mcp2_flexion").qposadr[0]] = 0.5
        mujoco.mj_forward(model, data)
        bent = model_points(model, data, reference, landmarks)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "solved_hand.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                for i in range(14):
                    points = neutral if i < 10 else bent
                    for name, point in [("right_hand", np.zeros(3)),
                                        *[(l.rokoko, p) for l, p in zip(landmarks, points)]]:
                        values = " ".join(map(str, [*point, 0, 0, 0, 1]))
                        writer.writerow([1_000_000 + 12_000 * i, "joints_world", name,
                                         "LTP_NED", "TRANSFORM", "FLOAT[7]", values])
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            outputs = []
            for budgets in ([1, 3], [3, 1]):
                output = Path(temp) / str(budgets[0])
                args = SimpleNamespace(input=str(path), max_nfev=budgets, model=None,
                                       landmarks=str(landmarks_path), output=str(output), neutral_end=.11,
                                       temporal_weight=.001, neutral_weight=.0005, tip_weight=2.0)
                with contextlib.redirect_stdout(io.StringIO()), patch('scripts.benchmark_ik.plot_results'):
                    summaries = run(args)
                self.assertTrue(all(s["frame_count"] == 14 for s in summaries))
                metadata = json.loads((output / 'benchmark_metadata.json').read_text())
                self.assertEqual(metadata['dropped_frames'], 0)
                with (output / 'benchmark_per_frame.csv').open(newline='', encoding='utf-8') as handle:
                    rows = list(csv.DictReader(handle))
                self.assertEqual(len(rows), 28)
                outputs.append((metadata, rows))
            self.assertEqual(before, hashlib.sha256(path.read_bytes()).hexdigest())
            self.assertEqual(outputs[0][0]['target_sha256'], outputs[1][0]['target_sha256'])
            for budget in (1, 3):
                first = [r for r in outputs[0][1] if int(r['max_nfev']) == budget]
                second = [r for r in outputs[1][1] if int(r['max_nfev']) == budget]
                self.assertEqual([r['timestamp_us'] for r in first], [r['timestamp_us'] for r in second])
                for a, b in zip(first, second):
                    self.assertTrue(1 <= int(a['optimizer_nfev']) <= budget)
                    for key in a.keys() - {'scenario', 'ik_compute_ms'}:
                        self.assertEqual(a[key], b[key], key)


if __name__ == '__main__':
    unittest.main()
