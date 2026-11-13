"""Hardware-free live RGMP/TCP integration checks."""

import contextlib
import io
import queue
import socket
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from myohand.ik_retargeting import load_landmarks, model_points
from myohand.model_inspector import default_model_path
from real_time.pipeline import LiveRetargeter, put_latest
from real_time.run_retargeting import run
from rgmp_client import RGMPClient, load_sdk


class LiveTests(unittest.TestCase):
    def test_latest_queue(self):
        q = queue.Queue(maxsize=1)
        self.assertFalse(put_latest(q, 1))
        self.assertTrue(put_latest(q, 2))
        self.assertEqual(q.get_nowait(), 2)

    @unittest.skipIf(default_model_path() is None, "MyoSuite model unavailable")
    def test_tcp_frames_calibration_and_live_ik(self):
        _, framing, _ = load_sdk()
        model = mujoco.MjModel.from_xml_path(str(default_model_path()))
        landmarks_path = Path(__file__).resolve().parents[1] / "config/myohand_landmarks.json"
        reference, landmarks = load_landmarks(model, landmarks_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        neutral = model_points(model, data, reference, landmarks)
        data.qpos[model.joint("mcp2_flexion").qposadr[0]] = .5
        mujoco.mj_forward(model, data)
        bent = model_points(model, data, reference, landmarks)
        definition = {"device_id": 7, "groups": [
            {"name": "joints_local", "streams": [
                {"target_frame": "right_hand", "reference_frame": "right_fore_arm",
                 "measure_type": "ORIENTATION", "data_type": "FLOAT[4]"}]},
            {"name": "joints_world", "streams": [
                {"target_frame": "right_hand", "reference_frame": "LTP_NED",
                 "measure_type": "TRANSFORM", "data_type": "FLOAT[7]"},
                *[{"target_frame": landmark.rokoko, "reference_frame": "LTP_NED",
                   "measure_type": "TRANSFORM", "data_type": "FLOAT[7]"}
                  for landmark in landmarks]]}]}
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        errors = []
        sent = threading.Event()
        peer_closed = threading.Event()

        def serve():
            try:
                conn, _ = listener.accept()
                with conn:
                    conn.settimeout(4)
                    conn.sendall(framing.pack_stream_definition(definition))
                    for i in range(90):
                        ts = 1_000_000 + 12_000 * i
                        points = neutral if i < 55 else bent
                        conn.sendall(framing.pack_data_frame(7, 0, ts,
                                                             framing.pack_quat((0, 0, 0, 1))))
                        payload = bytearray(framing.pack_transform((0, 0, 0), (0, 0, 0, 1)))
                        for point in points:
                            payload.extend(framing.pack_transform(tuple(point), (0, 0, 0, 1)))
                        conn.sendall(framing.pack_data_frame(7, 1, ts, payload))
                        time.sleep(.012)
                    sent.set()
                    self.assertEqual(conn.recv(1), b"")
                    peer_closed.set()
            except Exception as exc:
                errors.append(exc)
            finally:
                listener.close()

        server = threading.Thread(target=serve, daemon=True)
        server.start()
        pipeline = LiveRetargeter(model, reference, landmarks, neutral_window_s=.5,
                                  max_ik_evaluations=12)
        pipeline.start()
        client = RGMPClient(on_data=pipeline.on_data)
        try:
            client.connect("127.0.0.1", port)
            self.assertTrue(sent.wait(4))
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and (not pipeline.calibrated or pipeline.solved < 10):
                time.sleep(.02)
            self.assertTrue(pipeline.calibrated)
            self.assertGreater(pipeline.solved, 10)
            pose = pipeline.poses.get(timeout=2)
            while not pipeline.poses.empty():
                pose = pipeline.poses.get_nowait()
            self.assertEqual(pose["device_id"], 7)
            self.assertEqual(pose["qpos"].shape, (model.nq,))
            self.assertGreater(pose["qpos"][model.joint("mcp2_flexion").qposadr[0]], .2)
            self.assertTrue(np.isfinite(pose["qpos"]).all())
            self.assertGreater(pose["ik_compute_ms"], 0)
            self.assertGreaterEqual(pose["latency_ms"], pose["ik_compute_ms"])
            self.assertEqual(pose["optimizer_success"], pose["optimizer_status"] > 0)
            self.assertTrue(1 <= pose["optimizer_nfev"] <= 12)
            self.assertGreater(pose["optimizer_njev"], 0)
            self.assertGreaterEqual(pose["optimizer_cost"], 0)
            self.assertGreaterEqual(pose["optimizer_optimality"], 0)
            self.assertTrue(pose["optimizer_message"])
        finally:
            client.disconnect()
            pipeline.stop()
            client._thread.join(3)
            server.join(3)
        self.assertTrue(peer_closed.is_set())
        self.assertFalse(errors, errors)

    @unittest.skipIf(default_model_path() is None, "MyoSuite model unavailable")
    def test_headless_runner_with_rgmp_server(self):
        _, framing, _ = load_sdk()
        model = mujoco.MjModel.from_xml_path(str(default_model_path()))
        landmarks_path = Path(__file__).resolve().parents[1] / "config/myohand_landmarks.json"
        reference, landmarks = load_landmarks(model, landmarks_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        points = model_points(model, data, reference, landmarks)
        streams = [{"target_frame": "right_hand", "reference_frame": "LTP_NED",
                    "measure_type": "TRANSFORM", "data_type": "FLOAT[7]"}]
        streams += [{"target_frame": landmark.rokoko, "reference_frame": "LTP_NED",
                     "measure_type": "TRANSFORM", "data_type": "FLOAT[7]"}
                    for landmark in landmarks]
        definition = {"device_id": 9, "groups": [{"name": "joints_world", "streams": streams}]}
        payload = bytearray(framing.pack_transform((0, 0, 0), (0, 0, 0, 1)))
        for point in points:
            payload.extend(framing.pack_transform(tuple(point), (0, 0, 0, 1)))
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        errors = []
        closed = threading.Event()

        def serve():
            try:
                conn, _ = listener.accept()
                with conn:
                    conn.settimeout(4)
                    conn.sendall(framing.pack_stream_definition(definition))
                    for i in range(80):
                        conn.sendall(framing.pack_data_frame(9, 0, 1_000_000 + i * 12_000, payload))
                        time.sleep(.012)
                    if conn.recv(1) == b"":
                        closed.set()
            except Exception as exc:
                errors.append(exc)
            finally:
                listener.close()

        server = threading.Thread(target=serve, daemon=True)
        server.start()
        args = SimpleNamespace(host="127.0.0.1", port=port, model=None, landmarks=str(landmarks_path),
                               device_id=None, neutral_window=.5, stability_mm=5.0,
                               max_ik_evaluations=12, tip_weight=2.0, viewer_fps=30.0,
                               headless=True, duration=1.6)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = run(args)
        server.join(3)
        self.assertEqual(code, 0, output.getvalue())
        self.assertIn("retargeting aktif", output.getvalue())
        self.assertTrue(closed.is_set())
        self.assertFalse(errors, errors)


if __name__ == "__main__":
    unittest.main()
