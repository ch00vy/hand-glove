"""Bounded live-frame pipeline. The RGMP callback never performs IK or MuJoCo FK."""

from __future__ import annotations

import queue
import threading
import time
from collections import deque

import mujoco
import numpy as np

from motion_processing.csv_parser import HandFrame, measurement_from_stream
from myohand.ik_retargeting import KinematicFitter, model_points, register_neutral, source_points


def put_latest(destination: queue.Queue, item) -> bool:
    """Replace an old item when full; returns True if an item was dropped."""
    dropped = False
    try:
        destination.put_nowait(item)
        return dropped
    except queue.Full:
        try:
            destination.get_nowait()
            dropped = True
        except queue.Empty:
            pass
        destination.put_nowait(item)
        return dropped


class LiveRetargeter:
    def __init__(self, model, reference_body: int, landmarks, *, device_id: int | None = None,
                 neutral_window_s: float = 0.5, stable_p95_mm: float = 5.0,
                 min_neutral_frames: int = 15, max_ik_evaluations: int = 12,
                 tip_weight: float = 2.0):
        if neutral_window_s <= 0 or stable_p95_mm <= 0 or min_neutral_frames < 2 or max_ik_evaluations < 1:
            raise ValueError("Parameter kalibrasi/IK tidak valid")
        self.model = model
        self.reference_body = reference_body
        self.landmarks = landmarks
        self.device_id = device_id
        self.neutral_window_us = int(neutral_window_s * 1e6)
        self.stable_p95_m = stable_p95_mm / 1000
        self.min_neutral_frames = min_neutral_frames
        self.max_ik_evaluations = max_ik_evaluations
        self.tip_weight = tip_weight
        self.incoming: queue.Queue = queue.Queue(maxsize=1)
        self.poses: queue.Queue = queue.Queue(maxsize=1)
        self.events: queue.SimpleQueue = queue.SimpleQueue()
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.received_world = 0
        self.dropped_input = 0
        self.invalid_world = 0
        self.solved = 0
        self.last_input_monotonic = None
        self.last_solved_monotonic = None
        self.last_timestamp_us = None
        self.calibrated = False
        self._selected_device = device_id

    def start(self) -> None:
        if self.worker is not None:
            raise RuntimeError("LiveRetargeter sudah dimulai")
        self.worker = threading.Thread(target=self._run, name="myohand-ik", daemon=True)
        self.worker.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=3)
            if self.worker.is_alive():
                self.events.put(("warning", "Worker IK belum berhenti dalam 3 detik"))

    def on_data(self, message: dict) -> None:
        """Called by RGMP reader thread; retain only complete world-group messages."""
        if self.stop_event.is_set() or message.get("group_name") != "joints_world":
            return
        device = message["device_id"]
        if self._selected_device is None:
            self._selected_device = device
            self.events.put(("device", f"Memilih device RGMP {device}"))
        if device != self._selected_device:
            return
        self.received_world += 1
        self.last_input_monotonic = time.monotonic()
        if put_latest(self.incoming, (message, self.last_input_monotonic)):
            self.dropped_input += 1

    def _decode_points(self, message: dict):
        frame = HandFrame(int(message["timestamp_us"]))
        for stream in message["streams"]:
            entry = stream["metadata"]
            if entry.get("target_frame"):
                measurement = measurement_from_stream("joints_world", entry, stream["values"])
                frame.groups["joints_world"][measurement.joint] = measurement
        return source_points(frame, self.landmarks)

    def _run(self) -> None:
        neutral_samples = deque()
        fitter = None
        previous = None
        last_ts = None
        tip_ids = [i for i, landmark in enumerate(self.landmarks) if landmark.is_tip]
        neutral_data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, neutral_data)
        neutral_model = model_points(self.model, neutral_data, self.reference_body, self.landmarks)
        self.events.put(("state", "Menunggu frame joints_world dari Hand Solver"))
        last_warn = 0.0
        while not self.stop_event.is_set():
            try:
                message, arrival = self.incoming.get(timeout=0.1)
            except queue.Empty:
                continue
            ts = int(message["timestamp_us"])
            if last_ts is not None and ts <= last_ts:
                self.invalid_world += 1
                continue
            last_ts = ts
            try:
                source = self._decode_points(message)
                if source is None or not np.all(np.isfinite(source)):
                    raise ValueError("Landmark world tidak lengkap/non-finite")
            except (ValueError, KeyError, IndexError) as exc:
                self.invalid_world += 1
                if time.monotonic() - last_warn > 2:
                    self.events.put(("warning", f"Frame world dilewati: {exc}"))
                    last_warn = time.monotonic()
                continue
            if fitter is None:
                neutral_samples.append((ts, source))
                while neutral_samples and ts - neutral_samples[0][0] > self.neutral_window_us:
                    neutral_samples.popleft()
                if (len(neutral_samples) < self.min_neutral_frames or
                        ts - neutral_samples[0][0] < 0.8 * self.neutral_window_us):
                    continue
                neutral = np.mean(np.stack([sample for _, sample in neutral_samples]), axis=0)
                deviations = np.linalg.norm(np.stack([sample for _, sample in neutral_samples]) - neutral,
                                            axis=2)
                p95 = float(np.percentile(deviations, 95))
                if p95 > self.stable_p95_m:
                    if time.monotonic() - last_warn > 2:
                        self.events.put(("state", f"Kalibrasi: tahan tangan terbuka dan stabil; p95={p95*1000:.1f} mm"))
                        last_warn = time.monotonic()
                    continue
                try:
                    registration = register_neutral(neutral, neutral_model)
                    fitter = KinematicFitter(self.model, self.reference_body, self.landmarks,
                                              neutral, registration, tip_weight=self.tip_weight)
                except (ValueError, np.linalg.LinAlgError) as exc:
                    self.events.put(("error", f"Kalibrasi gagal: {exc}"))
                    self.stop_event.set()
                    return
                previous = fitter.default_qpos[fitter.qadr].copy()
                self.calibrated = True
                self.events.put(("state", "Kalibrasi netral selesai; retargeting aktif"))
                self.events.put(("calibration", {
                    "samples": len(neutral_samples), "stability_p95_mm": p95 * 1000,
                    "scale": registration["scale"],
                    "neutral_shape_rmse_mm": registration["neutral_shape_rmse_m"] * 1000,
                }))
            try:
                target = fitter.target_points(source)
                previous, error, info = fitter.fit(target, previous, self.max_ik_evaluations)
                qpos = fitter.full_qpos(previous)
            except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as exc:
                self.events.put(("error", f"IK gagal pada timestamp {ts}: {exc}"))
                self.stop_event.set()
                return
            self.solved += 1
            self.last_solved_monotonic = time.monotonic()
            self.last_timestamp_us = ts
            put_latest(self.poses, {
                "qpos": qpos, "timestamp_us": ts, "device_id": self._selected_device,
                "tip_mean_error_mm": float(np.mean(error[tip_ids]) * 1000),
                "all_mean_error_mm": float(np.mean(error) * 1000),
                "latency_ms": (self.last_solved_monotonic - arrival) * 1000,
                "optimizer_success": info["success"],
            })
