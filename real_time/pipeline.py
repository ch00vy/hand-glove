"""Bounded live-frame pipeline. The RGMP callback never performs IK or MuJoCo FK."""

from __future__ import annotations

import queue
import json
import threading
import time
from collections import deque
from pathlib import Path

import mujoco
import numpy as np

from motion_processing.csv_parser import HandFrame, measurement_from_stream
from myohand.ik_retargeting import KinematicFitter, model_points, register_neutral, source_points
from myohand.wrist_retargeting import (OrientationCalibration, WristRetargeter,
                                       JOINT_NAMES, extract_orientations)


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
                 min_neutral_frames: int = 15, max_ik_evaluations: int = 20,
                 tip_weight: float = 2.0, wrist_mode: str = 'relative',
                 wrist_stability_deg: float = 3.0, wrist_calibration_path=None):
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
        if wrist_mode not in ('off', 'relative', 'forearm-twist'):
            raise ValueError('Mode wrist tidak dikenal')
        self.wrist_mode = wrist_mode
        self.wrist_qadr = [model.joint(name).qposadr[0] for name in JOINT_NAMES]
        self.wrist_calibration = OrientationCalibration(neutral_window_s, min_neutral_frames,
                                                       wrist_stability_deg)
        self.wrist_calibration_path = wrist_calibration_path
        self.wrist_calibrated = False
        self.recalibrate_wrist_event = threading.Event()
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

    def recalibrate_wrist(self):
        """Thread-safe request; finger neutral registration is preserved."""
        self.recalibrate_wrist_event.set()

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
        arrival = time.perf_counter()
        self.received_world += 1
        self.last_input_monotonic = time.monotonic()
        if put_latest(self.incoming, (message, arrival)):
            self.dropped_input += 1

    def _decode_frame(self, message: dict):
        frame = HandFrame(int(message["timestamp_us"]))
        for stream in message["streams"]:
            entry = stream["metadata"]
            if entry.get("target_frame"):
                try:
                    measurement = measurement_from_stream("joints_world", entry, stream["values"])
                except (ValueError, KeyError, IndexError):
                    # A malformed forearm orientation must not stop finger IK.
                    if entry['target_frame'] == 'right_fore_arm':
                        continue
                    raise
                if measurement.joint in frame.groups['joints_world']:
                    raise ValueError('Duplicate world target_frame')
                frame.groups["joints_world"][measurement.joint] = measurement
        return frame

    def _decode_points(self, message: dict):
        return source_points(self._decode_frame(message), self.landmarks)

    def _run(self) -> None:
        neutral_samples = deque()
        fitter = None
        wrist = None
        held_wrist = None
        previous = None
        last_ts = None
        tip_ids = [i for i, landmark in enumerate(self.landmarks) if landmark.is_tip]
        neutral_data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, neutral_data)
        neutral_model = model_points(self.model, neutral_data, self.reference_body, self.landmarks)
        self.events.put(("state", "Menunggu frame joints_world dari Hand Solver"))
        last_warn = 0.0
        last_wrist_warn = 0.0
        if self.wrist_mode == 'forearm-twist':
            self.events.put(('warning', 'Forearm twist adalah proxy; rotasi bahu/siku dan drift '
                             'tidak dapat dipisahkan tanpa referensi upper arm.'))
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
                frame = self._decode_frame(message)
                source = source_points(frame, self.landmarks)
                if source is None or not np.all(np.isfinite(source)):
                    raise ValueError("Landmark world tidak lengkap/non-finite")
            except (ValueError, KeyError, IndexError) as exc:
                self.invalid_world += 1
                if time.monotonic() - last_warn > 2:
                    self.events.put(("warning", f"Frame world dilewati: {exc}"))
                    last_warn = time.monotonic()
                continue
            if self.recalibrate_wrist_event.is_set():
                self.recalibrate_wrist_event.clear()
                self.wrist_calibration.samples.clear()
                self.wrist_calibration.neutral = None
                self.wrist_calibrated = False
                wrist = None
                self.events.put(('state', 'Kalibrasi ulang wrist; netral jari dipertahankan'))
            orientation = None
            wrist_issue = None
            if self.wrist_mode != 'off':
                try:
                    orientation = extract_orientations(frame)
                    # Finger landmarks and orientations all share one world reference.
                    world = frame.groups['joints_world']
                    if any(world[x.rokoko].parent != orientation.reference for x in self.landmarks):
                        raise ValueError('Reference landmark tidak sama dengan hand/forearm')
                    if wrist is None:
                        self.wrist_calibration.add(ts, orientation)
                except (ValueError, KeyError) as exc:
                    wrist_issue = str(exc)
                    self.wrist_calibration.samples.clear()
                    if wrist is None:
                        self.wrist_calibration.neutral = None
                if wrist_issue and time.monotonic() - last_wrist_warn > 2:
                    self.events.put(('warning', f'Wrist ditahan: {wrist_issue}'))
                    last_wrist_warn = time.monotonic()
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
            if (wrist is None and self.wrist_mode != 'off' and
                    self.wrist_calibration.neutral is not None and wrist_issue is None):
                try:
                    wrist = WristRetargeter(self.model, self.reference_body,
                                            fitter.registration['rotation'],
                                            self.wrist_calibration.neutral, self.wrist_mode)
                except ValueError as exc:
                    self.events.put(('error', f'Konfigurasi wrist gagal: {exc}'))
                    self.stop_event.set()
                    return
                self.wrist_calibrated = True
                self.events.put(('state', f'Kalibrasi wrist selesai; mode={self.wrist_mode}'))
                neutral_o = self.wrist_calibration.neutral
                calibration = {'forearm_xyzw': neutral_o.forearm.as_quat().tolist(),
                               'hand_xyzw': neutral_o.hand.as_quat().tolist(),
                               'reference_frame': neutral_o.reference,
                               'source_hand_to_model_hand': fitter.registration['rotation'].tolist(),
                               'mode': self.wrist_mode,
                               'stability_max_deg': self.wrist_calibration.stability_max_deg}
                self.events.put(('wrist_calibration', calibration))
                if self.wrist_calibration_path:
                    try:
                        path = Path(self.wrist_calibration_path)
                        path.parent.mkdir(parents=True, exist_ok=True)
                        path.write_text(json.dumps(calibration, indent=2) + '\n', encoding='utf-8')
                    except OSError as exc:
                        self.events.put(('warning', f'Gagal menyimpan kalibrasi wrist: {exc}'))
            try:
                target = fitter.target_points(source)
                previous, error, info = fitter.fit(target, previous, self.max_ik_evaluations)
                qpos = fitter.full_qpos(previous)
            except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as exc:
                self.events.put(("error", f"IK gagal pada timestamp {ts}: {exc}"))
                self.stop_event.set()
                return
            wrist_metrics = {'wrist_orientation_error_deg': None, 'wrist_compute_ms': 0.,
                             'wrist_optimizer_success': False, 'wrist_optimizer_status': None,
                             'wrist_mode': self.wrist_mode,
                             'wrist_observability': 'unavailable',
                             'wrist_sync_status': ('disabled' if self.wrist_mode == 'off' else
                                                   'calibrating' if wrist is None else 'missing_or_invalid')}
            if wrist is not None and orientation is not None and wrist_issue is None:
                try:
                    held_wrist, solved_metrics = wrist.fit(orientation)
                    wrist_metrics.update(solved_metrics)
                    wrist_metrics['wrist_sync_status'] = 'same_world_message'
                except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as exc:
                    if time.monotonic() - last_wrist_warn > 2:
                        self.events.put(('warning', f'Wrist ditahan: {exc}'))
                        last_wrist_warn = time.monotonic()
            # Explicit hold on missing data, never report a stale orientation error.
            if held_wrist is not None:
                qpos[self.wrist_qadr] = held_wrist
            wrist_metrics.update({f'{name}_angle_deg': float(np.rad2deg(qpos[adr]))
                                  for name, adr in zip(JOINT_NAMES, self.wrist_qadr)})
            self.solved += 1
            # perf_counter avoids coarse monotonic ticks on some Windows/Python
            # versions; age/status bookkeeping still uses monotonic as before.
            latency_ms = (time.perf_counter() - arrival) * 1000
            self.last_solved_monotonic = time.monotonic()
            self.last_timestamp_us = ts
            put_latest(self.poses, {
                "qpos": qpos, "timestamp_us": ts, "device_id": self._selected_device,
                "tip_mean_error_mm": float(np.mean(error[tip_ids]) * 1000),
                "all_mean_error_mm": float(np.mean(error) * 1000),
                "latency_ms": latency_ms,
                "ik_compute_ms": info["ik_compute_ms"],
                'dropped_input': self.dropped_input,
                **wrist_metrics,
                **{f"optimizer_{key}": info[key] for key in
                   ("success", "status", "message", "nfev", "njev", "cost", "optimality")},
            })
