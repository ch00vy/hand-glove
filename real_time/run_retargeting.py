"""Run live Hand Solver RGMP -> myoHand IK -> MuJoCo viewer."""

from __future__ import annotations

import argparse
import contextlib
import queue
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import DEFAULT_HOST
from rgmp_client import RGMPClient, validate_endpoint
from myohand.ik_retargeting import load_landmarks
from myohand.model_inspector import default_model_path
from real_time.pipeline import LiveRetargeter


def run(args) -> int:
    host, port = validate_endpoint(args.host, args.port)
    if args.duration is not None and args.duration <= 0:
        raise ValueError("--duration harus positif")
    if args.viewer_fps <= 0:
        raise ValueError("--viewer-fps harus positif")
    model_path = Path(args.model) if args.model else default_model_path()
    if model_path is None or not model_path.is_file():
        raise FileNotFoundError("myohand.xml tidak ditemukan; gunakan --model")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    reference, landmarks = load_landmarks(model, args.landmarks)
    pipeline = LiveRetargeter(model, reference, landmarks, device_id=args.device_id,
                              neutral_window_s=args.neutral_window,
                              stable_p95_mm=args.stability_mm,
                              max_ik_evaluations=args.max_ik_evaluations,
                              tip_weight=args.tip_weight)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    client = RGMPClient(on_data=pipeline.on_data)
    last_pose = None
    last_metrics = None
    last_state = None
    try:
        viewer_context = contextlib.nullcontext(None)
        if not args.headless:
            from mujoco import viewer as mujoco_viewer
            viewer_context = mujoco_viewer.launch_passive(model, data)
        pipeline.start()
        client.connect(host, port)
        print(f"RGMP {host}:{port} -> myoHand; tahan tangan TERBUKA dan stabil selama kalibrasi.", flush=True)
        with viewer_context as viewer:
            started = time.monotonic()
            next_report = started + 1
            while viewer is None or viewer.is_running():
                now = time.monotonic()
                if args.duration is not None and now - started >= args.duration:
                    return 0 if pipeline.solved else 1
                for _ in range(32):
                    try:
                        kind, value = client.events.get_nowait()
                    except queue.Empty:
                        break
                    if kind == "status" and value != last_state:
                        print(f"RGMP: {value}", flush=True)
                        last_state = value
                    elif kind == "log" and ("Definition:" in value or "Error" in value or
                                             "Refused" in value or "terputus" in value):
                        print(value, flush=True)
                    elif kind == "done":
                        return 1
                for _ in range(32):
                    try:
                        kind, value = pipeline.events.get_nowait()
                    except queue.Empty:
                        break
                    if kind == "error":
                        print(value, file=sys.stderr, flush=True)
                        return 1
                    if kind == "calibration":
                        print(f"Kalibrasi: {value['samples']} frame; stabilitas p95 "
                              f"{value['stability_p95_mm']:.2f} mm; skala {value['scale']:.3f}; "
                              f"beda bentuk netral {value['neutral_shape_rmse_mm']:.1f} mm", flush=True)
                    else:
                        print(f"{kind}: {value}", flush=True)
                try:
                    while True:
                        last_pose = pipeline.poses.get_nowait()
                except queue.Empty:
                    pass
                if last_pose is not None:
                    data.qpos[:] = last_pose["qpos"]
                    data.qvel[:] = 0
                    mujoco.mj_forward(model, data)
                    last_metrics = last_pose
                    last_pose = None
                if viewer is not None:
                    viewer.sync()
                if now >= next_report:
                    age = (now - pipeline.last_input_monotonic) if pipeline.last_input_monotonic else None
                    print(f"world={pipeline.received_world}, IK={pipeline.solved}, "
                          f"dropped={pipeline.dropped_input}, invalid={pipeline.invalid_world}, "
                          f"last_world_age_s={age:.2f}" if age is not None else
                          f"world=0, IK=0; menunggu joints_world pada port {port}", flush=True)
                    if last_metrics is not None:
                        print(f"IK tip mean={last_metrics['tip_mean_error_mm']:.1f} mm, "
                              f"latency={last_metrics['latency_ms']:.1f} ms, "
                              f"optimizer_success={last_metrics['optimizer_success']}", flush=True)
                    next_report = now + 1
                time.sleep(1 / args.viewer_fps)
    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()
        pipeline.stop()
        if client._thread is not None:
            client._thread.join(timeout=3)
        print("Koneksi client retargeting ditutup; Hand Solver tetap berjalan.", flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Live Hand Solver port 12277 to myoHand MuJoCo kinematic retargeting")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=12277)
    parser.add_argument("--model", help="myohand.xml; default instalasi MyoSuite")
    parser.add_argument("--landmarks", default=str(ROOT / "config/myohand_landmarks.json"))
    parser.add_argument("--device-id", type=int, help="Pilih satu device RGMP jika ada beberapa")
    parser.add_argument("--neutral-window", type=float, default=0.5, help="Detik tangan terbuka stabil")
    parser.add_argument("--stability-mm", type=float, default=5.0, help="Ambang p95 stabilitas landmark")
    parser.add_argument("--max-ik-evaluations", type=int, default=12)
    parser.add_argument("--tip-weight", type=float, default=2.0)
    parser.add_argument("--viewer-fps", type=float, default=30.0)
    parser.add_argument("--headless", action="store_true", help="Jalankan tanpa viewer untuk pemeriksaan")
    parser.add_argument("--duration", type=float, help="Berhenti otomatis setelah N detik")
    args = parser.parse_args()
    try:
        return run(args)
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        parser.exit(2, f"Real-time retargeting gagal: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
