"""Fit a myoHand qpos trajectory to recorded Rokoko hand landmark motion."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motion_processing.csv_parser import parse_csv
from myohand.model_inspector import default_model_path
from myohand.ik_retargeting import (KinematicFitter, load_landmarks, model_points,
                                    register_neutral, source_points)


def run(args) -> dict:
    if args.stride < 1 or args.max_evaluations < 1:
        raise ValueError("--stride dan --max-evaluations harus positif")
    model_path = Path(args.model) if args.model else default_model_path()
    if model_path is None or not model_path.is_file():
        raise FileNotFoundError("Model myohand.xml tidak ditemukan; gunakan --model")
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    reference, landmarks = load_landmarks(model, args.landmarks)
    recording = parse_csv(args.input)
    frames = recording.sorted_frames()
    sources = []
    for index, frame in enumerate(frames):
        points = source_points(frame, landmarks)
        if points is None:
            raise ValueError(f"Frame {index} ({frame.timestamp_us}) kehilangan landmark world")
        sources.append(points)
    sources = np.stack(sources)
    timestamps = np.array([frame.timestamp_us for frame in frames], dtype=np.int64)
    times = (timestamps - timestamps[0]) / 1e6
    neutral_mask = times <= args.neutral_end
    if np.count_nonzero(neutral_mask) < 10:
        raise ValueError("Interval neutral memerlukan minimal 10 frame")
    neutral_source = np.mean(sources[neutral_mask], axis=0)
    neutral_data = mujoco.MjData(model)
    mujoco.mj_forward(model, neutral_data)
    neutral_model = model_points(model, neutral_data, reference, landmarks)
    registration = register_neutral(neutral_source, neutral_model)
    fitter = KinematicFitter(model, reference, landmarks, neutral_source, registration,
                            temporal_weight_m=args.temporal_weight,
                            neutral_weight_m=args.neutral_weight, tip_weight=args.tip_weight)
    optimized = sorted(set(range(0, len(frames), args.stride)) | {len(frames) - 1})
    fitted = np.empty((len(optimized), len(fitter.joint_ids)))
    optimizer_results = []
    previous = fitter.default_qpos[fitter.qadr].copy()
    for k, index in enumerate(optimized):
        target = fitter.target_points(sources[index])
        previous, error, status = fitter.fit(target, previous, args.max_evaluations)
        fitted[k] = previous
        optimizer_results.append(status)
        if k % 100 == 0:
            print(f"IK {k+1}/{len(optimized)} t={times[index]:.2f}s "
                  f"mean landmark error={np.mean(error)*1000:.1f} mm", flush=True)
    # Preserve the recording timestamps. Interpolated frames are evaluated
    # independently; their error is reported as held-out temporal validation.
    finger_trajectory = np.column_stack([
        np.interp(times, times[optimized], fitted[:, j]) for j in range(fitted.shape[1])
    ])
    full_trajectory = np.stack([fitter.full_qpos(q) for q in finger_trajectory])
    errors = np.empty((len(frames), len(landmarks)))
    fitted_landmarks = np.empty((len(frames), len(landmarks), 3))
    target_landmarks = np.empty_like(fitted_landmarks)
    for i, q in enumerate(finger_trajectory):
        actual = fitter.evaluate(q)
        target = fitter.target_points(sources[i])
        fitted_landmarks[i] = actual
        target_landmarks[i] = target
        errors[i] = np.linalg.norm(actual - target, axis=1)
    baseline_errors = np.linalg.norm(target_landmarks - fitter.neutral_model[None, :, :], axis=2)
    tip_ids = [i for i, landmark in enumerate(landmarks) if landmark.is_tip]
    holdout = np.ones(len(frames), dtype=bool)
    holdout[optimized] = False
    def metrics(values):
        return {"rmse_mm": float(np.sqrt(np.mean(values ** 2)) * 1000),
                "median_mm": float(np.median(values) * 1000),
                "p95_mm": float(np.percentile(values, 95) * 1000),
                "max_mm": float(np.max(values) * 1000)}
    summary = {
        "input": str(Path(args.input).resolve()), "model": str(model_path.resolve()),
        "frame_count": len(frames), "duration_s": float(times[-1]), "stride": args.stride,
        "optimized_frame_count": len(optimized), "heldout_frame_count": int(np.sum(holdout)),
        "neutral_end_s": args.neutral_end,
        "registration": {"proper_rotation_source_to_model": registration["rotation"].tolist(),
                         "uniform_scale": registration["scale"],
                         "translation_m": registration["translation"].tolist(),
                         "neutral_shape_rmse_mm": registration["neutral_shape_rmse_m"] * 1000},
        "method": "hand-relative Rokoko world landmark displacements from neutral; "
                  "uniform similarity registration to default myoHand; bounded finger IK; fixed wrist; "
                  "linear qpos interpolation at non-optimized frames",
        "landmark_count": len(landmarks),
        "landmark_names": [x.rokoko for x in landmarks],
        "all_landmarks": metrics(errors),
        "fingertips": metrics(errors[:, tip_ids]),
        "static_open_hand_fingertips": metrics(baseline_errors[:, tip_ids]),
        "heldout_fingertips": metrics(errors[holdout][:, tip_ids]) if np.any(holdout) else None,
        "per_fingertip": {landmarks[i].rokoko: metrics(errors[:, i]) for i in tip_ids},
        "optimizer": {"success_count": sum(x["success"] for x in optimizer_results),
                      "total_count": len(optimizer_results),
                      "median_function_evaluations": float(np.median([x["nfev"] for x in optimizer_results]))},
        "joint_limit_near_count": int(np.count_nonzero(np.isclose(finger_trajectory, fitter.lower,
                                                                    atol=.01) | np.isclose(finger_trajectory,
                                                                                          fitter.upper, atol=.01))),
        "joint_limit_near_fraction": float(np.mean(np.isclose(finger_trajectory, fitter.lower,
                                                                atol=.01) | np.isclose(finger_trajectory,
                                                                                      fitter.upper, atol=.01))),
        "validation_scope": "numerical/kinematic landmark fit; no independent anatomical or instrumented ground truth",
    }
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output / "myohand_ik_trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp_us", "time_s", *[model.joint(i).name for i in range(model.njnt)],
                         "landmark_mean_error_mm", "fingertip_mean_error_mm", "optimized_frame"])
        for i, qpos in enumerate(full_trajectory):
            writer.writerow([int(timestamps[i]), times[i], *qpos,
                             np.mean(errors[i])*1000, np.mean(errors[i, tip_ids])*1000,
                             i in optimized])
    np.savez_compressed(output / "myohand_ik_trajectory.npz", qpos=full_trajectory,
                        timestamp_us=timestamps, time_s=times,
                        error_m=errors, target_landmarks_m=target_landmarks,
                        fitted_landmarks_m=fitted_landmarks,
                        landmark_names=np.array([x.rokoko for x in landmarks]),
                        joint_names=np.array([model.joint(i).name for i in range(model.njnt)]),
                        optimized_frame=np.isin(np.arange(len(frames)), optimized))
    (output / "myohand_ik_validation.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    fig, ax = plt.subplots(figsize=(10, 4))
    for i in tip_ids:
        ax.plot(times, errors[:, i]*1000, label=landmarks[i].rokoko.replace("right_finger", "finger"))
    ax.set(xlabel="Time (s)", ylabel="IK landmark error (mm)",
           title="MyoHand fingertip fit to registered Rokoko displacement")
    ax.legend(ncol=5)
    fig.tight_layout()
    fig.savefig(output / "myohand_ik_error.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
    for ax, i in zip(axes, tip_ids):
        ax.plot(times, np.linalg.norm(target_landmarks[:, i], axis=1)*1000,
                label="Rokoko target, registered")
        ax.plot(times, np.linalg.norm(fitted_landmarks[:, i], axis=1)*1000,
                label="myoHand FK", alpha=.85)
        ax.set_ylabel(f"Finger {landmarks[i].rokoko.split('finger')[1][0]}\nmm")
    axes[0].legend(ncol=2)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Fingertip distance from model hand reference")
    fig.tight_layout()
    fig.savefig(output / "myohand_tip_comparison.png", dpi=150)
    plt.close(fig)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline myoHand landmark IK from solved_hand.csv")
    parser.add_argument("--input", default=str(ROOT / "recordings/solved_hand.csv"))
    parser.add_argument("--model", help="myohand.xml; defaults to installed MyoSuite model")
    parser.add_argument("--landmarks", default=str(ROOT / "config/myohand_landmarks.json"))
    parser.add_argument("--output", default=str(ROOT / "outputs"))
    parser.add_argument("--neutral-end", type=float, default=0.5, help="Neutral baseline interval, seconds")
    parser.add_argument("--stride", type=int, default=4, help="Fit every Nth frame, interpolate between")
    parser.add_argument("--max-evaluations", type=int, default=50)
    parser.add_argument("--temporal-weight", type=float, default=0.001)
    parser.add_argument("--neutral-weight", type=float, default=0.0005)
    parser.add_argument("--tip-weight", type=float, default=2.0,
                        help="Relative weight of fingertip landmarks in IK")
    args = parser.parse_args()
    try:
        report = run(args)
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        parser.exit(2, f"Retargeting gagal: {exc}\n")
    print(f"Selesai: {report['frame_count']} frame, "
          f"tip RMSE {report['fingertips']['rmse_mm']:.2f} mm, "
          f"held-out tip RMSE {report['heldout_fingertips']['rmse_mm']:.2f} mm "
          f"({report['heldout_frame_count']} frame).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
