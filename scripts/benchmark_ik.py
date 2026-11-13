"""Compare existing bounded myoHand IK budgets on identical offline frames."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motion_processing.csv_parser import parse_csv
from myohand.ik_retargeting import (KinematicFitter, load_landmarks, model_points,
                                    register_neutral, source_points)
from myohand.model_inspector import default_model_path


def summarize(rows: list[dict]) -> dict:
    """P95 error is over per-frame means of the five fingertip distances."""
    errors = np.array([r["tip_mean_error_mm"] for r in rows])
    compute = np.array([r["ik_compute_ms"] for r in rows])
    return {
        "scenario": rows[0]["scenario"], "max_nfev": rows[0]["max_nfev"],
        "frame_count": len(rows),
        "tip_mean_error_mm": float(errors.mean()),
        "tip_p95_error_mm": float(np.percentile(errors, 95)),
        "optimizer_success_pct": float(np.mean([r["optimizer_success"] for r in rows]) * 100),
        "max_nfev_reached_pct": float(np.mean([r["optimizer_status"] == 0 for r in rows]) * 100),
        "mean_nfev": float(np.mean([r["optimizer_nfev"] for r in rows])),
        "ik_compute_mean_ms": float(compute.mean()),
        "ik_compute_median_ms": float(np.median(compute)),
        "ik_compute_p95_ms": float(np.percentile(compute, 95)),
        "effective_ik_fps": float(1000 * len(rows) / compute.sum()),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_results(output: Path, rows: list[dict], summaries: list[dict]) -> None:
    for metric, ylabel, filename in (
        ("tip_mean_error_mm", "Mean five-fingertip error (mm)", "ik_error_comparison.png"),
        ("ik_compute_ms", "Optimizer compute time (ms)", "ik_compute_time_comparison.png"),
    ):
        fig, ax = plt.subplots(figsize=(11, 4.5))
        for summary in summaries:
            subset = [r for r in rows if r["scenario"] == summary["scenario"]]
            ax.plot([r["time_s"] for r in subset], [r[metric] for r in subset],
                    label=f"{summary['scenario']}: max_nfev={summary['max_nfev']}",
                    linewidth=1, alpha=0.8)
        ax.set(xlabel="Recording time (s)", ylabel=ylabel)
        ax.grid(alpha=0.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output / filename, dpi=160)
        plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    x = np.arange(len(summaries))
    ax.bar(x - 0.18, [s["optimizer_success_pct"] for s in summaries],
           width=0.36, label="Optimizer success (status > 0)")
    ax.bar(x + 0.18, [s["max_nfev_reached_pct"] for s in summaries],
           width=0.36, label="Evaluation limit (status = 0)")
    ax.set(xticks=x, xticklabels=[f"{s['scenario']}: {s['max_nfev']}" for s in summaries],
           xlabel="Scenario: max_nfev", ylabel="Frames (%)", ylim=(0, 105))
    ax.legend()
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output / "optimizer_success_comparison.png", dpi=160)
    plt.close(fig)


def run(args) -> list[dict]:
    if not args.max_nfev or any(n < 1 for n in args.max_nfev):
        raise ValueError("--max-nfev harus positif")
    if len(set(args.max_nfev)) != len(args.max_nfev):
        raise ValueError("--max-nfev tidak boleh duplikat")
    if not np.isfinite(args.neutral_end) or args.neutral_end <= 0:
        raise ValueError("--neutral-end harus positif dan finite")
    weights = [args.temporal_weight, args.neutral_weight, args.tip_weight]
    if not np.all(np.isfinite(weights)) or min(weights[:2]) < 0 or weights[2] <= 0:
        raise ValueError("Bobot objective tidak valid")
    model_path = Path(args.model) if args.model else default_model_path()
    if model_path is None or not model_path.is_file():
        raise FileNotFoundError("myohand.xml tidak ditemukan; gunakan --model")
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    reference, landmarks = load_landmarks(model, args.landmarks)
    recording = parse_csv(args.input)
    frames = recording.sorted_frames()
    sources = []
    for index, frame in enumerate(frames):
        points = source_points(frame, landmarks)
        if points is None or not np.all(np.isfinite(points)):
            raise ValueError(f"Frame {index} ({frame.timestamp_us}) landmark world tidak lengkap/finite")
        sources.append(points)
    sources = np.stack(sources)
    timestamps = np.array([f.timestamp_us for f in frames], dtype=np.int64)
    times = (timestamps - timestamps[0]) / 1e6
    neutral_mask = times <= args.neutral_end
    if np.count_nonzero(neutral_mask) < 10:
        raise ValueError("Interval neutral memerlukan minimal 10 frame")
    neutral_source = np.mean(sources[neutral_mask], axis=0)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    registration = register_neutral(neutral_source, model_points(model, data, reference, landmarks))

    def new_fitter():
        return KinematicFitter(model, reference, landmarks, neutral_source, registration,
                               temporal_weight_m=args.temporal_weight,
                               neutral_weight_m=args.neutral_weight, tip_weight=args.tip_weight)

    template = new_fitter()
    initial = template.default_qpos[template.qadr].copy()
    targets = np.stack([template.target_points(source) for source in sources])
    tip_ids = [i for i, landmark in enumerate(landmarks) if landmark.is_tip]
    print(f"Input: {len(frames)} frame; durasi {times[-1]:.3f}s; "
          f"neutral {np.count_nonzero(neutral_mask)} frame; invalid rows={recording.invalid_rows}", flush=True)
    rows, summaries = [], []
    for scenario_index, budget in enumerate(args.max_nfev):
        scenario = chr(65 + scenario_index) if scenario_index < 26 else str(scenario_index + 1)
        # Identical untimed warm-up for each scenario, discarded before resetting.
        warmup = new_fitter()
        previous = initial.copy()
        for target in targets[:3]:
            previous, _, _ = warmup.fit(target, previous, 12)
        fitter = new_fitter()
        previous = initial.copy()
        scenario_rows = []
        for index, target in enumerate(targets):
            previous, error, info = fitter.fit(target, previous, budget)
            row = {
                "scenario": scenario, "max_nfev": budget, "frame_index": index,
                "timestamp_us": int(timestamps[index]), "time_s": float(times[index]),
                "tip_mean_error_mm": float(np.mean(error[tip_ids]) * 1000),
                "all_mean_error_mm": float(np.mean(error) * 1000),
                **{f"optimizer_{key}": info[key] for key in
                   ("success", "status", "message", "nfev", "njev", "cost", "optimality")},
                "ik_compute_ms": info["ik_compute_ms"],
                **{f"{landmarks[i].rokoko}_error_mm": float(error[i] * 1000) for i in tip_ids},
            }
            scenario_rows.append(row)
            if (index + 1) % 250 == 0 or index == len(targets) - 1:
                print(f"{scenario} max_nfev={budget}: {index+1}/{len(targets)}", flush=True)
        summaries.append(summarize(scenario_rows))
        rows.extend(scenario_rows)
    output = Path(args.output).resolve()
    filenames = ["benchmark_summary.csv", "benchmark_per_frame.csv", "benchmark_metadata.json",
                 "ik_error_comparison.png", "ik_compute_time_comparison.png", "optimizer_success_comparison.png"]
    if any(output / name in {Path(args.input).resolve(), model_path.resolve(),
                             Path(args.landmarks).resolve()} for name in filenames):
        raise ValueError("Output akan menimpa input/model/landmarks")
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / "benchmark_summary.csv", summaries)
    write_csv(output / "benchmark_per_frame.csv", rows)
    metadata = {
        "input": str(Path(args.input).resolve()),
        "input_sha256": hashlib.sha256(Path(args.input).read_bytes()).hexdigest(),
        "model": str(model_path.resolve()),
        "landmarks": str(Path(args.landmarks).resolve()),
        "landmarks_sha256": hashlib.sha256(Path(args.landmarks).read_bytes()).hexdigest(),
        "python": sys.version, "platform": platform.platform(), "processor": platform.processor(),
        "numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__,
        "max_nfev": args.max_nfev, "scenario_order": args.max_nfev,
        "neutral_end_s": args.neutral_end, "neutral_frames": int(neutral_mask.sum()),
        "temporal_weight_m": args.temporal_weight, "neutral_weight_m": args.neutral_weight,
        "tip_weight": args.tip_weight,
        "ftol": 1e-8, "xtol": 1e-8, "gtol": 1e-9,
        "optimizer": "Existing KinematicFitter.fit / scipy.optimize.least_squares; analytic Jacobian; default method trf",
        "initial_finger_qpos": initial.tolist(),
        "warm_start": "Identical initial pose per scenario; previous solution within each scenario (no shared state across scenarios)",
        "registration": {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in registration.items()},
        "neutral_source_m": neutral_source.tolist(),
        "target_sha256": hashlib.sha256(targets.tobytes()).hexdigest(),
        "parser": {"invalid_rows": recording.invalid_rows, "duplicate_rows": recording.duplicate_rows,
                   "out_of_order_rows": recording.out_of_order_rows},
        "frame_count": len(frames), "dropped_frames": 0, "interpolation": False,
        "error_definition": "Euclidean distance to registered/scaled displacement targets; mean of five tips per frame; P95 over those frame means; includes neutral frames",
        "compute_definition": "perf_counter around least_squares only, including residual and Jacobian; excludes parsing, calibration, target construction, final errors, I/O and rendering",
        "fps_definition": "1000 / mean(ik_compute_ms); offline compute capacity estimate, NOT end-to-end streaming FPS",
        "timing_limitations": "One sequential pass per budget; identical 3-frame unmeasured warm-up at max_nfev=12 then fresh fitter. Timing depends on CPU load and scenario order.",
    }
    (output / "benchmark_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    plot_results(output, rows, summaries)
    print("\nScenario nfev  Mean/P95 tip(mm)  Success% Limit% Mean nfev  Compute mean/med/P95(ms)  IK FPS")
    for s in summaries:
        print(f"{s['scenario']:>4} {s['max_nfev']:>7}  {s['tip_mean_error_mm']:6.3f}/{s['tip_p95_error_mm']:6.3f}"
              f"  {s['optimizer_success_pct']:7.2f} {s['max_nfev_reached_pct']:6.2f} {s['mean_nfev']:9.2f}"
              f"  {s['ik_compute_mean_ms']:7.2f}/{s['ik_compute_median_ms']:6.2f}/{s['ik_compute_p95_ms']:6.2f}"
              f"  {s['effective_ik_fps']:7.1f}")
    print(f"\nOutput: {output}\nFPS offline = estimasi kapasitas komputasi optimizer; bukan FPS streaming end-to-end.")
    return summaries


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(ROOT / "recordings/solved_hand.csv"))
    parser.add_argument("--max-nfev", type=int, nargs="+", default=[8, 12, 20, 30])
    parser.add_argument("--model", help="myohand.xml; default instalasi MyoSuite")
    parser.add_argument("--landmarks", default=str(ROOT / "config/myohand_landmarks.json"))
    parser.add_argument("--output", default=str(ROOT / "outputs/ik_benchmark"))
    parser.add_argument("--neutral-end", type=float, default=0.5)
    parser.add_argument("--temporal-weight", type=float, default=0.001)
    parser.add_argument("--neutral-weight", type=float, default=0.0005)
    parser.add_argument("--tip-weight", type=float, default=2.0)
    args = parser.parse_args()
    try:
        run(args)
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        parser.exit(2, f"Benchmark gagal: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
