"""Analyze a recorded RGMP solved hand without touching the source CSV."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motion_processing.csv_parser import parse_csv
from motion_processing.skeleton import build_hierarchy, write_inventory, write_skeleton_markdown
from motion_processing.calibration import calibrate
from motion_processing.joint_angles import angle_rows, infer_candidate_axes, write_angles
from motion_processing.validation import make_plots, world_geometry
from myohand.model_inspector import default_model_path, inspect_model, mapping_template
from myohand.retargeting import retarget_frame


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def analyze(args) -> dict:
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    recording = parse_csv(args.input)
    summary = recording.summary()
    hierarchy = build_hierarchy(recording)
    summary["hierarchy"] = hierarchy
    write_inventory(recording, hierarchy, output / "joint_inventory.csv")
    write_skeleton_markdown(hierarchy, output / "skeleton.md")

    calibration = calibrate(recording, args.calibration_start, args.calibration_end,
                            args.stability_threshold_deg)
    write_json(output / "neutral_calibration.json", calibration)

    axis_path = Path(args.axis_config)
    axis_data = json.loads(axis_path.read_text(encoding="utf-8")) if axis_path.is_file() else {"joints": {}}
    axes = axis_data.get("joints", {})
    if not isinstance(axes, dict):
        raise ValueError("axis_config.joints harus object JSON")
    unknown = set(axes) - set(calibration["joints"])
    if unknown:
        raise ValueError(f"Joint pada axis_config tidak ada dalam rekaman: {sorted(unknown)}")
    inferred = infer_candidate_axes(recording, calibration)
    write_json(output / "axis_candidates.json", inferred)
    rows = list(angle_rows(recording, calibration, axes, inferred))
    write_angles(rows, output / "joint_angles_candidate.csv")
    summary["candidate_axis_count"] = sum(x.get("status") == "candidate_pca" for x in inferred.values())
    summary["configured_axis_count"] = len([x for x in axes.values() if x.get("flexion_axis")])
    summary["calibration_status"] = calibration["overall_status"]

    geometry, series = world_geometry(recording)
    write_json(output / "world_geometry.json", geometry)
    plots = make_plots(recording, calibration, rows, series, hierarchy, output / "plots")
    summary["plots"] = plots

    model_path = Path(args.model).resolve() if args.model else default_model_path()
    if model_path is not None:
        inventory, model = inspect_model(model_path)
        write_json(output / "myohand_model_inventory.json", inventory)
        mapping_path = Path(args.mapping)
        mapping_path.parent.mkdir(parents=True, exist_ok=True)
        if not mapping_path.exists():
            write_json(mapping_path, mapping_template(inventory))
        mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
        valid = [m for m in mapping.get("mappings", []) if m.get("mapping_status") == "validated"]
        summary["myohand"] = {"model_path": str(model_path), "joint_count": inventory["joint_count"],
                              "validated_mappings": len(valid), "retargeted_frames": 0}
        if valid:
            angle_by_time: dict[int, dict] = {}
            for row in rows:
                angle_by_time.setdefault(row["timestamp"], {})[row["joint_name"]] = row
            with (output / "retargeted_qpos.csv").open("w", newline="", encoding="utf-8") as handle:
                names = [f"qpos_{i}" for i in range(model.nq)]
                writer = csv.writer(handle)
                writer.writerow(["timestamp_us", "applied_joint_count", *names])
                for ts in sorted(angle_by_time):
                    qpos, applied = retarget_frame(model, mapping, angle_by_time[ts])
                    writer.writerow([ts, len(applied), *qpos])
                    if applied:
                        summary["myohand"]["retargeted_frames"] += 1
    else:
        summary["myohand"] = {"status": "model unavailable; mapping not generated"}
    write_json(output / "recording_summary.json", summary)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline RGMP Hand Solver calibration and candidate-angle analysis")
    parser.add_argument("--input", default=str(ROOT / "recordings/solved_hand.csv"), help="Source CSV; never modified")
    parser.add_argument("--output", default=str(ROOT / "outputs"), help="Directory for generated results")
    parser.add_argument("--calibration-start", type=float, default=0, help="Start seconds from first frame")
    parser.add_argument("--calibration-end", type=float, default=1.5, help="End seconds from first frame")
    parser.add_argument("--stability-threshold-deg", type=float, default=5.0,
                        help="Maximum p95 angular deviation for stable calibration")
    parser.add_argument("--axis-config", default=str(ROOT / "config/axis_mapping.json"),
                        help="Editable local hinge axes; missing file means no explicit axes")
    parser.add_argument("--model", help="Explicit myoHand XML path; default installed MyoSuite model")
    parser.add_argument("--mapping", default=str(ROOT / "config/myohand_mapping.json"),
                        help="Mapping file; created only when missing")
    args = parser.parse_args()
    try:
        summary = analyze(args)
    except (OSError, ValueError, RuntimeError, KeyError, ImportError) as exc:
        parser.exit(2, f"Analisis gagal: {exc}\n")
    print(f"{summary['frame_count']} frame, {summary['joint_count']} joint, "
          f"{summary['duration_s']:.3f} s; kalibrasi {summary['calibration_status']}; "
          f"{summary['candidate_axis_count']} sumbu kandidat.")
    print(f"Hasil: {Path(args.output).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
