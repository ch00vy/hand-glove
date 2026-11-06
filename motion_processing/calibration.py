"""Neutral local orientation calibration using SciPy rotation means."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def normalize_quat(quat) -> np.ndarray:
    q = np.asarray(quat, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion harus 4 komponen finite")
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError("Quaternion nol")
    return q / norm


def relative_rotation(neutral, current) -> Rotation:
    """Local-frame convention: R_delta = R_neutral.inv() * R_current."""
    return Rotation.from_quat(normalize_quat(neutral)).inv() * Rotation.from_quat(normalize_quat(current))


def calibrate(recording, start_s: float = 0, end_s: float = 1.5,
              stable_threshold_deg: float = 5.0, minimum_samples: int = 10) -> dict:
    if not 0 <= start_s < end_s:
        raise ValueError("Interval kalibrasi harus 0 <= start < end")
    frames = recording.sorted_frames()
    if not recording.group_joints.get("joints_local"):
        raise ValueError("Grup joints_local tidak tersedia untuk kalibrasi")
    first_ts = frames[0].timestamp_us
    selected = [f for f in frames if start_s <= (f.timestamp_us - first_ts) / 1e6 <= end_s]
    if len(selected) < minimum_samples:
        raise ValueError(f"Hanya {len(selected)} frame pada interval kalibrasi; perlu {minimum_samples}")
    result = {"convention": "xyzw; R_delta = R_neutral^-1 * R_local_current",
              "source_group": "joints_local", "interval_requested_s": [start_s, end_s],
              "interval_actual_s": [(selected[0].timestamp_us-first_ts)/1e6,
                                    (selected[-1].timestamp_us-first_ts)/1e6],
              "stable_threshold_deg": stable_threshold_deg, "joints": {}}
    for joint in sorted(recording.group_joints.get("joints_local", set())):
        samples = [f.groups["joints_local"][joint].quaternion for f in selected
                   if joint in f.groups.get("joints_local", {})]
        if len(samples) < minimum_samples:
            result["joints"][joint] = {"status": "insufficient_data", "sample_count": len(samples),
                                        "quaternion_xyzw": None}
            continue
        # Rotation.mean is invariant to quaternion sign, unlike component means.
        mean = Rotation.from_quat(np.stack(samples)).mean()
        distances = np.rad2deg((mean.inv() * Rotation.from_quat(np.stack(samples))).magnitude())
        p95 = float(np.percentile(distances, 95))
        result["joints"][joint] = {
            "status": "stable" if p95 <= stable_threshold_deg else "unstable",
            "quaternion_xyzw": mean.as_quat().tolist(), "sample_count": len(samples),
            "p95_deviation_deg": p95, "max_deviation_deg": float(np.max(distances)),
            "median_deviation_deg": float(np.median(distances)),
        }
    result["stable_joint_count"] = sum(x["status"] == "stable" for x in result["joints"].values())
    result["overall_status"] = "stable" if result["stable_joint_count"] == len(result["joints"]) else "warning"
    return result
