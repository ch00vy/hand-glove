"""Candidate joint angles from calibrated local rotations."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import relative_rotation


def unit_axis(axis) -> np.ndarray:
    arr = np.asarray(axis, dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)) or np.linalg.norm(arr) < 1e-8:
        raise ValueError("Axis harus vektor 3D finite nonzero")
    return arr / np.linalg.norm(arr)


def hinge_angle(rotation: Rotation, axis) -> float:
    """Signed twist angle about an explicitly known local hinge axis."""
    a = unit_axis(axis)
    q = rotation.as_quat()
    angle = 2 * math.atan2(float(np.dot(q[:3], a)), float(q[3]))
    return (angle + math.pi) % (2 * math.pi) - math.pi


def infer_candidate_axes(recording, calibration, min_motion_deg=10.0,
                         min_dominance=0.8) -> dict:
    """Principal rotvec direction is exploratory, not an anatomical axis."""
    axes = {}
    for joint, info in calibration["joints"].items():
        if joint.endswith("_tip") or joint in ("right_fore_arm", "right_hand"):
            axes[joint] = {"status": "unmapped", "reason": "tip/root/wrist excluded"}
            continue
        if info["quaternion_xyzw"] is None:
            axes[joint] = {"status": "unmapped", "reason": "no calibration"}
            continue
        vectors = np.stack([relative_rotation(info["quaternion_xyzw"],
                                              f.groups["joints_local"][joint].quaternion).as_rotvec()
                            for f in recording.sorted_frames() if joint in f.groups.get("joints_local", {})])
        if len(vectors) < 10:
            axes[joint] = {"status": "unmapped", "reason": "too few samples"}
            continue
        centered = vectors - vectors.mean(axis=0)
        _, singular, vh = np.linalg.svd(centered, full_matrices=False)
        variance = singular ** 2
        dominance = float(variance[0] / variance.sum()) if variance.sum() else 0
        motion_deg = float(np.rad2deg(np.ptp(vectors @ vh[0])))
        if motion_deg < min_motion_deg or dominance < min_dominance:
            axes[joint] = {"status": "unmapped", "dominance": dominance,
                           "motion_range_deg": motion_deg,
                           "reason": "insufficient motion or no dominant rotational axis"}
        else:
            axis = vh[0]
            # Deterministic sign: dominant absolute component positive.
            if axis[np.argmax(np.abs(axis))] < 0:
                axis = -axis
            axes[joint] = {"status": "candidate_pca", "axis": axis.tolist(),
                           "dominance": dominance, "motion_range_deg": motion_deg,
                           "reason": "data-derived axis; anatomical sign/name unvalidated"}
    return axes


def angle_rows(recording, calibration, axis_config: dict, inferred: dict):
    """Return rows; optional explicit config uses local axes and ordered twist decomposition.

    For flexion axis a and abduction axis b: R_delta = R_flex(a) * R_residual;
    abduction is twist of residual about b. Large coupled rotations require
    anatomical calibration; these remain candidate angles.
    """
    first_ts = min(recording.frames)
    last_by_joint = {}
    for frame in recording.sorted_frames():
        for joint in sorted(recording.group_joints.get("joints_local", set())):
            measurement = frame.groups.get("joints_local", {}).get(joint)
            reference = calibration["joints"].get(joint, {})
            configured = axis_config.get(joint, {})
            info = inferred.get(joint, {})
            flex = abd = None
            status = reference.get("status", "missing")
            if measurement is not None and reference.get("quaternion_xyzw") is not None:
                delta = relative_rotation(reference["quaternion_xyzw"], measurement.quaternion)
                axis = configured.get("flexion_axis") or info.get("axis")
                if axis is not None:
                    a = unit_axis(axis)
                    raw = hinge_angle(delta, a)
                    previous = last_by_joint.get(joint)
                    if previous is not None:
                        raw += round((previous - raw) / (2 * math.pi)) * 2 * math.pi
                    last_by_joint[joint] = raw
                    flex = raw
                    if configured.get("abduction_axis") is not None:
                        b = unit_axis(configured["abduction_axis"])
                        if abs(np.dot(a, b)) > 0.95:
                            raise ValueError(f"Axis flexion dan abduction hampir sejajar: {joint}")
                        residual = Rotation.from_rotvec(a * raw).inv() * delta
                        abd = hinge_angle(residual, b)
                else:
                    status += ";axis_unmapped"
            else:
                status += ";measurement_missing"
            yield {"timestamp": frame.timestamp_us, "time_s": (frame.timestamp_us-first_ts)/1e6,
                   "joint_name": joint, "candidate_flexion_rad": flex, "candidate_abduction_rad": abd,
                   "candidate_flexion_deg": math.degrees(flex) if flex is not None else None,
                   "candidate_abduction_deg": math.degrees(abd) if abd is not None else None,
                   "calibration_status": status,
                   "axis_status": "configured" if configured.get("flexion_axis") else info.get("status", "unmapped")}


def write_angles(rows, path: Path) -> None:
    names = ("timestamp", "time_s", "joint_name", "candidate_flexion_rad",
             "candidate_abduction_rad", "candidate_flexion_deg", "candidate_abduction_deg",
             "calibration_status", "axis_status")
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)
