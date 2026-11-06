"""World-space geometry checks and offline diagnostic plots."""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import relative_rotation


def world_geometry(recording) -> tuple[dict, dict[str, np.ndarray]]:
    frames = recording.sorted_frames()
    start = frames[0].timestamp_us
    times = np.array([(f.timestamp_us - start) / 1e6 for f in frames])
    distances = {}
    for finger in range(1, 6):
        name = f"right_finger{finger}_tip"
        values = []
        for frame in frames:
            group = frame.groups.get("joints_world", {})
            hand = group.get("right_hand")
            tip = group.get(name)
            values.append(float(np.linalg.norm(tip.position - hand.position))
                          if hand is not None and tip is not None and
                          hand.position is not None and tip.position is not None else np.nan)
        distances[name] = np.array(values)
    first_mask = times <= 0.5
    last_mask = times >= times[-1] - 0.5
    summary = {"reference": "right_hand world position; Euclidean distances in model units (SDK uses meters)",
               "world_frame": "LTP_NED", "fingertips": {}}
    for name, values in distances.items():
        finite = values[np.isfinite(values)]
        if len(finite) == 0:
            summary["fingertips"][name] = {"status": "missing"}
            continue
        initial_values = values[first_mask & np.isfinite(values)]
        final_values = values[last_mask & np.isfinite(values)]
        initial = float(np.median(initial_values)) if len(initial_values) else None
        final = float(np.median(final_values)) if len(final_values) else None
        minimum = float(np.nanmin(values))
        summary["fingertips"][name] = {
            "initial_m": initial, "minimum_m": minimum, "final_m": final,
            "maximum_contraction_m": initial - minimum if initial is not None else None,
            "final_minus_initial_m": final - initial if initial is not None and final is not None else None,
            "initial_stability_std_m": float(np.std(initial_values)) if len(initial_values) else None,
            "missing_samples": int(np.count_nonzero(~np.isfinite(values))),
        }
    return summary, {"times": times, **distances}


def _hand_local_positions(frame) -> dict[str, np.ndarray]:
    world = frame.groups.get("joints_world", {})
    hand = world.get("right_hand")
    if hand is None or hand.position is None:
        return {}
    transform = Rotation.from_quat(hand.quaternion).inv()
    return {name: transform.apply(m.position - hand.position) for name, m in world.items()
            if m.position is not None}


def make_plots(recording, calibration, angle_rows: list[dict], geometry_series,
               hierarchy, output_dir: Path) -> list[str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    frames = recording.sorted_frames()
    times = geometry_series["times"]
    saved = []

    fig, ax = plt.subplots(figsize=(10, 5))
    for finger in range(1, 6):
        name = f"right_finger{finger}_tip"
        ax.plot(times, geometry_series[name], label=f"Finger {finger}")
    ax.set(xlabel="Time (s)", ylabel="Tip-to-hand distance (m)",
           title="World-space fingertip distance from hand")
    ax.legend(ncol=5)
    fig.tight_layout()
    fig.savefig(output_dir / "fingertip_distances.png", dpi=150)
    saved.append("fingertip_distances.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    for finger in range(1, 6):
        joint = f"right_finger{finger}_proximal"
        ref = calibration["joints"].get(joint, {}).get("quaternion_xyzw")
        if ref is None:
            continue
        values = [np.rad2deg(relative_rotation(ref, f.groups["joints_local"][joint].quaternion).magnitude())
                  if joint in f.groups.get("joints_local", {}) else np.nan for f in frames]
        ax.plot(times, values, label=f"Finger {finger}")
    ax.set(xlabel="Time (s)", ylabel="Neutral-relative rotation magnitude (deg)",
           title="Proximal orientation change (not an anatomical joint angle)")
    ax.legend(ncol=5)
    fig.tight_layout()
    fig.savefig(output_dir / "orientation_change.png", dpi=150)
    saved.append("orientation_change.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    for finger in range(1, 6):
        joint = f"right_finger{finger}_proximal"
        rows = [r for r in angle_rows if r["joint_name"] == joint]
        if rows and any(r["candidate_flexion_deg"] is not None for r in rows):
            ax.plot([r["time_s"] for r in rows],
                    [r["candidate_flexion_deg"] if r["candidate_flexion_deg"] is not None else np.nan
                     for r in rows], label=f"Finger {finger}")
    ax.set(xlabel="Time (s)", ylabel="Candidate hinge angle (deg)",
           title="Data-derived proximal candidate flexion; sign/anatomy unvalidated")
    if ax.lines:
        ax.legend(ncol=5)
    else:
        ax.text(.5, .5, "No candidate axes passed motion/dominance checks",
                transform=ax.transAxes, ha="center")
    fig.tight_layout()
    fig.savefig(output_dir / "candidate_flexion.png", dpi=150)
    saved.append("candidate_flexion.png")
    plt.close(fig)

    initial = _hand_local_positions(frames[0])
    final = _hand_local_positions(frames[-1])
    fig = plt.figure(figsize=(10, 5))
    for subplot, (title, points) in enumerate((("First frame", initial), ("Last frame", final)), 1):
        ax = fig.add_subplot(1, 2, subplot, projection="3d")
        for joint, info in hierarchy.items():
            parent = info["parent"]
            if joint in points and parent in points:
                p, q = points[joint], points[parent]
                ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], color="tab:blue")
        if points:
            xyz = np.stack(list(points.values()))
            ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], s=8, color="tab:orange")
            limit = float(np.max(np.abs(xyz))) or .1
            ax.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(-limit, limit))
        ax.set_title(title + " (hand-relative world position)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
    fig.tight_layout()
    fig.savefig(output_dir / "first_last_pose.png", dpi=150)
    saved.append("first_last_pose.png")
    plt.close(fig)
    return saved
