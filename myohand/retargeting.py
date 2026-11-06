"""Offline qpos construction for explicitly validated scalar joint mappings."""

from __future__ import annotations

import math

from motion_processing.joint_angles import unit_axis


def retarget_frame(model, mapping: dict, angles: dict[str, dict]):
    """Produce a MuJoCo qpos vector; never infer anatomy or muscle activation.

    Mapping must be marked validated, have a configured source axis, and target
    a one-DoF hinge/slide. Unmapped joints remain at model default qpos.
    """
    import mujoco

    data = mujoco.MjData(model)
    qpos = data.qpos.copy()
    applied = []
    for item in mapping.get("mappings", []):
        if item.get("mapping_status") != "validated":
            continue
        source = item["rokoko_joint"]
        target = item["myohand_joint"]
        unit_axis(item["axis"])
        row = angles.get(source)
        if row is None or row.get("axis_status") != "configured" or row.get("candidate_flexion_rad") in (None, ""):
            continue
        idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, target)
        if idx < 0 or model.jnt_type[idx] not in (mujoco.mjtJoint.mjJNT_HINGE,
                                                  mujoco.mjtJoint.mjJNT_SLIDE):
            raise ValueError(f"Mapping target tidak valid/1-DoF: {target}")
        sign = float(item["sign"])
        scale = float(item["scale"])
        offset = float(item["offset"])
        if sign not in (-1, 1) or not all(math.isfinite(x) for x in (scale, offset)):
            raise ValueError(f"Parameter mapping tidak valid: {target}")
        q = sign * scale * float(row["candidate_flexion_rad"]) + offset
        if model.jnt_limited[idx]:
            lower, upper = model.jnt_range[idx]
            q = min(max(q, lower), upper)
        qpos[model.jnt_qposadr[idx]] = q
        applied.append(target)
    return qpos, applied
