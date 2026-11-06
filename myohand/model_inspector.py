"""Inspect the installed MyoSuite myohand.xml using MuJoCo's compiled model."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from xml.etree import ElementTree


def default_model_path() -> Path | None:
    spec = importlib.util.find_spec("myosuite")
    if spec is None or spec.origin is None:
        return None
    path = Path(spec.origin).parent / "simhive/myo_sim/hand/myohand.xml"
    return path if path.is_file() else None


def inspect_model(path: str | Path) -> tuple[dict, object]:
    import mujoco

    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    joints = []
    for idx in range(model.njnt):
        jtype = int(model.jnt_type[idx])
        dof = {mujoco.mjtJoint.mjJNT_FREE: 6, mujoco.mjtJoint.mjJNT_BALL: 3,
               mujoco.mjtJoint.mjJNT_SLIDE: 1, mujoco.mjtJoint.mjJNT_HINGE: 1}[jtype]
        body_id = int(model.jnt_bodyid[idx])
        parent_id = int(model.body_parentid[body_id])
        joints.append({
            "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, idx),
            "type": mujoco.mjtJoint(jtype).name.replace("mjJNT_", "").lower(),
            "axis": model.jnt_axis[idx].tolist(),
            "range": model.jnt_range[idx].tolist() if model.jnt_limited[idx] else None,
            "qpos_address": int(model.jnt_qposadr[idx]),
            "qvel_address": int(model.jnt_dofadr[idx]), "dof": dof,
            "default_qpos": float(data.qpos[model.jnt_qposadr[idx]]) if dof == 1 else None,
            "body": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
            "parent_body": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
            if parent_id > 0 else "world",
        })
    return {"model_path": str(path), "model_name": ElementTree.parse(path).getroot().get("model"),
            "joint_count": model.njnt,
            "nq": model.nq, "nv": model.nv, "joints": joints}, model


def mapping_template(inventory: dict) -> dict:
    return {"model_path": inventory["model_path"],
            "status": "unmapped; requires axis alignment, sign, scale, offset and kinematic validation",
            "mappings": [{"rokoko_joint": None, "myohand_joint": joint["name"], "axis": None,
                          "sign": None, "offset": None, "scale": None,
                          "joint_range": joint["range"], "mapping_status": "unmapped"}
                         for joint in inventory["joints"]]}
