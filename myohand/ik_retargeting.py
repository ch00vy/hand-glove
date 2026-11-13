"""Landmark inverse kinematics for an offline, position-controlled myoHand pose."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Landmark:
    rokoko: str
    object_type: str
    object_id: int
    is_tip: bool


def load_landmarks(model, path: str | Path) -> tuple[int, list[Landmark]]:
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    reference = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config["reference_body"])
    if reference < 0:
        raise ValueError(f"Body referensi model tidak ada: {config['reference_body']}")
    landmarks = []
    seen = set()
    for row in config["landmarks"]:
        if ("body" in row) == ("site" in row):
            raise ValueError(f"Landmark harus menyebut tepat satu body/site: {row}")
        kind = "site" if "site" in row else "body"
        name = row[kind]
        object_type = mujoco.mjtObj.mjOBJ_SITE if kind == "site" else mujoco.mjtObj.mjOBJ_BODY
        object_id = mujoco.mj_name2id(model, object_type, name)
        if object_id < 0 or row["rokoko"] in seen:
            raise ValueError(f"Landmark model tidak ada atau Rokoko duplikat: {row}")
        seen.add(row["rokoko"])
        landmarks.append(Landmark(row["rokoko"], kind, object_id, row["rokoko"].endswith("_tip")))
    if len(landmarks) < 5 or sum(x.is_tip for x in landmarks) != 5:
        raise ValueError("Diperlukan lima landmark fingertip dan beberapa ruas")
    return reference, landmarks


def source_points(frame, landmarks: list[Landmark]) -> np.ndarray | None:
    """Rokoko world positions expressed in its current hand frame."""
    world = frame.groups.get("joints_world", {})
    hand = world.get("right_hand")
    if hand is None or hand.position is None:
        return None
    inverse = Rotation.from_quat(hand.quaternion).inv()
    points = []
    for landmark in landmarks:
        measurement = world.get(landmark.rokoko)
        if measurement is None or measurement.position is None:
            return None
        points.append(inverse.apply(measurement.position - hand.position))
    return np.stack(points)


def model_points(model, data, reference_body: int, landmarks: list[Landmark]) -> np.ndarray:
    """Model body origins or tip sites, expressed in capitate frame."""
    origin = data.xpos[reference_body]
    inverse = data.xmat[reference_body].reshape(3, 3).T
    return np.stack([inverse @ ((data.site_xpos[x.object_id] if x.object_type == "site"
                                 else data.xpos[x.object_id]) - origin) for x in landmarks])


def register_neutral(source: np.ndarray, model: np.ndarray) -> dict:
    """Least-squares proper rotation and uniform scale; no reflection."""
    if source.shape != model.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("Bentuk landmark neutral tidak sesuai")
    source_mean, model_mean = source.mean(axis=0), model.mean(axis=0)
    a, b = source - source_mean, model - model_mean
    u, singular, vt = np.linalg.svd(a.T @ b)
    correction = np.diag([1.0, 1.0, np.linalg.det(vt.T @ u.T)])
    rotation = vt.T @ correction @ u.T
    scale = float(np.sum(singular * np.diag(correction)) / np.sum(a * a))
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("Registrasi netral degenerat")
    translation = model_mean - scale * rotation @ source_mean
    aligned = (scale * (rotation @ source.T)).T + translation
    error = np.linalg.norm(aligned - model, axis=1)
    return {"rotation": rotation, "scale": scale, "translation": translation,
            "neutral_shape_rmse_m": float(np.sqrt(np.mean(error ** 2))),
            "neutral_shape_per_landmark_m": error.tolist()}


class KinematicFitter:
    """Fit finger qpos with MuJoCo FK and analytic position Jacobians.

    Wrist remains at the model default. A neutral hand shape anchors the
    landmarks; scaled, registered Rokoko *displacements* drive finger motion.
    """

    def __init__(self, model, reference_body: int, landmarks: list[Landmark],
                 neutral_source: np.ndarray, registration: dict,
                 temporal_weight_m: float = 0.001, neutral_weight_m: float = 0.0005,
                 tip_weight: float = 2.0):
        self.model = model
        self.data = mujoco.MjData(model)
        self.reference_body = reference_body
        self.landmarks = landmarks
        self.neutral_source = neutral_source
        self.registration = registration
        self.temporal_weight_m = temporal_weight_m
        self.neutral_weight_m = neutral_weight_m
        if tip_weight <= 0 or temporal_weight_m < 0 or neutral_weight_m < 0:
            raise ValueError("Bobot optimisasi tidak valid")
        self.marker_weights = np.array([tip_weight if x.is_tip else 1.0 for x in landmarks])
        self.default_qpos = self.data.qpos.copy()
        self.joint_ids = [i for i in range(model.njnt) if model.joint(i).name not in
                          ("pro_sup", "deviation", "flexion")]
        if len(self.joint_ids) != model.njnt - 3 or any(model.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE
                                                       for i in self.joint_ids):
            raise ValueError("Model harus memiliki 3 joint pergelangan dan sisanya hinge jari")
        body = reference_body
        while body > 0:
            first = int(model.body_jntadr[body])
            count = int(model.body_jntnum[body])
            if any(i in self.joint_ids for i in range(first, first + count)):
                raise ValueError("Body referensi harus tetap saat joint jari bergerak")
            body = int(model.body_parentid[body])
        self.qadr = np.array([model.jnt_qposadr[i] for i in self.joint_ids], dtype=int)
        self.dadr = np.array([model.jnt_dofadr[i] for i in self.joint_ids], dtype=int)
        self.lower = np.array([model.jnt_range[i, 0] if model.jnt_limited[i] else -np.pi
                               for i in self.joint_ids])
        self.upper = np.array([model.jnt_range[i, 1] if model.jnt_limited[i] else np.pi
                               for i in self.joint_ids])
        self.data.qpos[:] = self.default_qpos
        mujoco.mj_forward(model, self.data)
        self.neutral_model = model_points(model, self.data, reference_body, landmarks)
        self.base_rotation_inv = self.data.xmat[reference_body].reshape(3, 3).T.copy()
        self._last_q = None

    def target_points(self, source: np.ndarray) -> np.ndarray:
        difference = source - self.neutral_source
        return self.neutral_model + self.registration["scale"] * (
            self.registration["rotation"] @ difference.T).T

    def evaluate(self, finger_qpos: np.ndarray) -> np.ndarray:
        if self._last_q is None or not np.array_equal(finger_qpos, self._last_q):
            self.data.qpos[:] = self.default_qpos
            self.data.qpos[self.qadr] = finger_qpos
            mujoco.mj_forward(self.model, self.data)
            self._last_q = finger_qpos.copy()
        return model_points(self.model, self.data, self.reference_body, self.landmarks)

    def _jacobian(self) -> np.ndarray:
        jac = np.empty((len(self.landmarks) * 3, len(self.joint_ids)))
        for i, landmark in enumerate(self.landmarks):
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            if landmark.object_type == "site":
                mujoco.mj_jacSite(self.model, self.data, jacp, jacr, landmark.object_id)
            else:
                mujoco.mj_jacBody(self.model, self.data, jacp, jacr, landmark.object_id)
            jac[3*i:3*i+3] = self.base_rotation_inv @ jacp[:, self.dadr]
        return jac

    def fit(self, target: np.ndarray, previous: np.ndarray, max_evaluations: int = 25):
        if target.shape != self.neutral_model.shape:
            raise ValueError("Target landmark tidak lengkap")
        previous = np.clip(previous, self.lower, self.upper)
        def residual(x):
            points = self.evaluate(x)
            return np.concatenate(((((points - target) * self.marker_weights[:, None])).ravel(),
                                   self.temporal_weight_m * (x - previous),
                                   self.neutral_weight_m * x))
        def jacobian(x):
            self.evaluate(x)
            return np.vstack((self._jacobian() * np.repeat(self.marker_weights, 3)[:, None],
                              self.temporal_weight_m * np.eye(len(x)),
                              self.neutral_weight_m * np.eye(len(x))))
        started = time.perf_counter()
        result = least_squares(residual, previous, jac=jacobian, bounds=(self.lower, self.upper),
                               max_nfev=max_evaluations, ftol=1e-8, xtol=1e-8, gtol=1e-9)
        ik_compute_ms = (time.perf_counter() - started) * 1000
        fitted = self.evaluate(result.x)
        error = np.linalg.norm(fitted - target, axis=1)
        return result.x, error, {
            "success": bool(result.success), "status": int(result.status),
            "message": str(result.message), "nfev": int(result.nfev),
            "njev": int(result.njev) if result.njev is not None else None,
            "cost": float(result.cost), "optimality": float(result.optimality),
            # Only least_squares, including its residual/Jacobian calls; excludes
            # target construction, final error computation, queues and rendering.
            "ik_compute_ms": ik_compute_ms,
        }

    def full_qpos(self, finger_qpos: np.ndarray) -> np.ndarray:
        output = self.default_qpos.copy()
        output[self.qadr] = finger_qpos
        return output
