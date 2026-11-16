"""Orientation FK fitting, independent of the existing finger IK.

RGMP rotations are active, scalar-last xyzw; MuJoCo xmat is used directly.
Only an atomic world group is consumed live. Local extraction is provided
for validation, never silently mixed with another timestamp/device.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINT_NAMES = ('pro_sup', 'deviation', 'flexion')


@dataclass(frozen=True)
class Orientations:
    forearm: Rotation
    hand: Rotation
    reference: str


def extract_orientations(frame, group='joints_world') -> Orientations:
    measurements = frame.groups.get(group, {})
    forearm = measurements.get('right_fore_arm')
    hand = measurements.get('right_hand')
    if forearm is None or hand is None:
        raise ValueError('Orientasi right_fore_arm/right_hand tidak lengkap')
    if not forearm.parent:
        raise ValueError('reference_frame forearm kosong')
    f = Rotation.from_quat(forearm.quaternion)
    h = Rotation.from_quat(hand.quaternion)
    if group == 'joints_local':
        if hand.parent != 'right_fore_arm':
            raise ValueError('Parent local hand harus right_fore_arm')
        h = f * h
    elif group != 'joints_world' or hand.parent != forearm.parent:
        raise ValueError('Orientasi world harus memiliki reference_frame yang sama')
    return Orientations(f, h, forearm.parent)


class OrientationCalibration:
    """Rolling geodesic stability check and antipodal-invariant SO(3) mean."""
    def __init__(self, window_s=.5, min_frames=15, stability_deg=3.):
        if window_s <= 0 or min_frames < 2 or stability_deg <= 0:
            raise ValueError('Parameter kalibrasi wrist tidak valid')
        self.window_us = int(window_s * 1e6)
        self.min_frames = min_frames
        self.stability_deg = stability_deg
        self.samples = deque()
        self.neutral = None
        self.stability_max_deg = None

    def add(self, timestamp_us, orientation):
        # A previously stable window must not remain eligible after motion,
        # a data gap, or a reference change while finger calibration is pending.
        self.neutral = None
        self.stability_max_deg = None
        if self.samples and (orientation.reference != self.samples[-1][1].reference or
                             timestamp_us <= self.samples[-1][0] or
                             timestamp_us - self.samples[-1][0] > self.window_us):
            self.samples.clear()
        self.samples.append((timestamp_us, orientation))
        while self.samples and timestamp_us - self.samples[0][0] > self.window_us:
            self.samples.popleft()
        if (len(self.samples) < self.min_frames or
                timestamp_us - self.samples[0][0] < .8 * self.window_us):
            return None
        stacks = [Rotation.from_quat(np.stack([getattr(o, key).as_quat()
                  for _, o in self.samples])) for key in ('forearm', 'hand')]
        means = [r.mean() for r in stacks]
        self.stability_max_deg = max(float(np.rad2deg((mean.inv() * r).magnitude()).max())
                           for mean, r in zip(means, stacks))
        # Conservative maximum prevents short spikes being hidden by a percentile.
        if self.stability_max_deg > self.stability_deg:
            return None
        self.neutral = Orientations(*means, orientation.reference)
        return self.neutral


class WristRetargeter:
    """Fit wrist relative orientation and optional forearm axial twist proxy.

    `relative` fixes pro_sup at neutral. `forearm-twist` projects the measured
    forearm change onto the FK pro_sup axis. It cannot separate shoulder/elbow
    contributions without an upper-arm reference. No global swing is fitted.
    """
    def __init__(self, model, hand_body, source_to_model, neutral,
                 mode='relative', temporal_weight=.01, max_evaluations=20):
        if mode not in ('relative', 'forearm-twist'):
            raise ValueError('Mode wrist tidak dikenal')
        if temporal_weight < 0 or max_evaluations < 1:
            raise ValueError('Parameter optimizer wrist tidak valid')
        self.model, self.hand_body = model, hand_body
        self.data = mujoco.MjData(model)
        self.default = self.data.qpos.copy()
        self.jids = np.array([model.joint(name).id for name in JOINT_NAMES])
        if any(model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE or not model.jnt_limited[j]
               for j in self.jids):
            raise ValueError('Wrist memerlukan tiga limited hinge dari model')
        self.qadr = model.jnt_qposadr[self.jids].copy()
        self.lower, self.upper = model.jnt_range[self.jids].T.copy()
        self.forearm_body = int(model.jnt_bodyid[self.jids[0]])
        chain = []
        body = hand_body
        while body:
            chain.append(body)
            body = int(model.body_parentid[body])
        if any(int(model.jnt_bodyid[j]) not in chain for j in self.jids):
            raise ValueError('Ketiga joint harus ancestor body hand')
        mujoco.mj_forward(model, self.data)
        self.f0, self.h0 = self._rotations()
        self.rel0 = self.f0.inv() * self.h0
        self.c = Rotation.from_matrix(source_to_model)
        self.mode, self.neutral = mode, neutral
        self.temporal_weight, self.max_evaluations = temporal_weight, max_evaluations
        self.previous = self.default[self.qadr].copy()
        # Transform a source forearm-local delta into model world coordinates.
        self.forearm_alignment = self.h0 * self.c * neutral.hand.inv() * neutral.forearm
        self.axis = self.data.xaxis[self.jids[0]].copy()
        # Diagnostic projections only; joint control still uses FK optimization.
        self.hand_local_axes = self.h0.inv().apply(self.data.xaxis[self.jids].copy())
        self.last_twist = 0.

    def _rotations(self):
        return tuple(Rotation.from_matrix(self.data.xmat[b].reshape(3, 3).copy())
                     for b in (self.forearm_body, self.hand_body))

    def targets(self, current):
        if current.reference != self.neutral.reference:
            raise ValueError('reference_frame berubah; kalibrasi wrist ulang diperlukan')
        rel = current.forearm.inv() * current.hand
        source_rel0 = self.neutral.forearm.inv() * self.neutral.hand
        target_rel = self.rel0 * self.c * source_rel0.inv() * rel * self.c.inv()
        twist = 0.
        if self.mode == 'forearm-twist':
            delta = (self.forearm_alignment * self.neutral.forearm.inv() *
                     current.forearm * self.forearm_alignment.inv())
            quat = delta.as_quat()
            axial = float(quat[:3] @ self.axis)
            if np.hypot(axial, quat[3]) < 1e-8:
                raise ValueError('Twist forearm ambigu pada swing 180 derajat')
            wrapped = 2 * np.arctan2(axial, quat[3])
            twist = self.last_twist + (wrapped - self.last_twist + np.pi) % (2*np.pi) - np.pi
        return target_rel, twist

    def fit(self, current):
        started = time.perf_counter()
        target_rel, twist = self.targets(current)
        previous = self.previous.copy()
        active = np.array([0, 1, 2] if self.mode == 'forearm-twist' else [1, 2])
        def residual(x):
            q = self.default[self.qadr].copy()
            q[active] = x
            self.data.qpos[:] = self.default
            self.data.qpos[self.qadr] = q
            mujoco.mj_forward(self.model, self.data)
            f, h = self._rotations()
            rotation_error = (target_rel.inv() * f.inv() * h).as_rotvec()
            parts = [rotation_error, self.temporal_weight * (x - previous[active])]
            if self.mode == 'forearm-twist':
                parts.append(np.array([q[0] - self.default[self.qadr[0]] - twist]))
            return np.concatenate(parts)
        result = least_squares(residual, np.clip(previous[active], self.lower[active], self.upper[active]),
                               bounds=(self.lower[active], self.upper[active]),
                               max_nfev=self.max_evaluations, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        self.previous = self.default[self.qadr].copy()
        self.previous[active] = result.x
        residual(result.x)
        f, h = self._rotations()
        rel_error = float(np.rad2deg((target_rel.inv() * f.inv() * h).magnitude()))
        twist_error = float(np.rad2deg(self.previous[0] - self.default[self.qadr[0]] - twist))
        target_hand = Rotation.from_rotvec(self.axis * twist) * self.f0 * target_rel
        error = float(np.rad2deg((target_hand.inv() * h).magnitude()))
        self.last_twist = twist
        metrics = {'wrist_orientation_error_deg': error,
                   'wrist_relative_error_deg': rel_error,
                   'forearm_twist_error_deg': abs(twist_error),
                   'wrist_compute_ms': (time.perf_counter() - started) * 1000,
                   'wrist_optimizer_success': bool(result.success),
                   'wrist_optimizer_status': int(result.status),
                   'wrist_mode': self.mode,
                   'wrist_observability': ('relative_only' if self.mode == 'relative'
                                           else 'forearm_twist_proxy'),
                   **{f'{name}_angle_deg': float(np.rad2deg(q))
                      for name, q in zip(JOINT_NAMES, self.previous)}}
        relative_delta = self.rel0.inv() * target_rel
        components = np.rad2deg(self.hand_local_axes @ relative_delta.as_rotvec())
        metrics.update({
            'wrist_input_forearm_delta_deg': float(np.rad2deg(
                (self.neutral.forearm.inv() * current.forearm).magnitude())),
            'wrist_input_hand_delta_deg': float(np.rad2deg(
                (self.neutral.hand.inv() * current.hand).magnitude())),
            'wrist_input_relative_deg': float(np.rad2deg(relative_delta.magnitude())),
            'wrist_input_forearm_twist_deg': float(np.rad2deg(twist)),
            'wrist_input_deviation_component_deg': float(components[1]),
            'wrist_input_flexion_component_deg': float(components[2]),
            **{f'{name}_at_limit': bool(min(q - lo, hi - q) < np.deg2rad(.5))
               for name, q, lo, hi in zip(JOINT_NAMES, self.previous, self.lower, self.upper)},
        })
        return self.previous.copy(), metrics

    def merge_qpos(self, finger_qpos):
        """Merge only wrist addresses into the complete finger output."""
        output = np.array(finger_qpos, copy=True)
        output[self.qadr] = self.previous
        return output
